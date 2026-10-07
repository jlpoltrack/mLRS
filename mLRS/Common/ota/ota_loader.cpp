//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// OTA Loader
// lives in low flash, owns the reset vector, receives the app over the air
//*******************************************************
// Must be fully self-contained: no calls into the app, no libc, no .data/.bss.
// The linker script places .text/.rodata of this object into the loader region.
//*******************************************************

#if defined RX_MATEK_MR900_30_G431KB || defined RX_MATEK_MR900_30C_G431KB || \
    defined RX_MATEK_MR24_30_G431KB || defined RX_MATEK_MR24_30C_G431KB

#pragma GCC optimize ("no-tree-loop-distribute-patterns") // don't let gcc emit memcpy/memset calls

#include "stm32g4xx.h"
#include "../hal/device_conf.h" // only for the radio chip, DEVICE_NAME and VERSION
#ifdef DEVICE_HAS_SX128x
#include "../../modules/sx12xx-lib/src/sx128x.h" // only for the constants
#else
#include "../../modules/sx12xx-lib/src/sx126x.h" // only for the constants
#endif
#include "ota_loader.h"

#ifndef OTA_LOADER_BASE
  #error OTA loader: mcu not supported!
#endif


//-------------------------------------------------------
// Board
//-------------------------------------------------------
// pins must match hal-matek-mr-g431kb-common.h

#define OTA_IDLE_TIMEOUT_MS       30000 // leave if a valid app is present and no tx shows up

#define LED_RED_PIN               0 // PA0
#define LED_GREEN_PIN             1 // PA1
#define SX_CS_PIN                 4 // PA4
#define SX_FAN_PIN                8 // PA8
#define SX_DIO_PIN                15 // PA15
#define SX_RX_EN_PIN              0 // PB0
#define SX_BUSY_PIN               5 // PB5
#define SX_RESET_PIN              6 // PB6
#define SX_TX_EN_PIN              7 // PB7

#define SX_IRQ_MASK               (OTA_SX(IRQ_TX_DONE) | OTA_SX(IRQ_RX_DONE) | OTA_SX(IRQ_RX_TX_TIMEOUT) | \
                                   OTA_SX(IRQ_CRC_ERROR) | OTA_SX(IRQ_HEADER_ERROR))


//-------------------------------------------------------
// Low level helpers
//-------------------------------------------------------

static inline void pin_high(GPIO_TypeDef* gpio, uint8_t pin) { gpio->BSRR = (1u << pin); }
static inline void pin_low(GPIO_TypeDef* gpio, uint8_t pin) { gpio->BRR = (1u << pin); }
static inline bool pin_read(GPIO_TypeDef* gpio, uint8_t pin) { return (gpio->IDR & (1u << pin)) != 0; }

// mode: 0 = input, 1 = output, 2 = alternate function, pull: 0 = none, 1 = up, 2 = down
static void pin_init(GPIO_TypeDef* gpio, uint8_t pin, uint8_t mode, uint8_t pull)
{
    gpio->PUPDR = (gpio->PUPDR & ~(3u << (pin * 2))) | ((uint32_t)pull << (pin * 2));
    gpio->MODER = (gpio->MODER & ~(3u << (pin * 2))) | ((uint32_t)mode << (pin * 2));
}

// SysTick runs at 1 ms, is polled, no isr
static inline bool tick_1ms(void) { return (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0; }

static void delay_ms(uint32_t ms)
{
    while (ms) { if (tick_1ms()) ms--; }
}

static uint8_t spi_byte(uint8_t b)
{
    while (!(SPI1->SR & SPI_SR_TXE)) {}
    *(volatile uint8_t*)&SPI1->DR = b;
    while (!(SPI1->SR & SPI_SR_RXNE)) {}
    return *(volatile uint8_t*)&SPI1->DR;
}

static void hw_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    (void)RCC->APB2ENR;

    FLASH->ACR &= ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN); // we modify flash, so no caches

    SysTick->LOAD = 16000 - 1; // HSI16 is the clock after reset
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    pin_low(GPIOA, LED_RED_PIN); pin_init(GPIOA, LED_RED_PIN, 1, 0);
    pin_low(GPIOA, LED_GREEN_PIN); pin_init(GPIOA, LED_GREEN_PIN, 1, 0);
    pin_low(GPIOA, SX_FAN_PIN); pin_init(GPIOA, SX_FAN_PIN, 1, 0);

    pin_high(GPIOA, SX_CS_PIN); pin_init(GPIOA, SX_CS_PIN, 1, 0);
    pin_init(GPIOA, SX_DIO_PIN, 0, 2);
    pin_low(GPIOB, SX_RX_EN_PIN); pin_init(GPIOB, SX_RX_EN_PIN, 1, 0);
    pin_low(GPIOB, SX_TX_EN_PIN); pin_init(GPIOB, SX_TX_EN_PIN, 1, 0);
    pin_high(GPIOB, SX_RESET_PIN); pin_init(GPIOB, SX_RESET_PIN, 1, 0);
    pin_init(GPIOB, SX_BUSY_PIN, 0, 1);

    // SPI1 on PA5, PA6, PA7, AF5, mode 0, 4 MHz
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~0xFFF00000) | 0x55500000;
    GPIOA->OSPEEDR |= (2u << (5 * 2)) | (2u << (7 * 2));
    pin_init(GPIOA, 5, 2, 0);
    pin_init(GPIOA, 6, 2, 0);
    pin_init(GPIOA, 7, 2, 0);
    SPI1->CR2 = (7u << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH; // 8 bit
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (1u << SPI_CR1_BR_Pos) | SPI_CR1_SPE;
}


//-------------------------------------------------------
// Flash
//-------------------------------------------------------

#define FLASH_SR_ERRORS  (FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | \
                          FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR)

static bool flash_done(void)
{
    while (FLASH->SR & FLASH_SR_BSY) {}
    bool ok = (FLASH->SR & FLASH_SR_ERRORS) == 0;
    FLASH->SR = FLASH_SR_ERRORS | FLASH_SR_EOP;
    return ok;
}

static void flash_unlock(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123;
        FLASH->KEYR = 0xCDEF89AB;
    }
    flash_done();
}

static bool flash_erase_page(uint32_t adr)
{
    uint32_t page = (adr - 0x08000000) / OTA_FLASH_PAGE_SIZE;

    FLASH->CR = FLASH_CR_PER | (page << FLASH_CR_PNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    bool ok = flash_done();
    FLASH->CR = 0;
    return ok;
}

// len must be a multiple of 8, data may be unaligned
static bool flash_program(uint32_t adr, const uint8_t* data, uint16_t len)
{
    bool ok = true;

    FLASH->CR = FLASH_CR_PG;
    for (uint16_t n = 0; n < len; n += 4) {
        uint32_t w = data[n] | ((uint32_t)data[n+1] << 8) | ((uint32_t)data[n+2] << 16) | ((uint32_t)data[n+3] << 24);
        *(volatile uint32_t*)(adr + n) = w;
        if (n & 4) ok &= flash_done(); // double word is complete
    }
    FLASH->CR = 0;

    for (uint16_t n = 0; n < len; n++) {
        if (*(volatile uint8_t*)(adr + n) != data[n]) ok = false;
    }
    return ok;
}


//-------------------------------------------------------
// App image and params
//-------------------------------------------------------

// zlib crc32, len must be a multiple of 4
static uint32_t crc32(uint32_t adr, uint32_t len)
{
    uint32_t ahb1enr = RCC->AHB1ENR;
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
    (void)RCC->AHB1ENR;

    CRC->CR = CRC_CR_RESET | CRC_CR_REV_IN | CRC_CR_REV_OUT; // word-wise bit reversal
    for (uint32_t n = 0; n < len; n += 4) CRC->DR = *(volatile uint32_t*)(adr + n);
    uint32_t crc = ~CRC->DR;

    RCC->AHB1ENR = ahb1enr; // leave things as found, the app may get started next
    return crc;
}

static bool app_is_valid(void)
{
    const tOtaAppInfo* info = (const tOtaAppInfo*)(OTA_APP_BASE + OTA_APP_INFO_OFFSET);

    if (info->magic != OTA_APP_INFO_MAGIC) return false;
    if (info->target_id != OTA_TARGET_ID) return false;
    // length 0 is an image as it comes out of the build, length and crc are filled in by the tool which sends it
    // it can't be crc checked, and can get here only by wire
    if (info->length == 0) return true;
    if (info->length < OTA_APP_INFO_OFFSET + sizeof(tOtaAppInfo) + 4) return false;
    if (info->length > OTA_APP_SIZE_MAX || (info->length & 7)) return false;

    return crc32(OTA_APP_BASE, info->length - 4) == *(volatile uint32_t*)(OTA_APP_BASE + info->length - 4);
}

static bool params_are_valid(void)
{
    const uint32_t* p = (const uint32_t*)OTA_PARAMS_BASE;

    return (p[0] == OTA_PARAMS_MAGIC) && (p[4] == ~(p[1] ^ p[2] ^ p[3]));
}

__attribute__((noreturn)) static void app_jump(void)
{
    uint32_t sp = *(volatile uint32_t*)(OTA_APP_BASE);
    uint32_t pc = *(volatile uint32_t*)(OTA_APP_BASE + 4);

    SCB->VTOR = OTA_APP_BASE;
    __asm volatile ("msr msp, %0 \n bx %1" : : "r" (sp), "r" (pc));
    while (1) {}
}

// for errors we can't do anything about, wired flashing is the way out
__attribute__((noreturn)) static void fail(void)
{
    while (1) { GPIOA->ODR ^= (1u << LED_RED_PIN); delay_ms(100); }
}


//-------------------------------------------------------
// SX126x, SX128x
//-------------------------------------------------------

static void sx_begin(uint8_t opcode)
{
    uint32_t guard = 1000000;
    while (pin_read(GPIOB, SX_BUSY_PIN) && --guard) {}
    pin_low(GPIOA, SX_CS_PIN);
    spi_byte(opcode);
}

static void sx_end(void)
{
    pin_high(GPIOA, SX_CS_PIN);
    for (volatile uint8_t n = 0; n < 30; n++) {} // give busy time to go high
}

static void sx_cmd(uint8_t opcode, const uint8_t* data, uint8_t len)
{
    sx_begin(opcode);
    while (len--) spi_byte(*data++);
    sx_end();
}

static void sx_cmd1(uint8_t opcode, uint8_t data) { sx_cmd(opcode, &data, 1); }

static void sx_cmd_read(uint8_t opcode, uint8_t* data, uint8_t len)
{
    sx_begin(opcode);
    spi_byte(0);
    while (len--) *data++ = spi_byte(0);
    sx_end();
}

static void sx_write_reg(uint16_t adr, uint8_t value)
{
    const uint8_t buf[3] = { (uint8_t)(adr >> 8), (uint8_t)adr, value };
    sx_cmd(OTA_SX(CMD_WRITE_REGISTER), buf, 3);
}

static uint8_t sx_read_reg(uint16_t adr)
{
    sx_begin(OTA_SX(CMD_READ_REGISTER));
    spi_byte(adr >> 8);
    spi_byte(adr);
    spi_byte(0);
    uint8_t value = spi_byte(0);
    sx_end();
    return value;
}

static void sx_set_packet_len(uint8_t len)
{
#ifdef DEVICE_HAS_SX128x
    const uint8_t buf[7] = { 12, SX1280_LORA_HEADER_EXPLICIT, len, SX1280_LORA_CRC_ENABLE, SX1280_LORA_IQ_NORMAL, 0, 0 };
#else
    // as Sx126xGfskConfiguration[], with our length
    const uint8_t buf[9] = { 0, 16, SX126X_GFSK_PREAMBLE_DETECTOR_LENGTH_8BITS, 16, SX126X_GFSK_ADDRESS_FILTERING_DISABLE,
                             SX126X_GFSK_PKT_FIX_LEN, len, SX126X_GFSK_CRC_OFF, SX126X_GFSK_WHITENING_ENABLE };
#endif
    sx_cmd(OTA_SX(CMD_SET_PACKET_PARAMS), buf, sizeof(buf));
}

static uint16_t sx_get_clear_irq(void)
{
    uint8_t buf[2];
    sx_cmd_read(OTA_SX(CMD_GET_IRQ_STATUS), buf, 2);
    const uint8_t clr[2] = { 0xFF, 0xFF };
    sx_cmd(OTA_SX(CMD_CLR_IRQ_STATUS), clr, 2);
    return ((uint16_t)buf[0] << 8) | buf[1];
}

static bool sx_wait_dio(uint32_t tmo_ms)
{
    while (!pin_read(GPIOA, SX_DIO_PIN)) {
        if (tick_1ms()) { if (!tmo_ms) return false; tmo_ms--; }
    }
    return true;
}

static void sx_set_rx(void)
{
    const uint8_t tmo[3] = { 0, 0, 0 }; // no timeout, for both chips
    pin_low(GPIOB, SX_TX_EN_PIN);
    pin_high(GPIOB, SX_RX_EN_PIN);
#ifdef OTA_USE_FSK
    sx_set_packet_len(OTA_FSK_FRAME_LEN_TX);
#else
    sx_set_packet_len(255);
#endif
    sx_cmd(OTA_SX(CMD_SET_RX), tmo, 3);
}

// data must have room for a frame
static void sx_send(uint8_t* data, uint8_t len)
{
#ifdef DEVICE_HAS_SX128x
    const uint8_t tmo[3] = { SX1280_PERIODBASE_1_MS, 0, 200 }; // 200 ms
#else
    const uint8_t tmo[3] = { 0, 0x32, 0 }; // 200 ms
#endif

#ifdef OTA_USE_FSK
    delay_ms(2); // the preamble is short, the tx must be in receive when we start
    ota_fsk_frame_pack(data, len, OTA_FSK_FRAME_LEN_RX);
    len = OTA_FSK_FRAME_LEN_RX;
#endif
    sx_set_packet_len(len);
    sx_begin(OTA_SX(CMD_WRITE_BUFFER));
    spi_byte(0);
    for (uint8_t n = 0; n < len; n++) spi_byte(data[n]);
    sx_end();

    pin_low(GPIOB, SX_RX_EN_PIN);
    pin_high(GPIOB, SX_TX_EN_PIN);
    sx_cmd(OTA_SX(CMD_SET_TX), tmo, 3);
    sx_wait_dio(250);
    sx_get_clear_irq();
    pin_low(GPIOB, SX_TX_EN_PIN);
}

// returns length of the packet, 0 if nothing useful was received
// data must have room for a frame
static uint8_t sx_receive(uint8_t* data)
{
    uint16_t irq = sx_get_clear_irq();
    if (!(irq & OTA_SX(IRQ_RX_DONE)) || (irq & (OTA_SX(IRQ_CRC_ERROR) | OTA_SX(IRQ_HEADER_ERROR)))) return 0;

    uint8_t status[2]; // len, start
    sx_cmd_read(OTA_SX(CMD_GET_RX_BUFFER_STATUS), status, 2);
#ifdef OTA_USE_FSK
    if (status[0] != OTA_FSK_FRAME_LEN_TX) return 0;
#else
    if (status[0] > OTA_PACKET_LEN_MAX) return 0;
#endif

    sx_begin(OTA_SX(CMD_READ_BUFFER));
    spi_byte(status[1]);
    spi_byte(0);
    for (uint8_t n = 0; n < status[0]; n++) data[n] = spi_byte(0);
    sx_end();
#ifdef OTA_USE_FSK
    return ota_fsk_frame_unpack(data, OTA_FSK_FRAME_LEN_TX);
#else
    return status[0];
#endif
}

// sequence follows Sx126xDriverCommon::Configure(), Sx128xDriverCommon::Configure()
// SX126x does GFSK, SX128x does LoRa
static void sx_init(const tOtaParams* params)
{
    pin_low(GPIOB, SX_RESET_PIN);
    delay_ms(5);
    pin_high(GPIOB, SX_RESET_PIN);
    delay_ms(50);

    sx_cmd1(OTA_SX(CMD_SET_STANDBY), OTA_SX(STDBY_CONFIG_STDBY_RC));
    delay_ms(2);

#ifdef DEVICE_HAS_SX128x
    uint8_t status = sx_read_reg(SX1280_REG_FIRMWARE_VERSION_MSB);
#else
    sx_begin(SX126X_CMD_GET_STATUS);
    uint8_t status = spi_byte(0);
    sx_end();
#endif
    if (status == 0 || status == 0xFF) fail(); // no radio

#ifdef DEVICE_HAS_SX128x
    const uint8_t zero[2] = { 0, 0 };
    sx_cmd1(SX1280_CMD_SET_REGULATOR_MODE, SX1280_REGULATOR_MODE_DCDC); // as SX_USE_REGULATOR_MODE_DCDC in the hal
    sx_cmd1(SX1280_CMD_SET_PACKET_TYPE, SX1280_PACKET_TYPE_LORA);
    sx_cmd1(SX1280_CMD_SET_AUTOFS, SX1280_AUTOFS_ENABLE);
    sx_write_reg(SX1280_REG_RxGain, sx_read_reg(SX1280_REG_RxGain) | 0xC0); // high sensitivity

    const uint8_t txparams[2] = { (uint8_t)params->sx_power, SX1280_RAMPTIME_04_US };
    sx_cmd(SX1280_CMD_SET_TX_PARAMS, txparams, 2);

    uint32_t reg = params->sx_freq_reg;
    const uint8_t freq[3] = { (uint8_t)(reg >> 16), (uint8_t)(reg >> 8), (uint8_t)reg };
    sx_cmd(SX1280_CMD_SET_RF_FREQUENCY, freq, 3);

    const uint8_t mod[3] = { params->sx_sf, params->sx_bw, params->sx_cr };
    sx_cmd(SX1280_CMD_SET_MODULATION_PARAMS, mod, 3);
    sx_write_reg(SX1280_REG_SFAdditionalConfiguration, 0x1E); // for SF5, SF6, datasheet after table 14-47
    sx_write_reg(SX1280_REG_FrequencyErrorCorrection, 0x01);

    sx_set_packet_len(255);
#else
    sx_cmd1(SX126X_CMD_SET_PACKET_TYPE, SX126X_PACKET_TYPE_GFSK);
    sx_write_reg(SX126X_REG_TX_CLAMP_CONFIG, sx_read_reg(SX126X_REG_TX_CLAMP_CONFIG) | 0x1E);

    const uint8_t zero[2] = { 0, 0 };
    sx_cmd(SX126X_CMD_CLEAR_DEVICE_ERRORS, zero, 2);
    const uint8_t tcxo[4] = { SX126X_DIO3_OUTPUT_1_8, 0, 0, 250 };
    sx_cmd(SX126X_CMD_SET_DIO3_AS_TCXO_CTRL, tcxo, 4);

    uint8_t cal[2] = { SX126X_CAL_IMG_902_MHZ_1, SX126X_CAL_IMG_902_MHZ_2 };
    if (params->sx_freq_reg < SX126X_FREQ_MHZ_TO_REG(900)) { cal[0] = SX126X_CAL_IMG_863_MHZ_1; cal[1] = SX126X_CAL_IMG_863_MHZ_2; }
    sx_cmd(SX126X_CMD_CALIBRATE_IMAGE, cal, 2);

    sx_cmd1(SX126X_CMD_SET_RX_TX_FALLBACK_MODE, SX126X_RX_TX_FALLBACK_MODE_FS);
    sx_write_reg(SX126X_REG_RX_GAIN, SX126X_RX_GAIN_BOOSTED_GAIN);
    sx_write_reg(SX126X_REG_OCP_CONFIGURATION, SX126X_OCP_CONFIGURATION_140_MA);

    const uint8_t pa[4] = { SX126X_PA_CONFIG_PA_DUTY_CYCLE_SX1262_10DBM, SX126X_PA_CONFIG_HP_SX1262_10DBM,
                            SX126X_PA_CONFIG_DEVICE_SEL_SX1262, SX126X_PA_CONFIG_PA_LUT };
    sx_cmd(SX126X_CMD_SET_PA_CONFIG, pa, 4);
    const uint8_t txparams[2] = { (uint8_t)params->sx_power, SX126X_RAMPTIME_40_US };
    sx_cmd(SX126X_CMD_SET_TX_PARAMS, txparams, 2);

    uint32_t reg = params->sx_freq_reg;
    const uint8_t freq[4] = { (uint8_t)(reg >> 24), (uint8_t)(reg >> 16), (uint8_t)(reg >> 8), (uint8_t)reg };
    sx_cmd(SX126X_CMD_SET_RF_FREQUENCY, freq, 4);

    // as Sx126xDriverBase::SetModulationParamsGFSK() with Sx126xGfskConfiguration[]
    const uint32_t br = (32 * (uint32_t)SX126X_FREQ_XTAL_HZ) / OTA_FSK_BITRATE_BPS;
    const uint32_t fdev = (uint32_t)(((uint64_t)OTA_FSK_FDEV_HZ << 25) / SX126X_FREQ_XTAL_HZ);
    const uint8_t mod[8] = { (uint8_t)(br >> 16), (uint8_t)(br >> 8), (uint8_t)br, SX126X_GFSK_PULSESHAPE_BT_1,
                             SX126X_GFSK_BW_312000, (uint8_t)(fdev >> 16), (uint8_t)(fdev >> 8), (uint8_t)fdev };
    sx_cmd(SX126X_CMD_SET_MODULATION_PARAMS, mod, 8);
    sx_write_reg(SX126X_REG_TX_MODULATION, sx_read_reg(SX126X_REG_TX_MODULATION) | 0x04); // datasheet 15.1

    sx_set_packet_len(OTA_FSK_FRAME_LEN_TX);
    const uint8_t sync[10] = { (uint8_t)(SX126X_REG_SYNC_WORD_0 >> 8), (uint8_t)SX126X_REG_SYNC_WORD_0,
                               (uint8_t)(OTA_FSK_SYNCWORD >> 8), (uint8_t)OTA_FSK_SYNCWORD, 0, 0, 0, 0, 0, 0 };
    sx_cmd(SX126X_CMD_WRITE_REGISTER, sync, 10);
#endif

    sx_cmd(OTA_SX(CMD_SET_BUFFER_BASEADDRESS), zero, 2);
    const uint8_t irq[8] = { (uint8_t)(SX_IRQ_MASK >> 8), (uint8_t)SX_IRQ_MASK, (uint8_t)(SX_IRQ_MASK >> 8), (uint8_t)SX_IRQ_MASK, 0, 0, 0, 0 };
    sx_cmd(OTA_SX(CMD_SET_DIOIRQ_PARAMS), irq, 8);
    sx_get_clear_irq();
}


//-------------------------------------------------------
// OTA
//-------------------------------------------------------

typedef struct
{
    uint32_t length; // 0 = no transfer started
    uint16_t next_block;
} tOtaState;

static inline uint32_t get_u32(const uint8_t* p) { return p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static inline void put_u32(uint8_t* p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

// handles the packet in buf, puts the response into buf, returns its length
static uint8_t ota_handle(tOtaState* ota, uint8_t* buf, uint8_t len, bool* done)
{
    uint8_t cmd = buf[0];
    uint8_t* payload = buf + OTA_PACKET_HEADER_LEN;
    uint8_t payload_len = len - OTA_PACKET_HEADER_LEN;
    uint8_t status = OTA_STATUS_OK;

    buf[0] = cmd | OTA_CMD_RESPONSE; // session_id stays in place

    switch (cmd) {
    case OTA_CMD_HELLO:
        payload[0] = OTA_STATUS_OK;
        payload[1] = OTA_LOADER_VERSION;
        payload[2] = OTA_BLOCK_SIZE;
        put_u32(payload + 3, OTA_TARGET_ID);
        put_u32(payload + 7, OTA_APP_SIZE_MAX);
        payload[11] = 0; // no flags, we can't do deflate
        return OTA_PACKET_HEADER_LEN + 12;

    case OTA_CMD_BEGIN: {
        if (payload_len != 13) return 0;
        uint32_t length = get_u32(payload);
        if (get_u32(payload + 4) != OTA_TARGET_ID) {
            status = OTA_STATUS_ERR_TARGET;
        } else
        if (payload[8] != 0 || get_u32(payload + 9) != length) {
            status = OTA_STATUS_ERR_UNSUPPORTED;
        } else
        if (length <= OTA_APP_INFO_OFFSET || length > OTA_APP_SIZE_MAX || (length & 7)) {
            status = OTA_STATUS_ERR_LENGTH;
        } else {
            ota->length = length;
            ota->next_block = 0;
        }
        break; }

    case OTA_CMD_DATA: {
        if (payload_len < 2 + 8 || payload_len > 2 + OTA_BLOCK_SIZE || ((payload_len - 2) & 7)) return 0;
        uint16_t block = payload[0] | ((uint16_t)payload[1] << 8);
        uint32_t offset = (uint32_t)block * OTA_BLOCK_SIZE;
        uint8_t data_len = payload_len - 2;
        if (!ota->length) {
            status = OTA_STATUS_ERR_STATE;
        } else
        if (block != ota->next_block) {
            // repeated or out of order, the response tells which one we want
        } else
        if (offset + data_len > ota->length) {
            status = OTA_STATUS_ERR_LENGTH;
        } else {
            bool ok = true;
            if ((offset % OTA_FLASH_PAGE_SIZE) == 0) ok = flash_erase_page(OTA_APP_BASE + offset);
            if (ok) ok = flash_program(OTA_APP_BASE + offset, payload + 2, data_len);
            if (ok) ota->next_block++; else status = OTA_STATUS_ERR_FLASH;
            GPIOA->ODR ^= (1u << LED_GREEN_PIN);
        }
        break; }

    case OTA_CMD_END:
        if (!ota->length) {
            status = OTA_STATUS_ERR_STATE;
        } else
        if (!app_is_valid() || ((const tOtaAppInfo*)(OTA_APP_BASE + OTA_APP_INFO_OFFSET))->length != ota->length) {
            status = OTA_STATUS_ERR_IMAGE;
        } else {
            *done = true;
        }
        break;

    default:
        return 0;
    }

    payload[0] = status;
    payload[1] = (uint8_t)ota->next_block;
    payload[2] = (uint8_t)(ota->next_block >> 8);
    return OTA_PACKET_HEADER_LEN + 3;
}

__attribute__((noreturn)) static void ota_run(bool app_valid)
{
    const tOtaParams* params = (const tOtaParams*)OTA_PARAMS_BASE;
    tOtaState ota = { 0, 0 };
    uint8_t buf[OTA_PACKET_LEN_MAX + 8];
    uint32_t idle_ms = 0;
    bool done = false;

    hw_init();
    pin_high(GPIOA, LED_RED_PIN);
    flash_unlock();
    sx_init(params);
    sx_set_rx();

    while (!done) {
        if (!sx_wait_dio(100)) {
            idle_ms += 100;
            if (app_valid && !ota.length && idle_ms >= OTA_IDLE_TIMEOUT_MS) break; // nobody came, old app is intact
            continue;
        }

        uint8_t len = sx_receive(buf);
        if (len >= OTA_PACKET_HEADER_LEN && (buf[1] | ((uint16_t)buf[2] << 8)) == params->session_id) {
            idle_ms = 0;
            len = ota_handle(&ota, buf, len, &done);
            if (len) sx_send(buf, len);
        }
        sx_set_rx();
    }

    flash_erase_page(OTA_PARAMS_BASE); // request is served
    NVIC_SystemReset(); // gives the app a clean start
    while (1) {}
}


//-------------------------------------------------------
// Entry
//-------------------------------------------------------

extern "C" __attribute__((noreturn)) void ota_loader_reset(void)
{
    bool app_valid = app_is_valid();
    bool requested = params_are_valid();

    if (app_valid && !requested) app_jump();

    if (!requested) { // no app and no radio params, nothing we can do
        hw_init();
        fail();
    }

    ota_run(app_valid);
}

extern "C" __attribute__((noreturn)) void ota_loader_fault(void)
{
    while (1) {}
}

// goes to a fixed place in the app, length is filled in by the build script
extern "C" __attribute__((section(".ota_app_info"), used))
const tOtaAppInfo ota_app_info = { OTA_APP_INFO_MAGIC, OTA_TARGET_ID, 0, VERSION };

typedef void (*tOtaVector)(void);

extern "C" __attribute__((section(".ota_loader_vectors"), used))
const tOtaVector ota_loader_vectors[16] = {
    (tOtaVector)OTA_RAM_END, // initial stack pointer
    ota_loader_reset,
    ota_loader_fault, // NMI
    ota_loader_fault, // HardFault
    ota_loader_fault, // MemManage
    ota_loader_fault, // BusFault
    ota_loader_fault, // UsageFault
    0, 0, 0, 0,
    ota_loader_fault, // SVC
    ota_loader_fault, // DebugMon
    0,
    ota_loader_fault, // PendSV
    ota_loader_fault, // SysTick
};

#endif

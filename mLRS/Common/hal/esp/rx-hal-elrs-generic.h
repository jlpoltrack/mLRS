//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// ESP, ELRS GENERIC RX, configured by ELRS_xxx defines
// from the ELRS targets repo, see tools/elrs/elrs_targets.py
//-------------------------------------------------------

#ifdef ELRS_LED_RGB
  #define DEVICE_HAS_SINGLE_LED_RGB
#else
  #define DEVICE_HAS_SINGLE_LED
#endif
#ifdef ELRS_RADIO2_NSS
  #define DEVICE_HAS_DIVERSITY_SINGLE_SPI
#endif
#ifdef ELRS_OUT_TX
  #define DEVICE_HAS_OUT
#endif
#define DEVICE_HAS_NO_DEBUG


//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_SERIAL
#define UARTB_BAUD                RX_SERIAL_BAUDRATE
#ifdef ELRS_SERIAL_TX
  #define UARTB_USE_TX_IO         ELRS_SERIAL_TX
  #define UARTB_USE_RX_IO         ELRS_SERIAL_RX
#endif
#define UARTB_TXBUFSIZE           RX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           RX_SERIAL_RXBUFSIZE

#ifdef ELRS_OUT_TX
  #define UART_USE_SERIAL1
  #define UART_BAUD               416666 // CRSF baud rate
  #define UART_USE_TX_IO          ELRS_OUT_TX
  #define UART_USE_RX_IO          -1 // no Rx pin needed
  #define UART_TXBUFSIZE          256
#endif

#define UARTF_USE_SERIAL
#define UARTF_BAUD                115200


//-- SX1: SX12xx & SPI

#define SPI_CS_IO                 ELRS_RADIO_NSS
#ifdef ELRS_SPI_MISO
  #define SPI_MISO                ELRS_SPI_MISO
  #define SPI_MOSI                ELRS_SPI_MOSI
  #define SPI_SCK                 ELRS_SPI_SCK
#endif
#define SPI_FREQUENCY             ELRS_SPI_FREQUENCY
#define SX_RESET                  ELRS_RADIO_RST
#define SX_DIO                    ELRS_RADIO_DIO
#ifdef ELRS_RADIO_BUSY
  #define SX_BUSY                 ELRS_RADIO_BUSY
#endif
#ifdef ELRS_RFSW_CTRL
  #define SX_USE_RFSW_CTRL        { ELRS_RFSW_CTRL }
#endif

#if defined ESP8266 || defined ESP8285
  #define ELRS_RESET_INIT         IO_MODE_OUTPUT_PP_HIGH
  #define ELRS_DIO_INIT           IO_MODE_INPUT_ANALOG
#else
  #define ELRS_RESET_INIT         IO_MODE_OUTPUT_PP_LOW
  #ifdef DEVICE_HAS_SX127x
    #define ELRS_DIO_INIT         IO_MODE_INPUT_PU
  #else
    #define ELRS_DIO_INIT         IO_MODE_INPUT_ANALOG
  #endif
#endif

IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_RESET, ELRS_RESET_INIT);
    gpio_init(SX_DIO, ELRS_DIO_INIT);
#ifdef SX_BUSY
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU);
#endif
#ifdef ELRS_RADIO_TXEN
    gpio_init(ELRS_RADIO_TXEN, IO_MODE_OUTPUT_PP_LOW);
#endif
#ifdef ELRS_RADIO_RXEN
    gpio_init(ELRS_RADIO_RXEN, IO_MODE_OUTPUT_PP_LOW);
#endif
#ifdef ELRS_RADIO_ANT
    gpio_init(ELRS_RADIO_ANT, IO_MODE_OUTPUT_PP_HIGH); // antenna1 only
#endif
}

#ifdef SX_BUSY
IRAM_ATTR bool sx_busy_read(void) { return (gpio_read_activehigh(SX_BUSY)) ? true : false; }
#endif

IRAM_ATTR void sx_amp_transmit(void)
{
#ifdef ELRS_RADIO_RXEN
    gpio_low(ELRS_RADIO_RXEN);
#endif
#ifdef ELRS_RADIO_TXEN
    gpio_high(ELRS_RADIO_TXEN);
#endif
}

IRAM_ATTR void sx_amp_receive(void)
{
#ifdef ELRS_RADIO_TXEN
    gpio_low(ELRS_RADIO_TXEN);
#endif
#ifdef ELRS_RADIO_RXEN
    gpio_high(ELRS_RADIO_RXEN);
#endif
}

#if defined ESP8266 || defined ESP8285
void sx_dio_init_exti_isroff(void) {}
#else
void sx_dio_init_exti_isroff(void) { detachInterrupt(SX_DIO); }
#endif
void sx_dio_enable_exti_isr(void) { attachInterrupt(SX_DIO, SX_DIO_EXTI_IRQHandler, RISING); }
IRAM_ATTR void sx_dio_exti_isr_clearflag(void) {}


//-- SX2: SX12xx & SPI

#ifdef ELRS_RADIO2_NSS

#define SX2_CS_IO                 ELRS_RADIO2_NSS
#define SX2_RESET                 ELRS_RADIO2_RST
#define SX2_DIO                   ELRS_RADIO2_DIO
#ifdef ELRS_RADIO2_BUSY
  #define SX2_BUSY                ELRS_RADIO2_BUSY
#endif

IRQHANDLER(void SX2_DIO_EXTI_IRQHandler(void);)

void sx2_init_gpio(void)
{
    gpio_init(SX2_CS_IO, IO_MODE_OUTPUT_PP_HIGH);
    gpio_init(SX2_RESET, ELRS_RESET_INIT);
    gpio_init(SX2_DIO, ELRS_DIO_INIT);
#ifdef SX2_BUSY
    gpio_init(SX2_BUSY, IO_MODE_INPUT_PU);
#endif
#ifdef ELRS_RADIO2_TXEN
    gpio_init(ELRS_RADIO2_TXEN, IO_MODE_OUTPUT_PP_LOW);
#endif
#ifdef ELRS_RADIO2_RXEN
    gpio_init(ELRS_RADIO2_RXEN, IO_MODE_OUTPUT_PP_LOW);
#endif
}

IRAM_ATTR void spib_select(void) { gpio_low(SX2_CS_IO); }
IRAM_ATTR void spib_deselect(void) { gpio_high(SX2_CS_IO); }

#ifdef SX2_BUSY
IRAM_ATTR bool sx2_busy_read(void) { return (gpio_read_activehigh(SX2_BUSY)) ? true : false; }
#endif

IRAM_ATTR void sx2_amp_transmit(void)
{
#ifdef ELRS_RADIO2_RXEN
    gpio_low(ELRS_RADIO2_RXEN);
#endif
#ifdef ELRS_RADIO2_TXEN
    gpio_high(ELRS_RADIO2_TXEN);
#endif
}

IRAM_ATTR void sx2_amp_receive(void)
{
#ifdef ELRS_RADIO2_TXEN
    gpio_low(ELRS_RADIO2_TXEN);
#endif
#ifdef ELRS_RADIO2_RXEN
    gpio_high(ELRS_RADIO2_RXEN);
#endif
}

void sx2_dio_init_exti_isroff(void) { detachInterrupt(SX2_DIO); }
void sx2_dio_enable_exti_isr(void) { attachInterrupt(SX2_DIO, SX2_DIO_EXTI_IRQHandler, RISING); }
IRAM_ATTR void sx2_dio_exti_isr_clearflag(void) {}

#endif // ELRS_RADIO2_NSS


//-- Out port

#ifdef ELRS_OUT_TX

void out_init_gpio(void) {}

void out_set_normal(void)
{
    gpio_matrix_out((gpio_num_t)UART_USE_TX_IO, U1TXD_OUT_IDX, false, false);
}

void out_set_inverted(void)
{
    gpio_matrix_out((gpio_num_t)UART_USE_TX_IO, U1TXD_OUT_IDX, true, false);
}

#endif


//-- Button

#ifdef ELRS_BUTTON

#define BUTTON                    ELRS_BUTTON

void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU);
}

IRAM_ATTR bool button_pressed(void)
{
    return gpio_read_activelow(BUTTON) ? true : false;
}

#else

void button_init(void) {}
IRAM_ATTR bool button_pressed(void) { return false; }

#endif


//-- LEDs

#ifdef ELRS_LED_RGB

#define LED_RGB                   ELRS_LED_RGB
#define LED_RGB_PIXEL_NUM         1
#include "esp-hal-led-rgb.h"

#else

#define LED_RED                   ELRS_LED

void leds_init(void)
{
#ifdef ELRS_LED_INVERTED
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_HIGH);
#else
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_LOW);
#endif
}

#ifdef ELRS_LED_INVERTED
IRAM_ATTR void led_red_off(void) { gpio_high(LED_RED); }
IRAM_ATTR void led_red_on(void) { gpio_low(LED_RED); }
#else
IRAM_ATTR void led_red_off(void) { gpio_low(LED_RED); }
IRAM_ATTR void led_red_on(void) { gpio_high(LED_RED); }
#endif
IRAM_ATTR void led_red_toggle(void) { gpio_toggle(LED_RED); }

#endif


//-- POWER
// ELRS power levels ELRS_POWER_MIN .. ELRS_POWER_MAX, with ELRS calibrated sx power settings

#define RFPOWER_DEFAULT           0 // index into rfpower_list array

constexpr int8_t elrs_power_dbm[] = { POWER_10_DBM, POWER_14_DBM, POWER_17_DBM, POWER_20_DBM, POWER_24_DBM, POWER_27_DBM, POWER_30_DBM, POWER_33_DBM };
constexpr int16_t elrs_power_mW[] = { 10, 25, 50, 100, 250, 500, 1000, 2000 };

#define ELRS_RFPOWER(i)           { .dbm = elrs_power_dbm[i], .mW = elrs_power_mW[i] }

const rfpower_t rfpower_list[] = {
    ELRS_RFPOWER(ELRS_POWER_MIN),
#if ELRS_POWER_MAX >= ELRS_POWER_MIN + 1
    ELRS_RFPOWER(ELRS_POWER_MIN + 1),
#endif
#if ELRS_POWER_MAX >= ELRS_POWER_MIN + 2
    ELRS_RFPOWER(ELRS_POWER_MIN + 2),
#endif
#if ELRS_POWER_MAX >= ELRS_POWER_MIN + 3
    ELRS_RFPOWER(ELRS_POWER_MIN + 3),
#endif
#if ELRS_POWER_MAX >= ELRS_POWER_MIN + 4
    ELRS_RFPOWER(ELRS_POWER_MIN + 4),
#endif
#if ELRS_POWER_MAX >= ELRS_POWER_MIN + 5
    ELRS_RFPOWER(ELRS_POWER_MIN + 5),
#endif
#if ELRS_POWER_MAX >= ELRS_POWER_MIN + 6
    ELRS_RFPOWER(ELRS_POWER_MIN + 6),
#endif
#if ELRS_POWER_MAX >= ELRS_POWER_MIN + 7
    ELRS_RFPOWER(ELRS_POWER_MIN + 7),
#endif
};

uint8_t elrs_rfpower_index(const int8_t power_dbm)
{
    uint8_t i = ARRAY_LEN(rfpower_list) - 1;
    while (i > 0 && power_dbm < rfpower_list[i].dbm) i--;
    return i;
}

#if defined DEVICE_HAS_LR11xx

#include "../../setup_types.h" // needed for frequency band condition in rfpower calc

void lr11xx_rfpower_calc(const int8_t power_dbm, int8_t* sx_power, int8_t* actual_power_dbm, const uint8_t frequency_band)
{
    static const int8_t sx_power_list_lf[] = { ELRS_SX_POWER_LIST_LF };
    static const int8_t sx_power_list_hf[] = { ELRS_SX_POWER_LIST_HF };
    uint8_t i = elrs_rfpower_index(power_dbm);
    *sx_power = (frequency_band == SX_FHSS_FREQUENCY_BAND_2P4_GHZ) ? sx_power_list_hf[i] : sx_power_list_lf[i];
    *actual_power_dbm = rfpower_list[i].dbm;
}

#else

#ifdef DEVICE_HAS_SX127x
void sx1276_rfpower_calc(const int8_t power_dbm, int8_t* sx_power, int8_t* actual_power_dbm)
#else
void sx128x_rfpower_calc(const int8_t power_dbm, int8_t* sx_power, int8_t* actual_power_dbm)
#endif
{
    static const int8_t sx_power_list[] = { ELRS_SX_POWER_LIST };
    uint8_t i = elrs_rfpower_index(power_dbm);
    *sx_power = sx_power_list[i];
    *actual_power_dbm = rfpower_list[i].dbm;
}

#endif

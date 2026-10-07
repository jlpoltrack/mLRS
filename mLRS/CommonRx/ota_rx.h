//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// OTA, Rx side
// STM32: hands over to the OTA loader in low flash
// ESP32, ESP8266: no loader, the app receives the new image itself
//*******************************************************
#ifndef OTA_RX_H
#define OTA_RX_H
#pragma once


#include "../Common/ota/ota_link.h"


#if defined ESP32 || defined ESP8266

#define OTA_IDLE_TIMEOUT_MS       30000 // leave if no tx shows up
#define OTA_END_LINGER_MS         1500 // stay for repeats, the end response may get lost
#define OTA_STALL_TIMEOUT_MS      5000 // leave if a transfer stalls
#define OTA_WDT_TIMEOUT_S         5 // reboots us if we hang, e.g. sx driver waiting for busy

// tells the host tool which target the image is for
// volatile, else hello doesn't really read it and the linker drops it
extern "C" const volatile tOtaAppInfo ota_app_info = { OTA_APP_INFO_MAGIC, OTA_TARGET_ID, 0, VERSION };

static inline uint32_t ota_get_u32(const uint8_t* p) { return p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static inline void ota_put_u32(uint8_t* p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }


#ifdef ESP32
//-------------------------------------------------------
// ESP32, where the app goes
//-------------------------------------------------------
// The image goes into the inactive app partition, the bootloader switches over.

#include <esp_ota_ops.h>
#include <esp_task_wdt.h>
#if defined CONFIG_IDF_TARGET_ESP32C3 // inflate is in the rom
  #include <esp32c3/rom/miniz.h>
#elif defined CONFIG_IDF_TARGET_ESP32S3
  #include <esp32s3/rom/miniz.h>
#else
  #include <esp32/rom/miniz.h>
#endif


typedef struct
{
    const esp_partition_t* partition;
    esp_ota_handle_t handle;
    uint32_t space; // size of the partition
    uint32_t length; // what is transferred, 0 = no transfer started
    uint32_t image_length; // what it becomes in flash
    uint32_t written;
    uint16_t next_block;
    bool done;
    // deflate, inflator = nullptr if not used
    tinfl_decompressor* inflator;
    uint8_t* dict; // is also the output buffer
    uint32_t dict_pos;
    bool inflate_done;
} tOtaState;


uint8_t ota_write(tOtaState* ota, const uint8_t* data, uint32_t len)
{
    if (!len) return OTA_STATUS_OK;
    if (ota->written + len > ota->image_length) return OTA_STATUS_ERR_DATA;
    if (esp_ota_write(ota->handle, data, len) != ESP_OK) return OTA_STATUS_ERR_FLASH;
    ota->written += len;
    return OTA_STATUS_OK;
}


// inflates the data and writes the output, more = not the last data
uint8_t ota_inflate_and_write(tOtaState* ota, const uint8_t* data, uint32_t len, bool more)
{
    if (ota->inflate_done) return OTA_STATUS_ERR_DATA;

    while (1) {
        size_t in_len = len;
        size_t out_len = TINFL_LZ_DICT_SIZE - ota->dict_pos;
        tinfl_status res = tinfl_decompress(ota->inflator, data, &in_len, ota->dict, ota->dict + ota->dict_pos, &out_len,
                                            TINFL_FLAG_PARSE_ZLIB_HEADER | ((more) ? TINFL_FLAG_HAS_MORE_INPUT : 0));
        if (res < TINFL_STATUS_DONE) return OTA_STATUS_ERR_DATA;

        uint8_t status = ota_write(ota, ota->dict + ota->dict_pos, out_len);
        if (status != OTA_STATUS_OK) return status;
        ota->dict_pos = (ota->dict_pos + out_len) & (TINFL_LZ_DICT_SIZE - 1);
        data += in_len;
        len -= in_len;

        if (res == TINFL_STATUS_DONE) {
            ota->inflate_done = true;
            return (len) ? OTA_STATUS_ERR_DATA : OTA_STATUS_OK;
        }
        if (res == TINFL_STATUS_NEEDS_MORE_INPUT && !len) return OTA_STATUS_OK;
        if (!in_len && !out_len) return OTA_STATUS_ERR_DATA; // must not happen
    }
}


#define OTA_STORAGE_FLAGS         OTA_FLAG_DEFLATE


uint8_t ota_storage_begin(tOtaState* ota, uint32_t length, uint8_t flags, uint32_t image_length)
{
    if (ota->length) esp_ota_abort(ota->handle); // transfer is started over

    if (image_length > ota->space || (!(flags & OTA_FLAG_DEFLATE) && image_length != length)) return OTA_STATUS_ERR_LENGTH;

    ota->written = 0;
    ota->dict_pos = 0;
    ota->inflate_done = false;
    free(ota->inflator); ota->inflator = nullptr;
    free(ota->dict); ota->dict = nullptr;
    if (flags & OTA_FLAG_DEFLATE) {
        ota->inflator = (tinfl_decompressor*)malloc(sizeof(tinfl_decompressor));
        ota->dict = (uint8_t*)malloc(TINFL_LZ_DICT_SIZE);
        if (!ota->inflator || !ota->dict) return OTA_STATUS_ERR_UNSUPPORTED; // no memory
        tinfl_init(ota->inflator);
    }
    // erases along with the writes, erasing all here would take seconds
    if (esp_ota_begin(ota->partition, OTA_WITH_SEQUENTIAL_WRITES, &ota->handle) != ESP_OK) return OTA_STATUS_ERR_FLASH;
    ota->image_length = image_length;
    return OTA_STATUS_OK;
}


uint8_t ota_storage_write(tOtaState* ota, const uint8_t* data, uint8_t len, bool more)
{
    led_green_toggle();
    return (ota->inflator) ? ota_inflate_and_write(ota, data, len, more) : ota_write(ota, data, len);
}


uint8_t ota_storage_end(tOtaState* ota, const uint8_t* payload, uint8_t payload_len)
{
    if (ota->written != ota->image_length || (ota->inflator && !ota->inflate_done)) {
        esp_ota_abort(ota->handle);
        return OTA_STATUS_ERR_DATA;
    }
    // checks the image, releases the handle in any case
    if (esp_ota_end(ota->handle) != ESP_OK || esp_ota_set_boot_partition(ota->partition) != ESP_OK) return OTA_STATUS_ERR_IMAGE;
    return OTA_STATUS_OK;
}


void ota_storage_init(tOtaState* ota)
{
    ota->partition = esp_ota_get_next_update_partition(NULL);
    ota->space = (ota->partition) ? ota->partition->size : 0;
}


#else
//-------------------------------------------------------
// ESP8266, where the app goes
//-------------------------------------------------------
// No second app partition. The image goes as gzip stream into the free flash behind the app,
// the bootloader (eboot) unpacks it over the app at the next reset.
// ATTENTION: no way back once eboot has started, it doesn't check the stream before!

#include <Updater.h>


typedef struct
{
    uint32_t space; // flash for the app and the update together
    uint32_t length; // what is transferred, 0 = no transfer started
    uint32_t crc; // crc32 of what was received
    uint16_t next_block;
    bool done;
} tOtaState;


// = zlib crc32()
uint32_t ota_crc32(uint32_t crc, const uint8_t* data, uint8_t len)
{
    crc = ~crc;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
    return ~crc;
}


#define OTA_STORAGE_FLAGS         OTA_FLAG_GZIP


uint8_t ota_storage_begin(tOtaState* ota, uint32_t length, uint8_t flags, uint32_t image_length)
{
    uint32_t length_in_flash = (length + FLASH_SECTOR_SIZE - 1) & ~(FLASH_SECTOR_SIZE - 1);

    // the unpacked app must not run into the stream
    if ((!(flags & OTA_FLAG_GZIP) && image_length != length) || image_length + length_in_flash > ota->space) return OTA_STATUS_ERR_LENGTH;

    ota->crc = 0;
    // fails if a transfer is running, or the chip was not power cycled after flashing by wire
    return (Update.begin(length, U_FLASH)) ? OTA_STATUS_OK : OTA_STATUS_ERR_FLASH;
}


uint8_t ota_storage_write(tOtaState* ota, const uint8_t* data, uint8_t len, bool more)
{
    led_red_toggle();
    ota->crc = ota_crc32(ota->crc, data, len);
    if (Update.write((uint8_t*)data, len) == len) return OTA_STATUS_OK;
    ota->length = 0;
    return OTA_STATUS_ERR_FLASH;
}


uint8_t ota_storage_end(tOtaState* ota, const uint8_t* payload, uint8_t payload_len)
{
    // eboot doesn't check the stream, so it gets it only if it is what was sent
    if (payload_len != 4 || ota_get_u32(payload) != ota->crc) return OTA_STATUS_ERR_DATA;
    return (Update.end()) ? OTA_STATUS_OK : OTA_STATUS_ERR_IMAGE;
}


void ota_storage_init(tOtaState* ota)
{
    ota->space = ((ESP.getSketchSize() + FLASH_SECTOR_SIZE - 1) & ~(FLASH_SECTOR_SIZE - 1)) + ESP.getFreeSketchSpace();
}


#endif
//-------------------------------------------------------
// The packets, see ota_loader.h
//-------------------------------------------------------

// handles the packet in buf, puts the response into buf, returns its length
uint8_t ota_handle(tOtaState* ota, uint8_t* buf, uint8_t len)
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
        ota_put_u32(payload + 3, ota_app_info.target_id);
        ota_put_u32(payload + 7, ota->space);
        payload[11] = OTA_STORAGE_FLAGS;
        return OTA_PACKET_HEADER_LEN + 12;

    case OTA_CMD_BEGIN: {
        if (payload_len != 13) return 0;
        uint32_t length = ota_get_u32(payload);
        uint8_t flags = payload[8];
        uint32_t image_length = ota_get_u32(payload + 9);
        if (ota->done) {
            status = OTA_STATUS_ERR_STATE;
        } else
        if (ota_get_u32(payload + 4) != OTA_TARGET_ID) {
            status = OTA_STATUS_ERR_TARGET;
        } else
        if (flags & ~OTA_STORAGE_FLAGS) {
            status = OTA_STATUS_ERR_UNSUPPORTED;
        } else
        if (!length || !image_length) {
            status = OTA_STATUS_ERR_LENGTH;
        } else {
            status = ota_storage_begin(ota, length, flags, image_length);
            ota->length = (status == OTA_STATUS_OK) ? length : 0;
            ota->next_block = 0;
        }
        break; }

    case OTA_CMD_DATA: {
        if (payload_len < 2 + 1 || payload_len > 2 + OTA_BLOCK_SIZE) return 0;
        uint16_t block = payload[0] | ((uint16_t)payload[1] << 8);
        uint32_t offset = (uint32_t)block * OTA_BLOCK_SIZE;
        uint8_t data_len = payload_len - 2;
        if (!ota->length || ota->done) {
            status = OTA_STATUS_ERR_STATE;
        } else
        if (block != ota->next_block) {
            // repeated or out of order, the response tells which one we want
        } else
        if (offset + data_len > ota->length || (data_len < OTA_BLOCK_SIZE && offset + data_len != ota->length)) {
            status = OTA_STATUS_ERR_LENGTH; // only the last block can be short
        } else {
            status = ota_storage_write(ota, payload + 2, data_len, (offset + data_len < ota->length));
            if (status == OTA_STATUS_OK) ota->next_block++;
        }
        break; }

    case OTA_CMD_END:
        if (ota->done) {
            // repeated, our response got lost
        } else
        if (!ota->length) {
            status = OTA_STATUS_ERR_STATE;
        } else
        if ((uint32_t)ota->next_block * OTA_BLOCK_SIZE < ota->length) {
            status = OTA_STATUS_ERR_LENGTH;
        } else {
            status = ota_storage_end(ota, payload, payload_len);
            ota->length = 0;
            ota->done = (status == OTA_STATUS_OK);
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


//-------------------------------------------------------
// The link, see ota_link.h
//-------------------------------------------------------
// isrs are off, DIO is polled and spi done only when it is set, buf must have room for a frame

// short preamble, the tx must be in receive when we start
#ifdef DEVICE_HAS_SX127x
  #define OTA_RESPONSE_DELAY_US   2000
#else
  #define OTA_RESPONSE_DELAY_US   1000
#endif


void ota_radio_start(uint32_t sx_freq_reg)
{
    ota_link_start(sx_freq_reg);
    // also the errors, else a bad packet leaves us waiting
#ifdef DEVICE_HAS_LR11xx
    sx.SetDioIrqParams(LR11XX_IRQ_TX_DONE | LR11XX_IRQ_RX_DONE | LR11XX_IRQ_TIMEOUT | OTA_SX_IRQ_RX_ERROR, 0);
#elif defined DEVICE_HAS_SX128x
    sx.SetDioIrqParams(SX1280_IRQ_ALL, SX1280_IRQ_RX_DONE | SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_TX_TIMEOUT | OTA_SX_IRQ_RX_ERROR,
                       SX1280_IRQ_NONE, SX1280_IRQ_NONE);
#endif
}


void ota_set_rx(void)
{
    ota_link_set_packet_len((OTA_LINK_IS_FSK) ? OTA_FSK_FRAME_LEN_TX : 255);
    sx.SetToRx();
}


// returns the packet length, 0 if what came in is of no use, -1 if nothing came in yet
int16_t ota_receive(uint8_t* buf)
{
#ifdef DEVICE_HAS_SX127x_FSK
    if (gpio_read_activehigh(SX_DIO1)) sx.HandleDio1Irq(); // FifoLevel, fetches what has come in
#endif
    if (!gpio_read_activehigh(SX_DIO)) return -1;
    return ota_link_read(sx.GetAndClearIrqStatus(OTA_SX(IRQ_ALL)), buf, OTA_FSK_FRAME_LEN_TX);
}


void ota_send(uint8_t* buf, uint8_t len)
{
    len = ota_link_pack(buf, len, OTA_FSK_FRAME_LEN_RX);

    delay_us(OTA_RESPONSE_DELAY_US);
    ota_link_set_packet_len(len);
    sx.SendFrame(buf, len, 200);

    uint32_t tstart_ms = millis32();
    while (!gpio_read_activehigh(SX_DIO)) { // tx done or timeout
        if (millis32() - tstart_ms > 250) break;
    }
    sx.GetAndClearIrqStatus(OTA_SX(IRQ_ALL));
}


//-------------------------------------------------------
// The update
//-------------------------------------------------------

// runs the update, then reboots into the new app, or the running one if it failed, does not return
void ota_enter_loader(uint32_t sx_freq_reg, uint16_t session_id)
{
tOtaState ota = {};
uint8_t buf[OTA_FSK_FRAME_LEN_TX + 8];

    // the isrs do spi, and we poll anyway
#ifdef ESP8266 // sx_dio_init_exti_isroff() is empty on these, and they have no sx2
    detachInterrupt(SX_DIO);
  #ifdef DEVICE_HAS_SX127x_FSK
    detachInterrupt(SX_DIO1);
  #endif
#else
    sx_dio_init_exti_isroff();
  #ifdef DEVICE_HAS_SX127x_FSK
    sx_dio1_init_exti_isroff();
  #endif
  #ifdef USE_SX2
    sx2_dio_init_exti_isroff();
  #endif
  #if defined USE_SX2 && defined DEVICE_HAS_SX127x_FSK
    sx2_dio1_init_exti_isroff();
  #endif
#endif
    sx.SetToIdle(); // only after the isrs are off, they do spi
    sx2.SetToIdle();
    led_red_on();

    // only sx is used, sx2 is held in reset, idle is not off for all chips
#if defined USE_SX2 && defined SX2_RESET
    gpio_low(SX2_RESET);
  #ifdef SX2_RX_EN
    gpio_low(SX2_RX_EN); // reset doesn't switch off its front end
  #endif
  #ifdef SX2_TX_EN
    gpio_low(SX2_TX_EN);
  #endif
#endif

    ota_storage_init(&ota);

    ota_radio_start(sx_freq_reg);
    sx.SetRfPower_dbm(rfpower_list[0].dbm); // tx and rx sit next to each other
    ota_set_rx();

    uint32_t tlast_ms = millis32();

#ifdef ESP32
    esp_task_wdt_init(OTA_WDT_TIMEOUT_S, true);
    esp_task_wdt_add(NULL);
#endif

    while (1) {
#ifdef ESP8266
        ESP.wdtFeed(); // else its wdt reboots after some seconds
#else
        esp_task_wdt_reset();
#endif
        uint32_t tmo_ms = (ota.done) ? OTA_END_LINGER_MS : (ota.length) ? OTA_STALL_TIMEOUT_MS : OTA_IDLE_TIMEOUT_MS;
        if (millis32() - tlast_ms > tmo_ms) break;

        int16_t len = ota_receive(buf);
        if (len < 0) continue;

        if (len >= OTA_PACKET_HEADER_LEN && (buf[1] | ((uint16_t)buf[2] << 8)) == session_id) {
            tlast_ms = millis32();
            len = ota_handle(&ota, buf, len);
            if (len) ota_send(buf, len);
        }
        ota_set_rx();
    }

    // the chip is left in reset, the startup releases it
#if defined SX_RESET
    gpio_low(SX_RESET);
#elif defined DEVICE_HAS_SX127x
    sx.SetStandby();
#else
    sx.SetStandby(OTA_SX(STDBY_CONFIG_STDBY_RC));
#endif
    delay_us(1000);

    ESP.restart();
    while (1) {}
}


#else
//-------------------------------------------------------
// STM32
//-------------------------------------------------------

#ifndef OTA_LOADER_BASE
  #error OTA loader: mcu not supported!
#endif

// writes the params page, which is the update request for the loader, and reboots
// does not return
void ota_enter_loader(uint32_t sx_freq_reg, uint16_t session_id)
{
union {
    tOtaParams params;
    uint32_t w[6]; // 3 double words
} u;

    sx.SetToIdle();
    sx2.SetToIdle();

    for (uint8_t i = 0; i < 6; i++) u.w[i] = 0xFFFFFFFF;

    u.params.magic = OTA_PARAMS_MAGIC;
    u.params.sx_freq_reg = sx_freq_reg;
    u.params.sx_sf = OTA_SX_SF;
    u.params.sx_bw = OTA_SX_BW;
    u.params.sx_cr = OTA_SX_CR;
    u.params.sx_power = OTA_SX(POWER_MIN); // receiver and transmitter sit next to each other
    u.params.session_id = session_id;
    u.params.spare = 0;
    u.params.check = ~(u.w[1] ^ u.w[2] ^ u.w[3]);

    HAL_FLASH_Unlock();
    FLASH_ErasePage(OTA_PARAMS_BASE, (OTA_PARAMS_BASE - 0x08000000) / OTA_FLASH_PAGE_SIZE);
    for (uint8_t i = 0; i < 6; i += 2) {
        FLASH_ProgramDoubleWord(OTA_PARAMS_BASE + 4 * i, ((uint64_t)u.w[i + 1] << 32) | u.w[i]);
    }
    HAL_FLASH_Lock();

    NVIC_SystemReset();
    while (1) {}
}

#endif // ESP32, ESP8266


#endif // OTA_RX_H

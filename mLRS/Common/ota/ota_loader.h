//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// OTA, shared definitions
// image info, radio protocol
//*******************************************************
#ifndef OTA_LOADER_H
#define OTA_LOADER_H
#pragma once

#include <inttypes.h>


#define OTA_LOADER_VERSION        1 // protocol is frozen per version


//-------------------------------------------------------
// Target id
//-------------------------------------------------------
// fnv1a hash of the device name, tells which receiver an image is for
// renaming a device thus makes it a new target, its old firmware then refuses the new image!

constexpr uint32_t ota_fnv1a(const char* s, uint32_t h = 0x811C9DC5)
{
    return (*s) ? ota_fnv1a(s + 1, (h ^ (uint8_t)*s) * 0x01000193) : h;
}

#ifdef DEVICE_IS_RECEIVER
enum : uint32_t { OTA_TARGET_ID = ota_fnv1a(DEVICE_NAME) };
#endif


//-------------------------------------------------------
// App image info
//-------------------------------------------------------
// sits somewhere in the image, for the host tool, to find out which target the image is for

#define OTA_APP_INFO_MAGIC        0x4F4C524D // 'MRLO'

typedef struct
{
    uint32_t magic;
    uint32_t target_id;
    uint32_t length; // 0, the image has its own checksum
    uint32_t version;
} tOtaAppInfo;


//-------------------------------------------------------
// Radio protocol
//-------------------------------------------------------
// Stop-and-wait, tx sends, rx responds to each packet.
// tx -> rx: cmd, session_id[2], payload
// rx -> tx: cmd | 0x80, session_id[2], status, payload
// tx and rx must have sx chips which can talk to each other on the band

// link settings, the frequency is the bind frequency of the band
// 2.4 GHz: LoRa, the fastest the chip does in mLRS, explicit header, radio crc on
// else: GFSK, as in the 50 Hz mode, see OTA_FSK further below
// a LR11xx does both, and uses the one of the band it is connected on
// OTA_SX(NAME) gives the SX126X_NAME, SX1280_NAME or LR11XX_NAME constant of the chip
#if defined DEVICE_HAS_SX128x
  #define OTA_SX(name)            SX1280_##name
  #define OTA_SX_SF               SX1280_LORA_SF5
  #define OTA_SX_BW               SX1280_LORA_BW_800
  #define OTA_SX_CR               SX1280_LORA_CR_LI_4_5
#else
  #define OTA_USE_FSK
  #if defined DEVICE_HAS_LR11xx
    #define OTA_SX(name)          LR11XX_##name
  #else
    #define OTA_SX(name)          SX126X_##name
  #endif
  #define OTA_SX_SF               0 // not used
  #define OTA_SX_BW               0
  #define OTA_SX_CR               0
#endif

#define OTA_BLOCK_SIZE            128 // divides page size, multiple of 8
#define OTA_PACKET_HEADER_LEN     3
#define OTA_PACKET_LEN_MAX        (OTA_PACKET_HEADER_LEN + 2 + OTA_BLOCK_SIZE)
#define OTA_RESPONSE_LEN_MAX      (OTA_PACKET_HEADER_LEN + 12) // hello

typedef enum {
    OTA_CMD_HELLO = 1,  // -              -> status, loader_version, block_size, target_id[4], app_size_max[4], flags
    OTA_CMD_BEGIN,      // length[4], target_id[4], flags, image_length[4] -> status, next_block[2]
    OTA_CMD_DATA,       // block[2], data[n] -> status, next_block[2]
    OTA_CMD_END,        // crc32[4]       -> status, next_block[2], rx reboots if ok, crc32 (zlib) of what was transferred
    OTA_CMD_RESPONSE = 0x80,
} OTA_CMD_ENUM;

// hello: what the rx can do, begin: what the tx wants
// length is what is transferred, image_length what it becomes in flash, they differ only with deflate
typedef enum {
    OTA_FLAG_DEFLATE = 0x01, // data is a zlib stream
    OTA_FLAG_GZIP = 0x02, // data is a gzip stream, which the rx stores as it is
} OTA_FLAG_ENUM;

typedef enum {
    OTA_STATUS_OK = 0,
    OTA_STATUS_ERR_TARGET,
    OTA_STATUS_ERR_LENGTH,
    OTA_STATUS_ERR_STATE,
    OTA_STATUS_ERR_FLASH,
    OTA_STATUS_ERR_IMAGE,
    OTA_STATUS_ERR_UNSUPPORTED,
    OTA_STATUS_ERR_DATA,
} OTA_STATUS_ENUM;


//-------------------------------------------------------
// GFSK link
//-------------------------------------------------------
// Radio settings as in the 50 Hz mode: 100 kbps, fixed length, no radio crc, whitening.
// So each direction has its fixed frame length, and the crc is ours:
// len, packet[len], padding, crc16[2]

#define OTA_FSK_SYNCWORD          0x2DD4
#define OTA_FSK_BITRATE_BPS       100000
#define OTA_FSK_FDEV_HZ           50000
#define OTA_FSK_FRAME_LEN_TX      (OTA_PACKET_LEN_MAX + 3) // tx -> rx
#define OTA_FSK_FRAME_LEN_RX      (OTA_RESPONSE_LEN_MAX + 3) // rx -> tx

// = fmav_crc_calculate()
static inline uint16_t ota_crc16(const uint8_t* buf, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        uint8_t tmp = *buf++ ^ (uint8_t)crc;
        tmp ^= (tmp << 4);
        crc = (crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ (tmp >> 4);
    }
    return crc;
}

// makes the packet in buf into a frame, in place, buf must have room for frame_len
static inline void ota_fsk_frame_pack(uint8_t* buf, uint8_t len, uint8_t frame_len)
{
    for (uint8_t n = len; n > 0; n--) buf[n] = buf[n - 1];
    buf[0] = len;
    for (uint8_t n = len + 1; n < frame_len - 2; n++) buf[n] = 0;
    uint16_t crc = ota_crc16(buf, frame_len - 2);
    buf[frame_len - 2] = (uint8_t)crc;
    buf[frame_len - 1] = (uint8_t)(crc >> 8);
}

// makes the frame in buf into a packet, in place, returns its length, 0 if the frame is bad
static inline uint8_t ota_fsk_frame_unpack(uint8_t* buf, uint8_t frame_len)
{
    uint16_t crc = ota_crc16(buf, frame_len - 2);
    if (buf[frame_len - 2] != (uint8_t)crc || buf[frame_len - 1] != (uint8_t)(crc >> 8)) return 0;
    uint8_t len = buf[0];
    if (len > frame_len - 3) return 0;
    for (uint8_t n = 0; n < len; n++) buf[n] = buf[n + 1];
    return len;
}


#endif // OTA_LOADER_H

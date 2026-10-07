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

// link settings, the frequency is the bind frequency of the band, all is LoRa with the settings of a mode
// 2.4 GHz: 50 Hz mode, sub-GHz: 31 Hz mode, both with explicit header and radio crc on
// sub-GHz in 19 Hz 7x mode: that mode, a SX127x can do its SF6 only with implicit header, see ota_link.h
// a LR11xx and a LR20xx do all three, and use the one of the band and mode it is connected on
// OTA_SX(NAME) gives the SX126X_NAME, SX1276_NAME, SX1280_NAME, LR11XX_NAME or LR20XX_NAME constant of the chip
#if defined DEVICE_HAS_SX128x
  #define OTA_SX(name)            SX1280_##name
  #define OTA_SX_SF               SX1280_LORA_SF5
  #define OTA_SX_BW               SX1280_LORA_BW_800
  #define OTA_SX_CR               SX1280_LORA_CR_LI_4_5
#elif defined DEVICE_HAS_LR11xx
  #define OTA_SX(name)            LR11XX_##name // its settings are in ota_link.h
#elif defined DEVICE_HAS_LR20xx
  #define OTA_SX(name)            LR20XX_##name // its settings are in ota_link.h
#elif defined DEVICE_HAS_SX127x
  #define OTA_SX(name)            SX1276_##name
  #define OTA_SX_SF               SX1276_LORA_SF6
  #define OTA_SX_BW               SX1276_LORA_BW_500
  #define OTA_SX_CR               SX1276_LORA_CR_4_5
#else
  #define OTA_SX(name)            SX126X_##name
  #define OTA_SX_SF               SX126X_LORA_SF5
  #define OTA_SX_BW               SX126X_LORA_BW_500
  #define OTA_SX_CR               SX126X_LORA_CR_4_5
#endif

// the receiver tells its block size in hello, the tx can do any up to the largest
// loader: divides page size, multiple of 8, ESP: a packet must fit into a frame of the implicit header link
#if defined DEVICE_IS_RECEIVER && !(defined ESP32 || defined ESP8266)
  #define OTA_BLOCK_SIZE          128
#else
  #define OTA_BLOCK_SIZE          240
#endif
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


#endif // OTA_LOADER_H

//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// OTA, the radio link, as tx relay and rx do it with the sx driver, see ota_loader.h
//*******************************************************
#ifndef OTA_LINK_H
#define OTA_LINK_H
#pragma once


#include "ota_loader.h"


// implicit header link, a SX127x has no other, a LR11xx and a LR20xx use it to talk to one
#if defined DEVICE_HAS_SX128x
  #define OTA_LINK_IS_IMPLICIT      false
  #define OTA_SX_IRQ_RX_ERROR       (SX1280_IRQ_CRC_ERROR | SX1280_IRQ_HEADER_ERROR)
#elif defined DEVICE_HAS_LR11xx
  #define OTA_LINK_IS_IMPLICIT      (Config.Mode == MODE_19HZ_7X)
  #define OTA_SX_IRQ_RX_ERROR       (LR11XX_IRQ_PACKET_ERROR | LR11XX_IRQ_HEADER_ERROR)
#elif defined DEVICE_HAS_LR20xx
  #define OTA_LINK_IS_IMPLICIT      (Config.Mode == MODE_19HZ_7X)
  #define OTA_SX_IRQ_RX_ERROR       (LR20XX_IRQ_CRC_ERROR | LR20XX_IRQ_LORA_HEADER_ERR)
#elif defined DEVICE_HAS_SX127x
  #define OTA_LINK_IS_IMPLICIT      true
  #define OTA_SX_IRQ_RX_ERROR       0
#else
  #define OTA_LINK_IS_IMPLICIT      false
  #define OTA_SX_IRQ_RX_ERROR       (SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR)
#endif


//-------------------------------------------------------
// Implicit header link
//-------------------------------------------------------
// Fixed length, no radio crc. So each direction has its fixed frame length, and the crc is ours:
// len, packet[len], padding, crc16[2]

#define OTA_FRAME_LEN_TX            (OTA_PACKET_LEN_MAX + 3) // tx -> rx
#define OTA_FRAME_LEN_RX            (OTA_RESPONSE_LEN_MAX + 3) // rx -> tx

STATIC_ASSERT(OTA_FRAME_LEN_TX <= 255, "OTA frame doesn't fit a LoRa packet")

// makes the packet in buf into a frame, in place, buf must have room for frame_len
void ota_frame_pack(uint8_t* buf, uint8_t len, uint8_t frame_len)
{
    for (uint8_t n = len; n > 0; n--) buf[n] = buf[n - 1];
    buf[0] = len;
    for (uint8_t n = len + 1; n < frame_len - 2; n++) buf[n] = 0;
    uint16_t crc = fmav_crc_calculate(buf, frame_len - 2);
    buf[frame_len - 2] = (uint8_t)crc;
    buf[frame_len - 1] = (uint8_t)(crc >> 8);
}

// makes the frame in buf into a packet, in place, returns its length, 0 if the frame is bad
uint8_t ota_frame_unpack(uint8_t* buf, uint8_t frame_len)
{
    uint16_t crc = fmav_crc_calculate(buf, frame_len - 2);
    if (buf[frame_len - 2] != (uint8_t)crc || buf[frame_len - 1] != (uint8_t)(crc >> 8)) return 0;
    uint8_t len = buf[0];
    if (len > frame_len - 3) return 0;
    for (uint8_t n = 0; n < len; n++) buf[n] = buf[n + 1];
    return len;
}


//-------------------------------------------------------
// The link
//-------------------------------------------------------

void ota_link_set_packet_len(uint8_t len)
{
#ifdef DEVICE_HAS_LR20xx
    if (OTA_LINK_IS_IMPLICIT) {
        sx.SetLoraPacketParams(12, LR20XX_LORA_HEADER_DISABLE, len, LR20XX_LORA_CRC_DISABLE, LR20XX_LORA_IQ_NORMAL);
        return;
    }
    sx.SetLoraPacketParams(12, LR20XX_LORA_HEADER_EXPLICIT, len, LR20XX_LORA_CRC_ENABLE, LR20XX_LORA_IQ_NORMAL);
#else
    if (OTA_LINK_IS_IMPLICIT) {
        sx.SetPacketParams(12, OTA_SX(LORA_HEADER_DISABLE), len, OTA_SX(LORA_CRC_DISABLE), OTA_SX(LORA_IQ_NORMAL));
        return;
    }
    sx.SetPacketParams(12, OTA_SX(LORA_HEADER_EXPLICIT), len, OTA_SX(LORA_CRC_ENABLE), OTA_SX(LORA_IQ_NORMAL));
#endif
}


void ota_link_start(uint32_t sx_freq_reg)
{
#ifdef DEVICE_HAS_SX127x
    sx.SetStandby(); // it is in LoRa mode
#elif defined DEVICE_HAS_LR20xx
    sx.SetStandby(LR20XX_STANDBY_MODE_RC);
    delay_us(1000);
    sx.SetPacketType(LR20XX_PACKET_TYPE_LORA); // it may be in FLRC or GFSK mode
#else
    sx.SetStandby(OTA_SX(STDBY_CONFIG_STDBY_RC));
    delay_us(1000);
    sx.SetPacketType(OTA_SX(PACKET_TYPE_LORA)); // it may be in FLRC or GFSK mode
#endif
    sx.SetRfFrequency(sx_freq_reg);
#ifdef DEVICE_HAS_LR11xx
    // = OTA_SX_xx of the SX128x, SX127x, SX126x
    if (Config.Sx.FrequencyBand == SX_FHSS_FREQUENCY_BAND_2P4_GHZ) {
        sx.SetModulationParams(LR11XX_LORA_SF5, LR11XX_LORA_BW_800, LR11XX_LORA_CR_LI_4_5, LR11XX_LORA_LDR_OFF);
    } else
    if (OTA_LINK_IS_IMPLICIT) {
        sx.SetModulationParams(LR11XX_LORA_SF6, LR11XX_LORA_BW_500, LR11XX_LORA_CR_4_5, LR11XX_LORA_LDR_OFF);
        sx.EnableSx127xCompatibility();
    } else {
        sx.SetModulationParams(LR11XX_LORA_SF5, LR11XX_LORA_BW_500, LR11XX_LORA_CR_4_5, LR11XX_LORA_LDR_OFF);
    }
#elif defined DEVICE_HAS_LR20xx
    // as the LR11xx
    if (Config.Sx.FrequencyBand == SX_FHSS_FREQUENCY_BAND_2P4_GHZ) {
        sx.SetLoraModulationParams(LR20XX_LORA_SF5, LR20XX_LORA_BW_812, LR20XX_LORA_CR_LI_4_5, LR20XX_LORA_LDRO_OFF);
    } else
    if (OTA_LINK_IS_IMPLICIT) {
        sx.SetLoraModulationParams(LR20XX_LORA_SF6, LR20XX_LORA_BW_500, LR20XX_LORA_CR_4_5, LR20XX_LORA_LDRO_OFF);
        sx.EnableSx127xCompatibility();
    } else {
        sx.SetLoraModulationParams(LR20XX_LORA_SF5, LR20XX_LORA_BW_500, LR20XX_LORA_CR_4_5, LR20XX_LORA_LDRO_OFF);
    }
#else
    sx.SetModulationParams(OTA_SX_SF, OTA_SX_BW, OTA_SX_CR);
#endif
}


// makes the packet in buf ready to be sent, returns the length to send, buf must have room for frame_len
uint8_t ota_link_pack(uint8_t* buf, uint8_t len, uint8_t frame_len)
{
    if (!OTA_LINK_IS_IMPLICIT) return len;
    ota_frame_pack(buf, len, frame_len);
    return frame_len;
}


// reads what was received into buf, returns the length of the packet, 0 if it is of no use
uint8_t ota_link_read(uint32_t irq, uint8_t* buf, uint8_t frame_len)
{
    if (!(irq & OTA_SX(IRQ_RX_DONE)) || (irq & OTA_SX_IRQ_RX_ERROR)) return 0;
#ifdef DEVICE_HAS_LR20xx
    // it has a rx fifo, and the driver's ReadBuffer() is only for the few bytes of its isr
    if (OTA_LINK_IS_IMPLICIT) {
        sx.ReadRadioRxFifo(buf, frame_len);
        return ota_frame_unpack(buf, frame_len);
    }
    uint16_t rx_len = sx.GetRxPktLength();
    if (rx_len > OTA_PACKET_LEN_MAX) return 0;
    sx.ReadRadioRxFifo(buf, rx_len);
    return rx_len;
#else
    uint8_t rx_len, rx_start;

    if (OTA_LINK_IS_IMPLICIT) {
        sx.ReadBuffer(0, buf, frame_len); // as the driver's ReadFrame(), the chip doesn't tell the length
        return ota_frame_unpack(buf, frame_len);
    }
    sx.GetRxBufferStatus(&rx_len, &rx_start);
    if (rx_len > OTA_PACKET_LEN_MAX) return 0;
    sx.ReadBuffer(rx_start, buf, rx_len);
    return rx_len;
#endif
}


#endif // OTA_LINK_H

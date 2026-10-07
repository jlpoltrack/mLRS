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


// a LR11xx does both links, and uses the one of the band it is on
#if defined DEVICE_HAS_SX128x
  #define OTA_LINK_IS_FSK           false
  #define OTA_SX_IRQ_RX_ERROR       (SX1280_IRQ_CRC_ERROR | SX1280_IRQ_HEADER_ERROR)
#elif defined DEVICE_HAS_LR11xx
  #define OTA_LINK_IS_FSK           (Config.Sx.FrequencyBand != SX_FHSS_FREQUENCY_BAND_2P4_GHZ)
  #define OTA_SX_IRQ_RX_ERROR       (LR11XX_IRQ_PACKET_ERROR | LR11XX_IRQ_HEADER_ERROR)
#else
  #define OTA_LINK_IS_FSK           true
  #define OTA_SX_IRQ_RX_ERROR       (SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR)
#endif


void ota_link_set_packet_len(uint8_t len)
{
#ifdef OTA_USE_FSK
    if (OTA_LINK_IS_FSK) {
        // as the GfskConfiguration[] of the driver, with our length
        sx.SetPacketParamsGFSK(16, OTA_SX(GFSK_PREAMBLE_DETECTOR_LENGTH_8BITS), 16, OTA_SX(GFSK_ADDRESS_FILTERING_DISABLE),
                               OTA_SX(GFSK_PKT_FIX_LEN), len, OTA_SX(GFSK_CRC_OFF), OTA_SX(GFSK_WHITENING_ENABLE));
        return;
    }
#endif
    sx.SetPacketParams(12, OTA_SX(LORA_HEADER_EXPLICIT), len, OTA_SX(LORA_CRC_ENABLE), OTA_SX(LORA_IQ_NORMAL));
}


void ota_link_start(uint32_t sx_freq_reg)
{
    sx.SetStandby(OTA_SX(STDBY_CONFIG_STDBY_RC));
    delay_us(1000);
    sx.SetPacketType((OTA_LINK_IS_FSK) ? OTA_SX(PACKET_TYPE_GFSK) : OTA_SX(PACKET_TYPE_LORA)); // it may be in FLRC mode
    sx.SetRfFrequency(sx_freq_reg);
#ifdef OTA_USE_FSK
    if (OTA_LINK_IS_FSK) { sx.SetGfskConfigurationByIndex(0, OTA_FSK_SYNCWORD); return; }
#endif
#ifdef DEVICE_HAS_LR11xx
    sx.SetModulationParams(LR11XX_LORA_SF5, LR11XX_LORA_BW_800, LR11XX_LORA_CR_LI_4_5, LR11XX_LORA_LDR_OFF); // = OTA_SX_xx of the SX128x
#else
    sx.SetModulationParams(OTA_SX_SF, OTA_SX_BW, OTA_SX_CR);
#endif
}


// makes the packet in buf ready to be sent, returns the length to send, GFSK: buf must have room for fsk_frame_len
uint8_t ota_link_pack(uint8_t* buf, uint8_t len, uint8_t fsk_frame_len)
{
    if (!OTA_LINK_IS_FSK) return len;
    ota_fsk_frame_pack(buf, len, fsk_frame_len);
    return fsk_frame_len;
}


// reads what was received into buf, returns the length of the packet, 0 if it is of no use
uint8_t ota_link_read(uint32_t irq, uint8_t* buf, uint8_t fsk_frame_len)
{
    uint8_t rx_len, rx_start;

    if (!(irq & OTA_SX(IRQ_RX_DONE)) || (irq & OTA_SX_IRQ_RX_ERROR)) return 0;
    sx.GetRxBufferStatus(&rx_len, &rx_start);
    if ((OTA_LINK_IS_FSK) ? (rx_len != fsk_frame_len) : (rx_len > OTA_PACKET_LEN_MAX)) return 0;
    sx.ReadBuffer(rx_start, buf, rx_len);
    return (OTA_LINK_IS_FSK) ? ota_fsk_frame_unpack(buf, rx_len) : rx_len;
}


#endif // OTA_LINK_H

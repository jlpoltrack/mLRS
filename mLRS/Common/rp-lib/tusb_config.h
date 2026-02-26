//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// TinyUSB configuration override
// shadows framework-arduinopico/include/tusb_config.h
// to allow custom CDC buffer sizes
//********************************************************
#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

#define CFG_TUSB_RHPORT0_MODE     OPT_MODE_DEVICE
#define CFG_TUSB_OS               OPT_OS_PICO

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG           0
#endif

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN          __attribute__ ((aligned(4)))
#endif

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE    64
#endif

//------------- CLASS -------------//
#define CFG_TUD_HID              (2)
#define CFG_TUD_CDC              (1)
#define CFG_TUD_MSC              (1)
#define CFG_TUD_MIDI             (1)
#define CFG_TUD_VENDOR           (0)

// increased from default 256 to reduce blocking in SerialUSB::write()
#define CFG_TUD_CDC_RX_BUFSIZE  (2048)
#define CFG_TUD_CDC_TX_BUFSIZE  (2048)

#define CFG_TUD_MSC_EP_BUFSIZE  (64)

// HID buffer size should be sufficient to hold ID (if any) + Data
#define CFG_TUD_HID_EP_BUFSIZE  (64)

// MIDI
#define CFG_TUD_MIDI_RX_BUFSIZE (64)
#define CFG_TUD_MIDI_TX_BUFSIZE (64)

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_CONFIG_H_ */

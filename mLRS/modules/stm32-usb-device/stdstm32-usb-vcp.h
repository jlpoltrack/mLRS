//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// STM32_USB_Device_Library based USB VCP standard library
//*******************************************************
// ../Drivers/STM32_USB_Device_Library/Core/Inc
// ../Drivers/STM32_USB_Device_Library/Class/CDC/Inc
// "${workspace_loc:/${ProjName}/modules/stm32-usb-device}"
// STDSTM32_USE_USB
// #define HAL_PCD_MODULE_ENABLED in stm32yyxx_hal_conf.h

#ifndef STDSTM32_USB_VCP
#define STDSTM32_USB_VCP
#ifdef __cplusplus
extern "C" {
#endif
#include "usbd_conf.h" // for the device selection, it is what defines STM32C5 for a C5 target

#if defined STM32C5 // C5 ships ST's HAL2, where the module switch is named differently
  #if !defined USE_HAL_PCD_MODULE || (USE_HAL_PCD_MODULE != 1)
    #error USE_HAL_PCD_MODULE not set to 1, set it in Core\Inc\stm32c5xx_hal_conf.h!
  #endif
#elif !defined HAL_PCD_MODULE_ENABLED
  #error HAL_PCD_MODULE_ENABLED not defined, enable it in Core\Inc\stm32yyxx_hal_conf.h!
#endif

#if !defined STM32G431xx && !defined STM32G441xx && !defined STM32G491xx && !defined STM32G474xx
  #warning NAK flow control not tested on non STM32G4 MCUs!
#endif



#include "usbd_cdc.h"


// pl adjust these settings in usbd_conf.h according to your needs
// - USB_RXBUFSIZE
// - USB_TXBUFSIZE
// - USBD_IRQ_PRIORITY


//-------------------------------------------------------
// User Interface
//-------------------------------------------------------

uint8_t usb_rx_available(void);
uint16_t usb_rx_bytesavailable(void);
char usb_getc(void);
void usb_getbuf(uint8_t* const buf, uint16_t len);

uint8_t usb_tx_full(void);
void usb_putc(uint8_t c);
void usb_puts(const char* s);
void usb_putbuf(uint8_t* const buf, uint16_t len);

void usb_rx_flush(void);
void usb_flush(void);

uint32_t usb_baudrate(void);
uint8_t usb_dtr_rts(void);
uint8_t usb_dtr_is_set(void);
uint8_t usb_rts_is_set(void);

void usb_init(void);

void usb_deinit(void);


//-------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif // STDSTM32_USB_VCP


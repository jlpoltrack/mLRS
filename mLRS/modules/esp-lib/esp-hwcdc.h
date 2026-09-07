//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP USB CDC on USB-Serial-JTAG
//********************************************************
// For ESP32C3, ESP32S3, HWCDC on the USB-Serial-JTAG peripheral.
// same usb_xxx() interface as stdstm32-usb-vcp.h.
// usefull resource
// - https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/api-reference/peripherals/usb_serial_jtag_console.html
//********************************************************
#ifndef ESPLIB_HWCDC_H
#define ESPLIB_HWCDC_H


//-------------------------------------------------------
// Guards
//-------------------------------------------------------

// = 0 selects the native USB peripheral, and leaves USBSerial undeclared
#if !ARDUINO_USB_MODE
  #error ARDUINO_USB_MODE must be 1 to use HWCDC!
#endif

// = 1 makes Serial the HWCDC and moves UART0 to Serial0, which would
// silently retarget any UARTx_USE_SERIAL in the hal onto USB
#if ARDUINO_USB_CDC_ON_BOOT
  #error ARDUINO_USB_CDC_ON_BOOT must be 0, else Serial is not UART0!
#endif


//-------------------------------------------------------
// Defines
//-------------------------------------------------------

#include "HWCDC.h"

#ifndef USB_TXBUFSIZE
  #define USB_TXBUFSIZE         2048 // is the tx ring buffer inside HWCDC
#endif

#ifndef USB_RXBUFSIZE
  #define USB_RXBUFSIZE         2048 // is the rx queue inside HWCDC
#endif


//-------------------------------------------------------
// TX routines
//-------------------------------------------------------

IRAM_ATTR uint8_t usb_tx_full(void)
{
    return (USBSerial.availableForWrite() > 0) ? 0 : 1;
}


IRAM_ATTR void usb_putbuf(uint8_t* const buf, uint16_t len)
{
    int space = USBSerial.availableForWrite();
    if (space <= 0) return;
    if (len > space) len = space; // drop what doesn't fit, as the STM32 driver does

    USBSerial.write(buf, len);
}


IRAM_ATTR void usb_putc(uint8_t c)
{
    usb_putbuf(&c, 1);
}


IRAM_ATTR void usb_puts(const char* s)
{
    usb_putbuf((uint8_t*)s, strlen(s));
}


//-------------------------------------------------------
// RX routines
//-------------------------------------------------------

IRAM_ATTR uint8_t usb_rx_available(void)
{
    return (USBSerial.available() > 0) ? 1 : 0;
}


IRAM_ATTR uint16_t usb_rx_bytesavailable(void)
{
    int available = USBSerial.available();
    return (available > 0) ? available : 0;
}


IRAM_ATTR char usb_getc(void)
{
    return (char)USBSerial.read();
}


IRAM_ATTR void usb_getbuf(uint8_t* const buf, uint16_t len)
{
    USBSerial.read(buf, len);
}


//-------------------------------------------------------
// Flush routines
//-------------------------------------------------------

IRAM_ATTR void usb_rx_flush(void)
{
    while (USBSerial.available() > 0) USBSerial.read();
}

IRAM_ATTR void usb_flush(void)
{
    usb_rx_flush();
}


//-------------------------------------------------------
// Line state routines
//-------------------------------------------------------

IRAM_ATTR uint32_t usb_baudrate(void)
{
    return USBSerial.baudRate();
}


IRAM_ATTR uint8_t usb_dtr_rts(void) { return 0; }
IRAM_ATTR uint8_t usb_dtr_is_set(void) { return 0; }
IRAM_ATTR uint8_t usb_rts_is_set(void) { return 0; }


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

void usb_init(void)
{
    USBSerial.setRxBufferSize(USB_RXBUFSIZE); // must be before begin(), else they default to 256
    USBSerial.setTxBufferSize(USB_TXBUFSIZE);
    USBSerial.begin();
}


void usb_deinit(void)
{
    USBSerial.end();
}


#endif // ESPLIB_HWCDC_H

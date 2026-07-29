//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP USB CDC
//********************************************************
// For ESP32S2, ESP32S3, which have a native USB peripheral.
// Uses the TinyUSB CDC device class via the Arduino USBCDC wrapper.
// Provides the same usb_xxx() interface as the STM32 VCP driver in
// modules/stm32-usb-device/stdstm32-usb-vcp.h, so tUsbPort in common.h
// and the DEVICE_HAS_COM_ON_USB code paths work unchanged.
// see esp-usb.md for how to enable it on a target
// useful resource
// - https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/usb_device.html
//********************************************************
#ifndef ESPLIB_USB_H
#define ESPLIB_USB_H


//-------------------------------------------------------
// Guards
//-------------------------------------------------------
// the Arduino core decides at compile time which USB peripheral is
// used, and which object is called Serial. both must be right, or
// things break in confusing ways.

#if !defined CONFIG_IDF_TARGET_ESP32S2 && !defined CONFIG_IDF_TARGET_ESP32S3
  #error USB CDC needs an ESP32 with native USB, i.e. an ESP32S2 or ESP32S3!
#endif

// = 1 selects the USB-Serial-JTAG peripheral (HWCDC), whose baudRate() is hardcoded
// to 115200, so the esp wifi bridge passthrough could not track the baudrate esptool
// asks for. it reports no DTR/RTS either, needed only if ESP_DTR_RTS_USB is used.
#if ARDUINO_USB_MODE
  #error ARDUINO_USB_MODE must be 0 to use USB CDC, see esp-usb.md!
#endif

// = 1 makes Serial the USB CDC and moves UART0 to Serial0, which would
// silently retarget any UARTx_USE_SERIAL in the hal onto USB
#if ARDUINO_USB_CDC_ON_BOOT
  #error ARDUINO_USB_CDC_ON_BOOT must be 0, else Serial is not UART0, see esp-usb.md!
#endif

#ifndef CONFIG_TINYUSB_CDC_ENABLED
  #error TinyUSB CDC is not enabled in the sdkconfig of this Arduino core!
#endif


//-------------------------------------------------------
// Defines
//-------------------------------------------------------

#include "USB.h"
#include "USBCDC.h"

#ifndef USB_TXBUFSIZE
  #define USB_TXBUFSIZE         2048 // helps with cli, MUST be 2^N
#endif
#if (USB_TXBUFSIZE & (USB_TXBUFSIZE - 1)) != 0
  #error USB_TXBUFSIZE must be a power of 2!
#endif
#define USB_TXBUFSIZEMASK       (USB_TXBUFSIZE-1)

#ifndef USB_RXBUFSIZE
  #define USB_RXBUFSIZE         2048 // for serial, is the rx queue inside USBCDC, needs not be 2^N
#endif

// max packet size of a full speed bulk endpoint, no point in ever handing over more
// in one go since the TinyUSB CDC tx fifo is only CONFIG_TINYUSB_CDC_TX_BUFSIZE = 64
// bytes, and that is baked into the precompiled lib
#define USB_TX_PACKET_SIZE      64

// max packets pushed per usb_tx_do() call, keeps the call time bounded
#define USB_TX_PACKETS_MAX      4

#define USB_DTR_FLAG            0x01
#define USB_RTS_FLAG            0x02

USBCDC usb_cdc;

volatile uint8_t usb_txbuf[USB_TXBUFSIZE];
volatile uint16_t usb_txwritepos; // pos at which the last byte was stored
volatile uint16_t usb_txreadpos; // pos at which the next byte is to be fetched

volatile uint8_t usb_dtr_rts_flags; // to track DTR RTS state, is set from the USB event task


//-------------------------------------------------------
// TX routines
//-------------------------------------------------------
// Note: USBCDC::write() busy spins for as long as the host has the port open
// but doesn't drain the fifo. its tx timeout does not bound that spin, it only
// bounds the taking of the tx mutex. so it must never be given more than it can
// take right now. availableForWrite() reports exactly that, and can only grow
// behind our back since the main loop is the only writer, so a write of at most
// that many bytes can't spin.

// pushes at most one packet, doesn't block, returns 1 if it pushed something
IRAM_ATTR uint8_t _usb_transmit(void)
{
    if (usb_txwritepos == usb_txreadpos) return 0; // nothing to send

    int space = usb_cdc.availableForWrite(); // is 0 if the host is not connected
    if (space <= 0) return 0;
    if (space > USB_TX_PACKET_SIZE) space = USB_TX_PACKET_SIZE;

    uint8_t buf[USB_TX_PACKET_SIZE];
    uint16_t len = 0;

    while ((usb_txwritepos != usb_txreadpos) && (len < space)) {
        usb_txreadpos = (usb_txreadpos + 1) & USB_TXBUFSIZEMASK;
        buf[len++] = usb_txbuf[usb_txreadpos];
    }

    // if the host vanished in between, write() returns 0 and these bytes are lost,
    // which is what we want in that situation anyway
    usb_cdc.write(buf, len);
    return 1;
}


// pumps the tx buffer, doesn't block
// is called from usb_putbuf() and usb_tx_full(), and should also be called
// regularly from the main loop, see esp-usb.md
IRAM_ATTR void usb_tx_do(void)
{
    for (uint8_t i = 0; i < USB_TX_PACKETS_MAX; i++) {
        if (!_usb_transmit()) break;
    }
}


IRAM_ATTR uint8_t _usb_putc(uint8_t c)
{
    uint16_t next = (usb_txwritepos + 1) & USB_TXBUFSIZEMASK;
    if (usb_txreadpos != next) { // fifo not full
        usb_txbuf[next] = c;
        usb_txwritepos = next;
        return 1;
    }
    return 0; // fifo full, drop it
}


IRAM_ATTR uint8_t usb_tx_full(void)
{
    usb_tx_do(); // make room if we can, is also the pump during esp passthrough
    uint16_t next = (usb_txwritepos + 1) & USB_TXBUFSIZEMASK;
    return !(usb_txreadpos != next); // not fifo not full
}


IRAM_ATTR void usb_putbuf(uint8_t* const buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) _usb_putc(buf[i]);
    usb_tx_do();
}


IRAM_ATTR void usb_putc(uint8_t c)
{
    _usb_putc(c);
    usb_tx_do();
}


IRAM_ATTR void usb_puts(const char* s)
{
    while (*s) { _usb_putc(*s); s++; }
    usb_tx_do();
}


//-------------------------------------------------------
// RX routines
//-------------------------------------------------------
// USBCDC has its own rx queue, which is filled from the USB task,
// so no second buffer is needed here. none of these block.

IRAM_ATTR uint8_t usb_rx_available(void)
{
    return (usb_cdc.available() > 0) ? 1 : 0;
}


IRAM_ATTR uint16_t usb_rx_bytesavailable(void)
{
    int available = usb_cdc.available();
    return (available > 0) ? available : 0;
}


IRAM_ATTR char usb_getc(void)
{
    return (char)usb_cdc.read();
}


IRAM_ATTR void usb_getbuf(uint8_t* const buf, uint16_t len)
{
    usb_cdc.read(buf, len);
}


//-------------------------------------------------------
// Flush routines
//-------------------------------------------------------

IRAM_ATTR void usb_rx_flush(void)
{
    while (usb_cdc.available() > 0) usb_cdc.read();
}


IRAM_ATTR void usb_tx_flush(void)
{
    usb_txwritepos = usb_txreadpos = 0;
}


IRAM_ATTR void usb_flush(void)
{
    usb_tx_flush();
    usb_rx_flush();
}


//-------------------------------------------------------
// Line state routines
//-------------------------------------------------------
// the dtr, rts fields in USBCDC are protected and have no getter, so
// they are tracked here from the line state event, bit 0 = DTR,
// bit 1 = RTS, same as on STM32

void _usb_line_state_cb(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_id != ARDUINO_USB_CDC_LINE_STATE_EVENT) return;

    arduino_usb_cdc_event_data_t* data = (arduino_usb_cdc_event_data_t*)event_data;
    usb_dtr_rts_flags = (data->line_state.dtr ? USB_DTR_FLAG : 0) + (data->line_state.rts ? USB_RTS_FLAG : 0);
}


IRAM_ATTR uint32_t usb_baudrate(void)
{
    return usb_cdc.baudRate();
}


IRAM_ATTR uint8_t usb_dtr_rts(void)
{
    return usb_dtr_rts_flags;
}


IRAM_ATTR uint8_t usb_dtr_is_set(void)
{
    return (usb_dtr_rts_flags & USB_DTR_FLAG) ? 1 : 0;
}


IRAM_ATTR uint8_t usb_rts_is_set(void)
{
    return (usb_dtr_rts_flags & USB_RTS_FLAG) ? 1 : 0;
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

void usb_init(void)
{
    usb_txwritepos = usb_txreadpos = 0;
    usb_dtr_rts_flags = 0;

    // with reboot enabled USBCDC watches DTR/RTS for the esptool reset sequence and
    // for a 1200 baud touch, and reboots the mcu into the bootloader on either, which
    // is how the host flashes this mcu. it is on unless the hal wants the raw lines:
    // the wifi bridge is flashed with esptool --before no_reset and mLRS drives the
    // bridge's RESET,GPIO0 itself in EnterFlash(), so the lines are normally free.
#ifdef ESP_DTR_RTS_USB
    // the hal wants the raw lines to reset the wifi bridge, so the reboot state
    // machine must be off, it swallows the line state events while it is armed and
    // usb_dtr_is_set(), usb_rts_is_set() would never see them. the price is that the
    // host can then no longer put this mcu into the bootloader.
    usb_cdc.enableReboot(false);
#else
    usb_cdc.enableReboot(true);
#endif

    usb_cdc.setRxBufferSize(USB_RXBUFSIZE); // must be before begin(), else it defaults to 256
    usb_cdc.setTxTimeoutMs(0); // we never write more than availableForWrite(), so belt and braces
    usb_cdc.onEvent(ARDUINO_USB_CDC_LINE_STATE_EVENT, _usb_line_state_cb);
    usb_cdc.begin();

    // these must all be before USB.begin()
    USB.manufacturerName("mLRS");
#ifdef DEVICE_NAME
    USB.productName(DEVICE_NAME);
#endif
    USB.begin();
}


void usb_deinit(void)
{
    usb_cdc.end();
}


#endif // ESPLIB_USB_H

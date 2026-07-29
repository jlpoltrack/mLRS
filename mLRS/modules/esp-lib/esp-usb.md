# esp-usb.h — Serial/Com over native USB on ESP32S2/S3

`esp-usb.h` implements the `usb_xxx()` interface that mLRS expects for a `DEVICE_HAS_COM_ON_USB`
target, on ESP32 chips that have a native USB peripheral (S2, S3). It is the ESP counterpart of
`modules/stm32-usb-device/stdstm32-usb-vcp.h`.

Nothing above the driver changes: `tUsbPort` in `Common/common.h` and `tSerialPorts::com_port()`
already handle `DEVICE_HAS_COM_ON_USB`, so the USB port is used as the Com (CLI) port, and becomes
the Serial port too when `SerialPort` is set to `com`.

## Why it does not simply call `USBCDC::write()`

`USBCDC::write()` is not safe to call from the mLRS main loop. Its inner loop is

```c
size_t space = tud_cdc_n_write_available(itf);
if (!space) { tud_cdc_n_write_flush(itf); continue; }   // no timeout on this
```

The tx timeout that `setTxTimeoutMs()` sets bounds only the tx mutex, not this spin. So whenever the
host has the port open but is not reading, `write()` blocks for as long as that lasts, which would
stall the radio loop. The TinyUSB CDC fifo is also only 64 bytes
(`CONFIG_TINYUSB_CDC_TX_BUFSIZE`, baked into the precompiled lib, not settable from `build_flags`),
so this is hit by any transfer bigger than one packet.

The driver therefore keeps its own tx ring buffer and only ever hands `write()` as many bytes as
`availableForWrite()` reports free. That count can only grow behind our back, since the main loop is
the only writer, so such a write can never enter the spin. When the host is gone,
`availableForWrite()` returns 0, the ring buffer fills, `usb_tx_full()` reports full and further
bytes are dropped, exactly like the STM32 driver behaves.

Rx needs none of this: `USBCDC` fills its own queue from the USB task and `read()` never blocks.

## Enabling it on a target

### 1. hal file

```c
#define DEVICE_HAS_COM_ON_USB
```

instead of `DEVICE_HAS_NO_COM`, and define no `UARTC_xxx` (UARTC becomes a dummy port).

GPIO19 and GPIO20 are the USB D-/D+ pins on the S2/S3 and are fixed. Do not assign them to
anything else in the hal.

### 2. platformio.ini

Most S3 board definitions, including `esp32-s3-devkitc-1`, set `-DARDUINO_USB_MODE=1`, which selects
the USB-Serial-JTAG peripheral. That one is unusable here: `HWCDC::baudRate()` is hardcoded to return
115200 and there is no DTR/RTS accessor, which the ESP wifi bridge passthrough in `CommonTx/esp.h`
needs. So unflag it:

```ini
[env:tx-my-target-esp32s3]
extends = env_common_tx_esp32s3
board = esp32-s3-devkitc-1
build_unflags =
  ${env_common_tx_esp32s3.build_unflags}
  -DARDUINO_USB_MODE=1
build_flags =
  ${env_common_tx_esp32s3.build_flags}
  -D ARDUINO_USB_MODE=0
  -D TX_MY_TARGET_ESP32S3
```

Leave `ARDUINO_USB_CDC_ON_BOOT` at its default of 0. With it set to 1 the core renames `Serial` to
the USB CDC and moves UART0 to `Serial0`, which would silently retarget any `UARTx_USE_SERIAL` in the
hal onto USB. `esp-usb.h` refuses to compile if either flag is wrong.

### 3. mlrs-tx.cpp

Add the include in the ESP branch, next to the other esp-lib includes:

```c
#ifdef USE_USB
#include "../modules/esp-lib/esp-usb.h"
#endif
```

`init_once()` already calls `usb_port.InitOnce()` → `usb_init()`. It is deliberately in `init_once()`
and not `init_hw()`, so that a setup reload does not drop the host connection.

### 4. main loop pump

The driver pumps its ring buffer from `usb_putbuf()` and `usb_tx_full()`, which covers the normal
data paths and the esp passthrough loops. Add one call in the 1 ms tick of `main_loop()` as a
backstop, so a partially sent buffer cannot sit there when no further writes come in:

```c
    if (doSysTask()) {
        ...
#ifdef USE_USB
        usb_tx_do();
#endif
```

This bounds the drain rate to `USB_TX_PACKETS_MAX` packets per call, so at least 64 kB/s at the 1 ms
tick, which is far above what the RF link or the CLI can produce. For more, call `usb_tx_do()` from
`tWhileTransmit::handle()` instead, which runs much more often.

## Configuration defines

| define | default | meaning |
| --- | --- | --- |
| `USB_TXBUFSIZE` | 2048 | tx ring buffer, must be a power of 2 |
| `USB_RXBUFSIZE` | 2048 | rx queue inside `USBCDC` |
| `USB_TX_PACKETS_MAX` | 4 | packets pushed per `usb_tx_do()`, bounds call time |

The buffer sizes match what `usbd_conf.h` uses for the G4 targets, 2048 for serial on rx and to help
the cli on tx, so an S3 target behaves like an STM32 one on the same traffic. That is 2 KB of static
DRAM plus a 2048 entry FreeRTOS queue, which is not a concern on an S2/S3.

Set them in the hal file before the include if the defaults do not suit.

## Interface

Same as `stdstm32-usb-vcp.h`, plus `usb_tx_do()` and `usb_tx_flush()`:

```c
uint8_t  usb_rx_available(void);
uint16_t usb_rx_bytesavailable(void);
char     usb_getc(void);
void     usb_getbuf(uint8_t* const buf, uint16_t len);

uint8_t  usb_tx_full(void);
void     usb_putc(uint8_t c);
void     usb_puts(const char* s);
void     usb_putbuf(uint8_t* const buf, uint16_t len);
void     usb_tx_do(void);        // esp only, pumps the tx ring buffer

void     usb_rx_flush(void);
void     usb_tx_flush(void);     // esp only
void     usb_flush(void);

uint32_t usb_baudrate(void);
uint8_t  usb_dtr_rts(void);      // bit 0 = DTR, bit 1 = RTS
uint8_t  usb_dtr_is_set(void);
uint8_t  usb_rts_is_set(void);

void     usb_init(void);
void     usb_deinit(void);
```

## Things to know

**Flashing this mcu over USB works.** `usb_init()` leaves `USBCDC`'s reboot handling on, so the host
can put the mcu into the bootloader the normal way, either with the esptool DTR/RTS reset sequence or
with a 1200 baud touch. Nothing special is needed to flash the target.

This does not clash with flashing the wifi bridge, because that never uses the CDC lines: `EnterFlash()`
in `esp.h` drives the bridge's `ESP_RESET` and `ESP_GPIO0` itself and esptool is run with
`--before no_reset`, so it leaves DTR/RTS alone. Baudrate tracking still works, since only an
exactly-1200 baud line coding triggers the reboot and no flashing tool uses that.

The one combination that does clash is a hal that defines `ESP_DTR_RTS_USB`, i.e. a board that has no
`ESP_RESET`/`ESP_GPIO0` wiring and resets the bridge from the CDC lines instead. `usb_init()` detects
that and turns reboot off, because the reset state machine swallows the line state events while
armed, so `usb_dtr_is_set()` / `usb_rts_is_set()` would never see them. On such a target the host can
no longer put this mcu into the bootloader, and it has to be flashed some other way.

Note the flip side of leaving reboot on: a host tool that happens to drive DTR/RTS through the reset
pattern, or that opens the port at 1200 baud, will reboot the module into the bootloader. That is
standard behaviour for every ESP32S3 board and the accepted price for host flashing.

**Line state and baudrate are advisory.** They are what the host asked for over CDC, not a real UART
setting. `tUsbPort` correctly has no `SetBaudRate()` override, so `Serials.Init()` setting
`TX_COM_BAUDRATE` on the com port is a no-op when com is on USB. `usb_baudrate()` exists only so the
passthrough can track what esptool negotiated.

**Behaviour when unplugged.** `availableForWrite()` returns 0 when the host is not connected, so tx
data accumulates in the ring buffer, and once it is full new bytes are dropped, same as the STM32
driver. Nothing blocks and nothing errors. On reconnect whatever is still buffered is sent first, so
expect a stale partial message at the join; the CLI and MAVLink both resync on their own.

**Task context.** The TinyUSB device task runs at `configMAX_PRIORITIES - 1` and preempts the
Arduino loop task that mLRS runs in, so the fifo drains promptly. The only thing touched from another
task is `usb_dtr_rts_flags`, written by the USB event task in `_usb_line_state_cb()`, which is why it
is `volatile`.

**Rx flow control.** There is NAK backpressure, but it is not free the way it is on STM32. TinyUSB
only arms the OUT endpoint when its rx fifo has room for a full packet, so the host gets NAKed
otherwise. On top of that `USBCDC::_onRX` is called before the endpoint is re-armed and does an
`xQueueSend` per byte with a 10 tick timeout, so with `CONFIG_FREERTOS_HZ=1000` a full rx queue holds
the host off for up to 10 ms per byte before it gives up, drops the rest of the packet and posts
`ARDUINO_USB_CDC_RX_OVERFLOW_EVENT`.

The catch is that this all runs inside `tud_task()`, so while rx is backpressured the USB stack is
blocked and **tx stalls too**. On STM32 the NAK decision sits in the isr and does not touch tx. This
is the main reason `USB_RXBUFSIZE` is 2048 and should not be trimmed.

**Not covered.** `usb_deinit()` is provided for parity but mLRS never calls it.

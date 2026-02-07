# Native WiFi Serial Destination for CYW43439 (Pico W)

## Overview

This document outlines the architecture for adding a native **WiFi** option to the `Tx Ser Dest` parameter, replacing the UART-based ESP WiFi bridge with direct CYW43439 implementation on the Raspberry Pi Pico W.

## Goals

- Add `SERIAL_DESTINATION_WIFI` as a fourth serial destination option
- Implement all five protocols: UDP (Soft AP), TCP (Soft AP), UDPSTA (Station), BT (Classic), BLE
- Reuse existing WiFi configuration parameters (Protocol, Channel, Power)
- Leverage existing multicore isolation (Core 0 utility, Core 1 radio timing per `rp-glue.h`)
- Support both RP2040 and RP2350

---

## Hardware Constraints

### PIO Resource Requirements

The CYW43439 WiFi driver requires an **entire PIO block** for SPI communication with the wireless chip.

| Platform | PIO Blocks | CYW43 Usage | Remaining for mLRS |
|----------|------------|-------------|---------------------|
| RP2040   | 2 (PIO0, PIO1) | 1 full block (PIO1) | 1 block (4 SMs) |
| RP2350   | 3 (PIO0, PIO1, PIO2) | 1 full block (PIO1) | 2 blocks (8 SMs) |

#### Current PIO Assignment

| PIO Block | Usage |
|-----------|-------|
| PIO0 | mLRS peripherals (JR Pin5, PIO UARTs, RGB LEDs) |
| PIO1 | CYW43439 WiFi SPI (SDK default) |

CYW43439 defaults to PIO1 for SPI communication. The mLRS codebase uses PIO0 for all RP peripherals, avoiding any conflicts.

#### Debug and Serial Port Allocation

| UART | Implementation | Notes |
|------|----------------|-------|
| UARTB | USB Serial | MAVLink telemetry via TinyUSB |
| UARTC | USB Serial | CLI via TinyUSB |
| UARTD | Hardware UART1 | ESP WiFi bridge (pins P0/P1) |
| UARTF | Hardware UART2 | Debug output (`UARTF_USE_SERIAL2`) |

> [!NOTE]
> **No PIO conflicts**: All mLRS peripherals use PIO0 and hardware UARTs. CYW43439 WiFi uses PIO1 (SDK default). No build flags required.

---

## WiFi Protocol Support Analysis

The existing ESP bridge supports five protocols. Here's the feasibility for native CYW43439 implementation:

```cpp
typedef enum {
    WIFI_PROTOCOL_TCP = 0,    // ✅ Fully Supportable
    WIFI_PROTOCOL_UDP,        // ✅ Fully Supportable (Primary Target)
    WIFI_PROTOCOL_BT,         // ✅ Supportable via BTstack SPP
    WIFI_PROTOCOL_UDPSTA,     // ✅ Supportable (Client Mode)
    WIFI_PROTOCOL_BLE,        // ✅ Supportable via BTstack NUS
    WIFI_PROTOCOL_NUM,
} TX_WIFI_PROTOCOL_ENUM;
```

### Protocol Details

| Protocol | Mode | Description | Arduino Support |
|----------|------|-------------|------------------|
| `TCP` | Soft AP | TCP server, clients connect to AP | ✅ `WiFiServer` / `WiFiClient` |
| `UDP` | Soft AP | UDP server, broadcast/unicast | ✅ `WiFiUDP` |
| `BT` | Classic BT | Bluetooth SPP serial profile | ✅ `SerialBT` (BTstack SPP) |
| `UDPSTA` | Station Mode | Connect to existing network | ✅ `WiFi.begin()` + `WiFiUDP` |
| `BLE` | BLE GATT | Nordic UART Service (NUS) | ✅ BTstack NUS |

> [!NOTE]
> The CYW43439 is a combo WiFi + Bluetooth chip. The `arduino-pico` core includes **BTstack** which provides both Classic Bluetooth (SPP serial profile) and BLE (GATT/NUS) support. All five protocol modes can be implemented natively.

### Recommended Implementation Phases

1. **Phase 1**: `UDP` (Soft AP) - Primary use case, simplest implementation
2. **Phase 2**: `TCP` (Soft AP) - Add connection tracking
3. **Phase 3**: `UDPSTA` (Station Mode) - Requires credential storage
4. **Phase 4**: `BT` (Classic Bluetooth SPP) - BTstack serial profile integration
5. **Phase 5**: `BLE` (Bluetooth Low Energy NUS) - BTstack Nordic UART Service

---

## WiFi Channel and Power Support

### Channel Configuration

```cpp
typedef enum {
    WIFI_CHANNEL_1 = 0,   // ✅ 2412 MHz
    WIFI_CHANNEL_6,       // ✅ 2437 MHz
    WIFI_CHANNEL_11,      // ✅ 2462 MHz
    WIFI_CHANNEL_13,      // ✅ 2472 MHz (may be restricted in some regions)
    WIFI_CHANNEL_NUM,
} TX_WIFI_CHANNEL_ENUM;
```

All channels are fully supportable. The CYW43439 supports 2.4GHz channels 1-13. Channel selection should avoid overlap with the 2.4GHz LoRa modulation band when applicable.

### Power Configuration

The current enum provides only 3 power levels. Suggested mappings for CYW43439:

| Mode | Current Enum | WiFi Mapping | BT Mapping |
|------|--------------|--------------|------------|
| Low | `WIFI_POWER_LOW` | 0 dBm | 0 dBm |
| Med | `WIFI_POWER_MED` | 10 dBm | 4 dBm |
| Max | `WIFI_POWER_MAX` | 20 dBm | 8 dBm |

**Implementation**: Use `WiFi.setTxPower(dBm)` which wraps the CYW43439 power control internally.

---

## Architecture Design

### Multicore Assignment

```
┌─────────────────────────────────────────────────────────────────┐
│  Core 0 (Utility Core)                                          │
│  ├── USB/TinyUSB stack                                          │
│  ├── Display refresh (Sharp SPI / I2C OLED)                     │
│  └── WiFi Stack (NEW)                                           │
│      ├── WiFiUDP.parsePacket() / WiFiClient polling             │
│      ├── WiFiUDP.write() — drain TX FIFO to network             │
│      └── WiFiUDP.read()  — fill RX FIFO from network            │
├─────────────────────────────────────────────────────────────────┤
│  Core 1 (Real-time Core)                                        │
│  ├── main_loop() - 50Hz/31Hz/19Hz radio timing                  │
│  ├── SX12xx driver (SPI)                                        │
│  ├── JR pin5 PIO UART                                           │
│  └── sx_serial interface                                        │
│      └── wifi_serial.putbuf() → Cross-core buffer write         │
└─────────────────────────────────────────────────────────────────┘
```

### Cross-Core Data Flow

> [!NOTE]
> **Why Lockless?** The WiFi TX path (`sx_serial.putbuf()`) is called from Core 1's `main_loop()` context. Core 0's WiFi stack processes data in its `loop()`. While mutexes could technically work (not ISR context), lockless SPSC FIFOs are preferred because:
> 1. **No blocking**: Core 1 never waits for Core 0, preserving radio timing
> 2. **Lower overhead**: Memory barriers are cheaper than mutex acquire/release
> 3. **Deterministic latency**: No priority inversion or lock contention

#### How SPSC FIFOs Handle Concurrent Access

The FIFO uses two pointers with **exclusive ownership**:

| Pointer | Written By | Read By | Purpose |
|---------|------------|---------|---------|
| `head` | Core 1 only | Both | Next write position |
| `tail` | Core 0 only | Both | Next read position |

**When Core 1 writes while Core 0 reads simultaneously:**

1. **Core 1** writes data to `buf[head]`, executes `__dmb()` (memory barrier), then increments `head`
2. **Core 0** reads data from `buf[tail]`, executes `__dmb()`, then increments `tail`

Since they operate on **different buffer positions**, there's no race. The memory barriers ensure:
- Core 0 sees `head` update **only after** the data is written
- Core 1 sees `tail` update **only after** Core 0 finished reading

**If buffer is full**: Core 1 checks `if (next == tail)` before writing. If true, write fails gracefully (no data loss, caller can retry or drop).

```
                    Core 1 (Radio)              Core 0 (WiFi)
                         │                           │
  ┌──────────────────────┴───────────────────────────┴────────────┐
  │                                                               │
  │   sx_serial.putbuf(mavlink_data)                              │
  │         │                                                     │
  │         ▼                                                     │
  │   ┌─────────────────────────────────────────┐                 │
  │   │  wifi_tx_fifo (lockless ring buffer)    │                 │
  │   │  [Core 1 writes ──────► Core 0 reads]   │                 │
  │   └─────────────────────────────────────────┘                 │
  │                              │                                │
  │                              ▼                                │
  │                        udp.beginPacket() / write()            │
  │                                                               │
  │                                                               │
  │                        udp.parsePacket() / read()             │
  │                              │                                │
  │                              ▼                                │
  │   ┌─────────────────────────────────────────┐                 │
  │   │  wifi_rx_fifo (lockless ring buffer)    │                 │
  │   │  [Core 0 writes ──────► Core 1 reads]   │                 │
  │   └─────────────────────────────────────────┘                 │
  │         │                                                     │
  │         ▼                                                     │
  │   sx_serial.available() / getc()                              │
  │                                                               │
  └───────────────────────────────────────────────────────────────┘
```

---

## Implementation Components

### New Files

| File | Purpose |
|------|---------|
| `Common/rp-lib/wifi_native.h` | Main WiFi AP/STA + UDP/TCP/BT/BLE implementation (`tTxWifiNative`) |
| `Common/rp-lib/crosscore_fifo.h` | Lockless SPSC ring buffer for inter-core data |
| `Common/hal/rp/tx-hal-pico-w-2400.h` | Pico W specific HAL configuration |
| `CommonTx/rp-wifi.h` | WiFi initialization wrapper (analogous to `esp.h`) |

### Modified Files

| File | Changes |
|------|---------|
| [setup_types.h](file:///Users/johnpoltrack/Documents/mLRS/mLRS/Common/setup_types.h) | Add `SERIAL_DESTINATION_WIFI` to enum |
| [setup_list.h](file:///Users/johnpoltrack/Documents/mLRS/mLRS/Common/setup_list.h) | Update option string: `"serial,serial2,mbridge,wifi"` |
| [setup.h](file:///Users/johnpoltrack/Documents/mLRS/mLRS/Common/setup.h) | Add WiFi destination mask handling |
| [sx_serial_interface_tx.h](file:///Users/johnpoltrack/Documents/mLRS/mLRS/CommonTx/sx_serial_interface_tx.h) | Add 4th `_wifiport` parameter + WiFi case |
| [mavlink_interface_tx.h](file:///Users/johnpoltrack/Documents/mLRS/mLRS/CommonTx/mavlink_interface_tx.h) | Add 4th `_wifiport` parameter + WiFi case (with router support) |
| [msp_interface_tx.h](file:///Users/johnpoltrack/Documents/mLRS/mLRS/CommonTx/msp_interface_tx.h) | Add 3rd `_wifiport` parameter + WiFi case |
| [mlrs-tx.cpp](file:///Users/johnpoltrack/Documents/mLRS/mLRS/CommonTx/mlrs-tx.cpp) | Extern `wifi`, set `wifi_requested` flag, pass `&wifi` to Init() calls |
| [rp-glue.h](file:///Users/johnpoltrack/Documents/mLRS/mLRS/Common/hal/rp-glue.h) | Include `rp-wifi.h`, add `wifi_loop()` in `loop()`, deferred init pattern |
| [platformio.ini](file:///Users/johnpoltrack/Documents/mLRS/platformio.ini) | Add Pico W environment |

---

## Class Design

### tTxWifiNative Class

```cpp
class tTxWifiNative : public tSerialBase
{
  public:
    // initialization (called from loop() on Core 0 via deferred init)
    void Init(tTxSetup* const tx_setup, tCommonSetup* const common_setup);
    
    // periodic processing (called from loop() on Core 0)
    void Do(void);
    
    // tSerialBase interface (called from Core 1 via sx_serial)
    bool available(void) override;
    char getc(void) override;
    void putc(char c) override;
    void putbuf(uint8_t* const buf, uint16_t len) override;
    void flush(void) override;
    
    // status
    bool IsConnected(void) { return client_connected; }
    
  private:
    // protocol handlers
    void init_soft_ap(void);
    void init_station(void);
    void init_udp(void);
    void init_tcp(void);
    
    // cross-core FIFOs
    tCrossCoreFifo<char, 4096> tx_fifo;  // Core 1 → Core 0 (4KB for ~400ms tolerance at 10KB/s)
    tCrossCoreFifo<char, 4096> rx_fifo;  // Core 0 → Core 1
    
    // arduino WiFi objects
    WiFiUDP udp;
    WiFiServer* tcp_server;
    WiFiClient tcp_client;
    
    // connection tracking
    bool client_connected;
    uint32_t client_last_seen_ms;
    
    // configuration
    uint8_t wifi_protocol;
    uint8_t wifi_channel;
    uint8_t wifi_power;
    char ssid[33];
};
```

### Cross-Core FIFO Template

SIZE must be a power of 2 so that modulo can be replaced with bitmask (`& MASK`). This avoids expensive division on Cortex-M0+ (RP2040) where there's no hardware divider for the modulo path. A `static_assert` enforces this at compile time.

```cpp
// lockless single-producer single-consumer FIFO
// safe for one core writing, one core reading
// SIZE must be a power of 2
template<typename T, uint16_t SIZE>
class tCrossCoreFifo
{
    static_assert((SIZE & (SIZE - 1)) == 0, "SIZE must be a power of 2");
    static constexpr uint16_t MASK = SIZE - 1;

  public:
    void Init(void) {
        head = 0;
        tail = 0;
    }

    // writer side (Core 1 for tx_fifo)
    bool Put(T c) {
        uint16_t next = (head + 1) & MASK;
        if (next == tail) return false;  // full
        buf[head] = c;
        __dmb();  // ARM data memory barrier
        head = next;
        return true;
    }

    // bulk copy with single memory barrier
    uint16_t PutBuf(const T* data, uint16_t len) {
        uint16_t h = head;
        uint16_t t = tail;
        uint16_t free = (t > h) ? (t - h - 1) : (SIZE - h + t - 1);
        if (len > free) len = free;
        if (len == 0) return 0;

        // copy in up to two chunks (wrap-around)
        // when h + len <= SIZE, all data fits in one contiguous chunk
        // when h + len > SIZE, first chunk fills to end of buffer, second wraps to start
        uint16_t to_end = SIZE - h;
        uint16_t first = (len <= to_end) ? len : to_end;
        memcpy(&buf[h], data, first * sizeof(T));
        if (first < len) {
            memcpy(&buf[0], data + first, (len - first) * sizeof(T));
        }
        __dmb();  // single barrier after all writes
        head = (h + len) & MASK;
        return len;
    }

    bool HasSpace(uint16_t count) {
        uint16_t h = head;
        uint16_t t = tail;
        uint16_t used = (h - t) & MASK;
        return (SIZE - 1 - used) >= count;
    }

    // reader side (Core 0 for tx_fifo)
    bool Available(void) {
        return head != tail;
    }

    uint16_t AvailableCount(void) {
        return (head - tail) & MASK;
    }

    T Get(void) {
        if (head == tail) return 0;
        T c = buf[tail];
        __dmb();
        tail = (tail + 1) & MASK;
        return c;
    }

    // bulk read with single memory barrier
    uint16_t GetBuf(T* data, uint16_t len) {
        uint16_t h = head;
        uint16_t t = tail;
        uint16_t avail = (h - t) & MASK;
        if (len > avail) len = avail;
        if (len == 0) return 0;

        // read in up to two chunks (wrap-around)
        uint16_t to_end = SIZE - t;
        uint16_t first = (len <= to_end) ? len : to_end;
        memcpy(data, &buf[t], first * sizeof(T));
        if (first < len) {
            memcpy(data + first, &buf[0], (len - first) * sizeof(T));
        }
        __dmb();  // single barrier after all reads
        tail = (t + len) & MASK;
        return len;
    }

    void Flush(void) {
        tail = head;
    }

  private:
    volatile uint16_t head;
    volatile uint16_t tail;
    T buf[SIZE];
};
```

> [!NOTE]
> **Why power-of-2?** On Cortex-M0+ (RP2040), there is no hardware integer divider. The `% SIZE` operation compiles to a software division (~20 cycles). With power-of-2 sizes, `& MASK` is a single cycle AND instruction. This matters on the cross-core hot path.
>
> **Bulk transfer correctness**: The `to_end` calculation in `PutBuf`/`GetBuf` handles the edge case where the index is at `SIZE - 1` and `len = 1`: `first = min(1, 1) = 1`, one element processed at `buf[SIZE-1]`, index advances to `(SIZE-1+1) & MASK = 0`. The wrap-around memcpy path only executes when data genuinely spans the buffer boundary.

---

## HAL Configuration

### Platform Support

| Board | MCU | WiFi Chip | PIO Blocks | Notes |
|-------|-----|-----------|------------|-------|
| Pico W | RP2040 | CYW43439 | 2 (PIO0, PIO1) | mLRS on PIO0, WiFi on PIO1 |
| Pico 2 W | RP2350 | CYW43439 | 3 (PIO0, PIO1, PIO2) | Extra PIO2 available |

### PIO Allocation Strategy

CYW43439 defaults to PIO1 for SPI communication. The mLRS codebase has been updated to use PIO0 for all RP peripherals:

- `jr_pin5_interface_rp.h` - JR Pin5 half-duplex UART uses PIO0
- `rp-uart-template.h` - PIO serial (SERIALPIO1/SERIALPIO2) uses PIO0
- `rp-hal-led-rgb.h` - NeoPixel RGB LED uses PIO0

**Final allocation**:
- **PIO0**: mLRS peripherals (JR Pin5, PIO UARTs, RGB LEDs)
- **PIO1**: CYW43439 WiFi SPI (uses SDK default)
- **PIO2**: Available on RP2350 only (future expansion)

### New HAL File

```
Common/hal/rp/tx-hal-pico-2-w-2400.h    # Pico 2 W (RP2350)
```

Inherits pin definitions from the existing generic HAL but adds:

```cpp
#define DEVICE_HAS_WIFI_NATIVE

// wifi configuration
#define WIFI_AP_PORT              14550   // MAVLink standard UDP port
#define WIFI_AP_MAX_CLIENTS       4
#define WIFI_CLIENT_TIMEOUT_MS    5000

// SSID format: "mLRS AP <METHOD> <MCU_SERIAL>"
// METHOD = UDP, TCP, BT, BLE (derived from WifiProtocol)
// MCU_SERIAL = unique chip serial number (last 4 hex digits)
// example: "mLRS AP UDP A1B2"
#define WIFI_AP_SSID_PREFIX       "mLRS AP"

// station mode credentials (UDPSTA)
// filled in by user via CLI or setup parameters
#define WIFI_STA_SSID             ""        // target network SSID
#define WIFI_STA_PASSWORD         ""        // target network password
```

> [!NOTE]
> **DHCP**: `WiFi.softAP()` automatically starts a lightweight DHCP server to assign IP addresses to connected clients (GCS). No additional configuration is needed.
>
> **SSID generation**: The SSID includes the protocol method and MCU serial number to make each AP uniquely identifiable when multiple mLRS units are in range. The serial is read via the existing `mcu_serial_number()` in `rp-mcu.h` (last 2 bytes formatted as 4 hex digits).

### platformio.ini Environment

```ini
[env:tx-pico-2-w-2400]
extends = env_common_tx_rp
board = rpipico2w
build_src_filter =
  ${env_common_tx_rp.build_src_filter}
  +<modules/sx12xx-lib/src/sx128x.cpp>
  +<Common/thirdparty/gdisp_sharp.cpp>
build_flags =
  ${env_common_rp.build_flags}
  -D ARDUINO_ARCH_RP2350
  -D TX_PICO_2_W_2400
```

---

## Setup Parameter Changes

### Enum Extension

Adding `SERIAL_DESTINATION_WIFI` at position 3 (after `MBRDIGE`) is safe:

- The `L0329_TX_SERIAL_DESTINATION_ENUM` (legacy enum with different ordering) is defined in `setup_types.h` but **never referenced** anywhere in the codebase — no migration code reads it. It's vestigial.
- The `SANITIZE` macro at `setup.h:416` clamps stored values to `< SERIAL_DESTINATION_NUM`. Since `NUM` moves from 3 to 4, a stored value of 3 (previously out-of-range, clamped to default) would now resolve to `WIFI`. However, the `TST_NOTALLOWED` check on the next line will force it to `SERIAL` on devices that don't have `DEVICE_HAS_WIFI_NATIVE`, so this is safe.

```cpp
// setup_types.h
typedef enum {
    SERIAL_DESTINATION_SERIAL = 0,
    SERIAL_DESTINATION_SERIAL2,
    SERIAL_DESTINATION_MBRDIGE,
    SERIAL_DESTINATION_WIFI,      // NEW — position 3, bit 3 in allowed_mask
    SERIAL_DESTINATION_NUM,
} TX_SERIAL_DESTINATION_ENUM;
```

### Option String

The option string is positional — position must match enum value. `"wifi"` at position 3 matches `SERIAL_DESTINATION_WIFI = 3`:

```cpp
// setup_list.h line 102
X( Setup.Tx[0].SerialDestination, LIST, "Tx Ser Dest", "TX_SER_DEST",
   0,0,0,"", "serial,serial2,mbridge,wifi", SETUP_MSK_TX_SER_DEST )
```

### Mask Handling

```cpp
// setup.h - in setup_configure_metadata()
#ifdef DEVICE_HAS_WIFI_NATIVE
    SetupMetaData.Tx_SerialDestination_allowed_mask |= (1 << SERIAL_DESTINATION_WIFI);
#endif
```

### WiFi Parameters Visibility

The existing WiFi parameters should be shown when native WiFi is selected:

```cpp
// setup_list.h - make SETUP_PARAMETER_LIST_TX_ESP available on native WiFi
#if defined USE_ESP_WIFI_BRIDGE_RST_GPIO0 && defined DEVICE_HAS_ESP_WIFI_BRIDGE_CONFIGURE
  #define SETUP_PARAMETER_LIST_TX  SETUP_PARAMETER_LIST_TX_MAIN  SETUP_PARAMETER_LIST_TX_ESP
#elif defined DEVICE_HAS_WIFI_NATIVE
  #define SETUP_PARAMETER_LIST_TX  SETUP_PARAMETER_LIST_TX_MAIN  SETUP_PARAMETER_LIST_TX_ESP
#else
  #define SETUP_PARAMETER_LIST_TX  SETUP_PARAMETER_LIST_TX_MAIN
#endif
```

---

## Serial Interface Integration

Three interfaces switch on `SerialDestination` and all three need a `SERIAL_DESTINATION_WIFI` case. Each has different semantics:

### sx_serial_interface_tx.h

Straightforward — maps destination to a `tSerialBase*` port:

```cpp
void tTxSxSerial::Init(tSerialBase* const _serialport,
                       tSerialBase* const _mbridge,
                       tSerialBase* const _serial2port,
                       tSerialBase* const _wifiport)  // NEW parameter
{
    tSerialBase::Init();

    switch (Setup.Tx[Config.ConfigId].SerialDestination) {
    case SERIAL_DESTINATION_SERIAL:
        ser = _serialport;
        break;
    case SERIAL_DESTINATION_SERIAL2:
        ser = _serial2port;
        break;
    case SERIAL_DESTINATION_MBRDIGE:
        ser = _mbridge;
        break;
    case SERIAL_DESTINATION_WIFI:           // NEW
        ser = _wifiport;
        break;
    default:
        while(1){} // must not happen
    }
    if (!ser) while(1){} // must not happen
}
```

### mavlink_interface_tx.h

More complex — MAVLink has a **router** mode (`ser2 != nullptr`) used when ChannelsSource is mBridge. WiFi behaves like serial/serial2: it's a standalone telemetry port, and if mBridge is the channel source, MAVLink routes between WiFi (`ser`) and mBridge (`ser2`):

```cpp
void tTxMavlink::Init(tSerialBase* const _serialport,
                      tSerialBase* const _mbridge,
                      tSerialBase* const _serial2port,
                      tSerialBase* const _wifiport)  // NEW parameter
{
    switch (Setup.Tx[Config.ConfigId].SerialDestination) {
    case SERIAL_DESTINATION_SERIAL:
        ser = _serialport;
        ser2 = (Setup.Tx[Config.ConfigId].ChannelsSource == CHANNEL_SOURCE_MBRIDGE) ? _mbridge : nullptr;
        break;
    case SERIAL_DESTINATION_SERIAL2:
        ser = _serial2port;
        ser2 = (Setup.Tx[Config.ConfigId].ChannelsSource == CHANNEL_SOURCE_MBRIDGE) ? _mbridge : nullptr;
        break;
    case SERIAL_DESTINATION_MBRDIGE:
        ser = _mbridge;
        ser2 = nullptr;
        break;
    case SERIAL_DESTINATION_WIFI:           // NEW — same pattern as serial/serial2
        ser = _wifiport;
        ser2 = (Setup.Tx[Config.ConfigId].ChannelsSource == CHANNEL_SOURCE_MBRIDGE) ? _mbridge : nullptr;
        break;
    default:
        while(1){} // must not happen
    }
    if (!ser) while(1){} // must not happen
    // ...
}
```

### msp_interface_tx.h

MSP doesn't support mBridge, so WiFi just sets `ser`:

```cpp
void tTxMsp::Init(tSerialBase* const _serialport,
                  tSerialBase* const _serial2port,
                  tSerialBase* const _wifiport)  // NEW parameter
{
    switch (Setup.Tx[Config.ConfigId].SerialDestination) {
    case SERIAL_DESTINATION_SERIAL:
        ser = _serialport;
        break;
    case SERIAL_DESTINATION_SERIAL2:
        ser = _serial2port;
        break;
    case SERIAL_DESTINATION_MBRDIGE:
        ser = nullptr;
        break;
    case SERIAL_DESTINATION_WIFI:           // NEW
        ser = _wifiport;
        break;
    default:
        while(1){} // must not happen
    }
    // ...
}
```

### mlrs-tx.cpp Call Sites

All three `Init()` calls gain the `&wifi` parameter:

```cpp
    mavlink.Init(&serial, &mbridge, &serial2, &wifi);
    msp.Init(&serial, &serial2, &wifi);
    sx_serial.Init(&serial, &mbridge, &serial2, &wifi);
```

Where `wifi` is a `tTxWifiNative` instance (or `nullptr` on non-WiFi builds — see mlrs-tx.cpp Integration section for the `#ifdef` pattern).

---

## Glue Layer Integration

### rp-glue.h Updates

WiFi init and polling both run on Core 0. However, `setup()` runs before Core 1's `main_init()` has loaded the Setup parameters from EEPROM. Instead of initializing WiFi in `setup()`, Core 1 sets a flag after config is loaded, and Core 0's `loop()` performs deferred one-time initialization when it sees the flag.

```cpp
#ifdef DEVICE_HAS_WIFI_NATIVE
#include "rp-wifi.h"
#endif

void setup() {}

void loop()
{
#ifdef DEVICE_HAS_WIFI_NATIVE
    wifi_loop();  // handles deferred init + periodic polling
#endif

    delay(1);  // yield for USB/TinyUSB background processing
}
```

> [!IMPORTANT]
> **Core affinity**: WiFi init and polling MUST run on Core 0. The `arduino-pico` core binds the CYW43 driver to the core that calls `WiFi.softAP()`. Core 1 never touches WiFi directly — it only sets the `wifi_requested` flag and writes to the cross-core FIFO.

---

## WiFi Initialization and ESP Bridge Coexistence

### ESP Bridge Disable

When `SERIAL_DESTINATION_WIFI` is selected, the ESP bridge must be held in reset to prevent it from interfering. The existing `esp_enable()` function (`esp.h:48`) already handles this — when `SerialDestination` doesn't match the ESP's serial port, it calls `esp_reset_low()`. No change needed for boards that have both ESP and native WiFi, since `SERIAL_DESTINATION_WIFI` won't match either `SERIAL_DESTINATION_SERIAL` or `SERIAL_DESTINATION_SERIAL2`.

However, for Pico W boards that don't have an ESP bridge at all, `USE_ESP_WIFI_BRIDGE` won't be defined, so `esp_enable()` is a no-op. This is correct — no ESP to disable.

### rp-wifi.h — Initialization Wrapper

A new `rp-wifi.h` header (analogous to `esp.h`) handles WiFi lifecycle. It is included from `rp-glue.h` and runs entirely on **Core 0**. Initialization is deferred until Core 1 signals that config is ready:

```cpp
// rp-wifi.h — included from rp-glue.h, runs on Core 0

#ifndef RP_WIFI_H
#define RP_WIFI_H

#include "../Common/rp-lib/wifi_native.h"

tTxWifiNative wifi;  // global instance, accessed via extern from mlrs-tx.cpp

volatile bool wifi_requested = false;  // set by Core 1 after config loaded
static bool wifi_initialized = false;  // Core 0 local state

// called from loop() on Core 0 — handles both deferred init and polling
void wifi_loop(void)
{
    if (!wifi_initialized && wifi_requested) {
        wifi.Init(&Setup.Tx[Config.ConfigId], &Setup.Common[Config.ConfigId]);
        wifi_initialized = true;
    }

    if (wifi_initialized) {
        wifi.Do();
    }
}

#endif
```

### mlrs-tx.cpp Integration

`mlrs-tx.cpp` runs on Core 1. After loading config and determining that the serial destination is WiFi, it sets the flag for Core 0 and passes the `wifi` object to the serial interfaces:

```cpp
#ifdef DEVICE_HAS_WIFI_NATIVE
extern tTxWifiNative wifi;
extern volatile bool wifi_requested;
#endif

// in main_init() (Core 1):
#ifdef DEVICE_HAS_WIFI_NATIVE
    if (Setup.Tx[Config.ConfigId].SerialDestination == SERIAL_DESTINATION_WIFI) {
        wifi_requested = true;  // signal Core 0 to initialize WiFi
    }
#endif

    mavlink.Init(&serial, &mbridge, &serial2,
#ifdef DEVICE_HAS_WIFI_NATIVE
                 &wifi
#else
                 nullptr
#endif
    );
    // similarly for msp.Init() and sx_serial.Init()
```

> [!NOTE]
> On non-WiFi builds, the `_wifiport` parameter is `nullptr`. The `switch` in each `Init()` will never select `SERIAL_DESTINATION_WIFI` because the mask prevents it, so the `nullptr` is never dereferenced. The existing `if (!ser) while(1){}` safety check remains.

---

## Verification Plan

### Automated Tests

1. **Build Verification**: Ensure all RP targets compile with and without `DEVICE_HAS_WIFI_NATIVE`
2. **PIO Conflict Check**: Verify PIO allocation doesn't exceed available resources on RP2040

### Manual Verification

1. **Soft AP Mode**:
   - Pico W creates AP with expected SSID (derived from bind phrase)
   - GCS (Mission Planner, QGC) can connect to AP
   - UDP packets sent/received on port 14550

2. **Telemetry Flow**:
   - MAVLink heartbeats visible in GCS
   - Parameter read/write functional
   - No data loss during continuous telemetry stream

3. **Radio Timing**:
   - LQ remains stable (~100%) during WiFi activity
   - No frame timing jitter introduced by WiFi polling

4. **Protocol Modes** (Phase 2+):
   - TCP connection establishment and teardown
   - Station mode connection to existing network

---

## Open Questions

1. **Station Mode Credentials**: How should SSID/password be stored for `WIFI_PROTOCOL_UDPSTA`?
   - Option A: Reuse bind phrase fields
   - Option B: Add new setup parameters
   - Option C: Web configuration portal

2. **mDNS/DNS-SD**: Should we advertise MAVLink service for auto-discovery?

3. **Connection Indicator**: How to display WiFi/BT client connection status on display/LEDs?

4. **Bluetooth Pairing**: How to handle BT pairing PIN and bonding persistence for SPP mode?

5. **Simultaneous WiFi+BT**: Can WiFi and BT modes coexist, or should they be mutually exclusive?
   - CYW43439 supports concurrent operation but with potential throughput trade-offs

---

## Implementation Phases

### Phase 1: UDP Soft AP (MVP)
- Add `SERIAL_DESTINATION_WIFI` enum
- Implement `tTxWifiNative` with UDP only
- Cross-core FIFO integration
- Basic parameter reuse (channel, power)

### Phase 2: TCP Soft AP
- Add TCP server mode via `WiFiServer` / `WiFiClient`
- Connection state tracking (small array of `WiFiClient` objects)
- Multiple client handling (`WiFiServer.available()` returns per-client `WiFiClient`)

### Phase 3: Station Mode (UDPSTA)
- STA mode connection logic
- Credential storage mechanism
- Reconnection handling

### Phase 4: Classic Bluetooth (BT)
- BTstack SPP (Serial Port Profile) integration
- Pairing and bonding support
- Connection state management

### Phase 5: Bluetooth Low Energy (BLE)
- BTstack Nordic UART Service (NUS)
- GATT service advertisement
- Connection management

---

*Document created: 2026-02-03*
*Updated: 2026-02-06 — Added serial interface integration details, ESP coexistence, WiFi.h wrapper, FIFO power-of-2 optimization, legacy enum safety analysis, consolidated FIFO API (PutBuf/GetBuf), fixed include paths, replaced raw CYW43/lwIP references with Arduino WiFi abstractions, deferred init pattern, SSID format, UDPSTA credentials*

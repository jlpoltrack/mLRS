# WiFi CYW43 + lwIP Migration — Progress & Blockers

7 Feb 2026

## Goal

Replace the Arduino WiFi/WiFiUDP abstraction with direct CYW43 driver + lwIP
calls for precise control over WiFi channel, country code, and power.

## What Works (Arduino Route)

The original `WiFi.softAP()` + `WiFiUDP` approach works end-to-end:
- AP visible, clients connect, DHCP assigns IPs
- UDP data flows to Mission Planner
- channel parameter accepted by `WiFi.softAP(ssid, nullptr, ch)`

The limitation: channel selection restricted to 1-11 because the Arduino
framework boots CYW43 with `CYW43_COUNTRY_WORLDWIDE`.

## What We Tried (Raw CYW43 + lwIP)

### Approach

Replace `WiFi.softAP()` with direct CYW43 calls:
```cpp
cyw43_wifi_ap_set_channel(&cyw43_state, ch);
cyw43_arch_enable_ap_mode(ssid, nullptr, CYW43_AUTH_OPEN);
```

Replace `WiFiUDP` with raw lwIP:
```cpp
udp_pcb = udp_new();
udp_bind(udp_pcb, IP_ADDR_ANY, WIFI_AP_PORT);
udp_recv(udp_pcb, rx_callback, this);
```

Manual DHCP via `dhcp_server_init()`.

### Results

- **AP appears** — SSID visible, correct name
- **Channel selection works for 1-11** — confirmed channel 6
- **UDP data pipe is dead** — no data flows in either direction

## Blockers

### 1. lwIP netif never gets created (UDP blocker)

The Arduino framework wraps `cyw43_cb_tcpip_init()` to a no-op in
`cyw43_wrappers.cpp:90`. This is the pico-sdk callback that creates the lwIP
netif for the CYW43 interface.

Additionally, `__getCYW43Netif()` is a weak function returning `nullptr`
(`cyw43_wrappers.cpp:49`). The strong override lives in the Arduino WiFi
library's `CYW43shim.cpp:42`, which only gets linked when `#include <WiFi.h>`
pulls in the library.

Without the netif:
- `__wrap_cyw43_cb_process_ethernet()` silently drops all incoming frames
- `udp_sendto()` has no interface to route through
- DHCP server has no interface to bind to

**This is the primary blocker.** To use raw CYW43 + lwIP, we must either:
- create the lwIP netif manually (complex — needs `netif_add()` with the
  correct CYW43 output function, plus `netif_set_up/default/link_up`)
- or bypass the Arduino wrapper system entirely

### 2. Channel 13 not achievable in AP mode

The CYW43439 firmware limits AP channels based on the country code set during
`cyw43_arch_init()`. The Arduino framework calls this at boot with
`CYW43_COUNTRY_WORLDWIDE` (channels 1-11 only).

Attempted fixes:
- `cyw43_wifi_set_up(..., CYW43_COUNTRY_GERMANY)` — country code ignored
  because `cyw43_wifi_on()` only applies it when `itf_state == 0`, and the
  framework already initialized it
- `cyw43_arch_deinit()` + `cyw43_arch_init_with_country(GERMANY)` — the
  deinit tears down Arduino framework state that doesn't come back
- `-D WIFICC=CYW43_COUNTRY_GERMANY` build flag — the framework's designed
  override mechanism (`cyw43_wrappers.cpp:100`), but did not appear to take
  effect (possibly a PlatformIO build system issue with framework flags)
- `-D PICO_CYW43_ARCH_DEFAULT_COUNTRY_CODE=0x4544` — same approach targeting
  pico-sdk directly, also did not take effect

**Conclusion:** channel 13 requires the country code to be set before
`cyw43_arch_init()` runs, which happens during Arduino framework boot. None of
our runtime approaches worked. The build flag approach should work in theory
but may have a PlatformIO compilation scope issue with framework source files.

### 3. Default AP channel is 3

`cyw43_ctrl.c:106`: `self->ap_channel = 3;` — hardcoded in `cyw43_init()`.
This is why the AP defaults to channel 3 if `cyw43_wifi_ap_set_channel()` is
not called, or called after the AP starts.

**Fix confirmed:** calling `cyw43_wifi_ap_set_channel()` BEFORE
`cyw43_arch_enable_ap_mode()` correctly applies the channel (verified for
channels 1, 6, 11).

## Key Source Files

| file | role |
|---|---|
| `cyw43_wrappers.cpp` | Arduino framework CYW43 integration, wraps tcpip callbacks to no-ops |
| `CYW43shim.cpp` | strong override for `__getCYW43Netif()`, provides netif to ethernet callback |
| `cyw43_ctrl.c` | CYW43 driver, AP init, channel defaults |
| `cyw43_arch.c` | pico-sdk CYW43 arch, `enable_ap_mode()`, country code static variable |
| `WiFiClass.h:134` | `softAP()` — takes channel parameter directly |

## Decision Required

### Option A: Full Arduino Route

Use `WiFi.softAP()` for AP + DHCP + netif, `WiFiUDP` for data.
- **pro:** works today, well-tested, simple
- **con:** channel limited to 1-11, no fine-grained power control
- **con:** no direct access to CYW43 driver state

### Option B: Full CYW43 + lwIP

Replace all Arduino WiFi abstractions with raw pico-sdk + lwIP calls.
- **pro:** full control over channel, country, power
- **pro:** no Arduino WiFi library dependency
- **con:** must solve the netif creation problem (significant effort)
- **con:** must handle DHCP server, IP configuration, ethernet bridging
- **con:** channel 13 still requires build flag or framework modification

### Recommended Next Step

If going option B, the key work item is creating the lwIP netif manually.
This requires studying how `pico_lwip_cyw43` creates netifs in the stock
pico-sdk (non-Arduino) path. The relevant code is in:
- `pico-sdk/src/rp2_common/pico_cyw43_driver/`
- `pico-sdk/lib/lwip/` netif sources

The Arduino wrapper interception (`__wrap_cyw43_cb_tcpip_init`) would need to
be either bypassed or supplemented with our own netif initialization.

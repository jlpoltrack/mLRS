//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP Delay utilities
//*******************************************************
#ifndef RP_DELAY_H
#define RP_DELAY_H

#include <pico/time.h>
#include <pico/platform.h>

inline void delay_init(void) {}

// used by the sx drivers for NSS setup/hold times (25..100 ns), which the call
// overhead alone likely covers, but guarantee the lower bound to be safe
inline void delay_ns(uint32_t ns) { busy_wait_at_least_cycles((ns * (F_CPU / 1000000) + 999) / 1000); }

inline void delay_us(uint32_t us) { busy_wait_us_32(us); }
inline void delay_ms(uint32_t ms) { busy_wait_ms(ms); }

#endif // RP_DELAY_H

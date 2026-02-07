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

inline void delay_init(void) {}

inline void delay_ns(uint32_t ns) {
    uint32_t f_mhz = F_CPU / 1000000;  // get system clock in MHz
    uint32_t cycles = (ns * f_mhz * 69) >> 16; // calculate required cycles: cycles = ns * f_mhz / 1000, approximate 1/1000 with 69/65536
    if (cycles <= 10) return;  // compensate for overhead (approx 10 cycles)
    busy_wait_at_least_cycles(cycles - 10);
}

// use busy-wait functions - works with interrupts disabled
inline void delay_us(uint32_t us) { busy_wait_us_32(us); }
inline void delay_ms(uint32_t ms) { busy_wait_ms(ms); }

#endif // RP_DELAY_H

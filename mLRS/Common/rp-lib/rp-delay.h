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
inline void delay_ns(uint32_t ns) {}
inline void delay_us(uint32_t us) { busy_wait_us_32(us); }
inline void delay_ms(uint32_t ms) { busy_wait_ms(ms); }

#endif // RP_DELAY_H

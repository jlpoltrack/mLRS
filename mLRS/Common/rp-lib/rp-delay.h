//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP2040 Delay utilities
//*******************************************************
#ifndef RP_DELAY_H
#define RP_DELAY_H

#include <Arduino.h>

inline void delay_init(void) { } // stub, Arduino already initialized
inline void delay_us(uint32_t us) { delayMicroseconds(us); }
inline void delay_ms(uint32_t ms) { delay(ms); }

#endif // RP_DELAY_H

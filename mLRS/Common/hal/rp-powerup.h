//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP2040 Powerup utilities
//*******************************************************
#ifndef RP_POWERUP_H
#define RP_POWERUP_H

#include <Arduino.h>

// Just a simple check for a bind button on startup
inline uint8_t hal_get_powerup_counter(void) {
    // If we want a persistent counter, we'd use LittleFS here.
    // For now, return 0 as most RXs use a button for bind or have no counter requirements.
    return 0;
}

#endif // RP_POWERUP_H

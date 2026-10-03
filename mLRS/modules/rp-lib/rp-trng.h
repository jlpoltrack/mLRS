//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP TRNG
//*******************************************************
#ifndef RP_TRNG_H
#define RP_TRNG_H

#include <pico/rand.h>

// On RP2350 pico_rand seeds from the hardware TRNG, on RP2040 from ROSC samples,
// board id, and timer, since the RP2040 has no TRNG peripheral.

inline void trng_init(void)
{
    get_rand_32(); // call it once, seeding happens lazily and can take a moment
}

inline uint32_t trng_get32(void)
{
    return get_rand_32();
}

#endif // RP_TRNG_H

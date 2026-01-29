//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP2040 RX Clock - header-only implementation
// 2026-01-28
//*******************************************************
#ifndef RP_RXCLOCK_H
#define RP_RXCLOCK_H

#include <Arduino.h>
#include <hardware/timer.h>
#include <hardware/sync.h>
#include "rp-timer.h"

//-------------------------------------------------------
// Clock constants and variables
//-------------------------------------------------------

#define CLOCK_SHIFT_10US          100 // 1 ms
#define CLOCK_CNT_1MS             100 // 10us interval * 100 = 1000us

static spin_lock_t* rp_clock_spinlock = nullptr;
static struct repeating_timer rp_clock_timer;

static volatile uint32_t CNT_10us = 0;
static volatile uint32_t CCR1 = 0xFFFF;
static volatile uint32_t CCR3 = 0xFFFF;
static volatile uint32_t MS_C = CLOCK_CNT_1MS;
static volatile bool doPostReceive = false;

static uint16_t CLOCK_PERIOD_10US = 2000; // default 20ms

//-------------------------------------------------------
// Clock ISR callback
//-------------------------------------------------------

static bool __not_in_flash_func(rp_clock_10us_callback)(struct repeating_timer *t) {
    (void)t;
    
    uint32_t save = spin_lock_blocking(rp_clock_spinlock);
    
    CNT_10us++;
    
    // call HAL_IncTick every 1 ms
    if (CNT_10us >= MS_C) {
        MS_C = CNT_10us + CLOCK_CNT_1MS;
        HAL_IncTick();
    }
    
    // this is at about when RX was or was supposed to be received
    if (CNT_10us >= CCR1) {
        CCR3 = CNT_10us + CLOCK_SHIFT_10US; // next doPostReceive
        CCR1 = CNT_10us + CLOCK_PERIOD_10US; // next tick
    }
    
    // this is 1 ms after RX was or was supposed to be received
    if (CNT_10us >= CCR3) {
        doPostReceive = true;
    }
    
    spin_unlock(rp_clock_spinlock, save);
    return true;
}

//-------------------------------------------------------
// RxClock Class
//-------------------------------------------------------

class tRxClock {
public:
    void Init(uint16_t period_ms);
    void SetPeriod(uint16_t period_ms);
    void Reset(void);
    
private:
    bool initialized = false;
};

inline void tRxClock::Init(uint16_t period_ms) {
    CLOCK_PERIOD_10US = period_ms * 100;
    doPostReceive = false;
    
    CNT_10us = 0;
    CCR1 = CLOCK_PERIOD_10US;
    CCR3 = CLOCK_SHIFT_10US;
    MS_C = CLOCK_CNT_1MS;
    
    if (initialized) return;
    
    // init spinlock
    rp_clock_spinlock = spin_lock_init(spin_lock_claim_unused(true));
    
    // start 10us repeating timer (negative value = repeat every N us)
    add_repeating_timer_us(-10, rp_clock_10us_callback, NULL, &rp_clock_timer);
    
    initialized = true;
}

inline void tRxClock::SetPeriod(uint16_t period_ms) {
    uint32_t save = spin_lock_blocking(rp_clock_spinlock);
    CLOCK_PERIOD_10US = period_ms * 100;
    spin_unlock(rp_clock_spinlock, save);
}

inline void tRxClock::Reset(void) {
    if (!CLOCK_PERIOD_10US) while(1) {}
    
    uint32_t save = spin_lock_blocking(rp_clock_spinlock);
    CCR1 = CNT_10us + CLOCK_PERIOD_10US;
    CCR3 = CNT_10us + CLOCK_SHIFT_10US;
    MS_C = CNT_10us + CLOCK_CNT_1MS;
    spin_unlock(rp_clock_spinlock, save);
}

#endif // RP_RXCLOCK_H

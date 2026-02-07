//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP RxClock
//*******************************************************
#ifndef RP_RXCLOCK_H
#define RP_RXCLOCK_H
#pragma once

#include <pico/time.h>


#define CLOCK_SHIFT_10US          100 // 1 ms in 10us units

static volatile alarm_id_t alarm_id;

static volatile uint32_t CLOCK_PERIOD_US;
static volatile uint32_t CLOCK_SHIFT_US;

static volatile bool doPostReceive; 
static volatile uint64_t next_event_time_us;


//-------------------------------------------------------
// Callbacks
//-------------------------------------------------------

int64_t __not_in_flash_func(rp_rx_event_callback)(alarm_id_t id, void *user_data) {
    doPostReceive = true;
    next_event_time_us += CLOCK_PERIOD_US;
    alarm_id = add_alarm_at(from_us_since_boot(next_event_time_us), rp_rx_event_callback, NULL, true);
    
    return 0;
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

void tRxClock::Init(uint16_t period_ms) {
    alarm_id = -1;
    CLOCK_PERIOD_US = period_ms * 1000;
    CLOCK_SHIFT_US = CLOCK_SHIFT_10US * 10;
    doPostReceive = false;
    next_event_time_us = 0;
    
    if (initialized) return;

    Reset();
    initialized = true;
}

void tRxClock::SetPeriod(uint16_t period_ms) {
    CLOCK_PERIOD_US = period_ms * 1000;
}

void tRxClock::Reset(void) {
    if (alarm_id >= 0) { cancel_alarm(alarm_id); } // cancel existing alarm before scheduling new one

    next_event_time_us = time_us_64() + CLOCK_SHIFT_US;
    alarm_id = add_alarm_at(from_us_since_boot(next_event_time_us), rp_rx_event_callback, NULL, true);
}


#endif // RP_RXCLOCK_H

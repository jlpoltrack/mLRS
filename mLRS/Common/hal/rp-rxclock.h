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


volatile bool doPostReceive;


//-------------------------------------------------------
// RxClock Class
//-------------------------------------------------------

class tRxClock
{
  public:
    void Init(uint16_t period_ms);
    void SetPeriod(uint16_t period_ms);
    void Reset(void);

  private:
    volatile alarm_id_t alarm_id;
    volatile uint32_t clock_period_us;
    volatile uint32_t clock_shift_us;
    volatile uint64_t next_event_time_us;

    static int64_t alarm_callback(alarm_id_t id, void* user_data);
};


int64_t __not_in_flash_func(tRxClock::alarm_callback)(alarm_id_t id, void* user_data)
{
    tRxClock* clock = static_cast<tRxClock*>(user_data);
    doPostReceive = true;
    clock->next_event_time_us += clock->clock_period_us;
    clock->alarm_id = add_alarm_at(from_us_since_boot(clock->next_event_time_us), alarm_callback, user_data, true);
    return 0;
}


void tRxClock::Init(uint16_t period_ms)
{
    alarm_id = -1;
    clock_period_us = period_ms * 1000;
    clock_shift_us = 1000; // 1 ms
    doPostReceive = false;
    next_event_time_us = 0;

    Reset();
}


void tRxClock::SetPeriod(uint16_t period_ms)
{
    clock_period_us = period_ms * 1000;
}


void __not_in_flash_func(tRxClock::Reset)(void)
{
    if (alarm_id >= 0) { cancel_alarm(alarm_id); } // cancel existing alarm before scheduling new one

    next_event_time_us = time_us_64() + clock_shift_us;
    alarm_id = add_alarm_at(from_us_since_boot(next_event_time_us), alarm_callback, this, true);
}


#endif // RP_RXCLOCK_H

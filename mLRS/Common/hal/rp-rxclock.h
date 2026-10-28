//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP RxClock
//*******************************************************
// One hardware timer alarm of its own, like the CCR on STM32: the irq sets doPostReceive and arms
// the next period, Reset() disarms and re-arms. No alarm pool: no ids to track, no heap, no lock
// shared with core 1, and a cancel always hits the one alarm there is.
//*******************************************************
#ifndef RP_RXCLOCK_H
#define RP_RXCLOCK_H
#pragma once

#include <pico/time.h>
#include <hardware/timer.h>
#include <hardware/irq.h>


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
    int alarm_num = -1; // hardware alarm, claimed once
    volatile uint32_t clock_period_us;
    volatile uint32_t clock_shift_us;
    volatile uint64_t next_event_time_us;

    static void alarm_callback(uint alarm_num);
};


static tRxClock* rxclock_ptr; // the hardware alarm callback carries no user data


void tRxClock::alarm_callback(uint alarm_num)
{
    tRxClock* clock = rxclock_ptr;
    doPostReceive = true;
    clock->next_event_time_us += clock->clock_period_us;
    if (hardware_alarm_set_target(alarm_num, from_us_since_boot(clock->next_event_time_us))) {
        // target already passed, must not happen, resync from now instead of firing a burst
        clock->next_event_time_us = time_us_64() + clock->clock_period_us;
        hardware_alarm_set_target(alarm_num, from_us_since_boot(clock->next_event_time_us));
    }
}


void tRxClock::Init(uint16_t period_ms)
{
    clock_period_us = period_ms * 1000;
    clock_shift_us = 1000; // 1 ms
    doPostReceive = false;
    next_event_time_us = 0;

    if (alarm_num < 0) { // claim once, the irq is registered on the calling core, which runs the main loop
        rxclock_ptr = this;
        alarm_num = hardware_alarm_claim_unused(true);
        hardware_alarm_set_callback(alarm_num, alarm_callback);
        irq_set_priority(TIMER_ALARM_IRQ_NUM(PICO_DEFAULT_TIMER_INSTANCE(), alarm_num), PICO_HIGHEST_IRQ_PRIORITY);
    }

    Reset();
}


void tRxClock::SetPeriod(uint16_t period_ms)
{
    clock_period_us = period_ms * 1000;
}


void tRxClock::Reset(void)
{
    // the callback runs on this core, so it either completed before the cancel, and the cancel
    // disarms what it re-armed, or it does not run at all until the new target is set
    hardware_alarm_cancel(alarm_num);
    next_event_time_us = time_us_64() + clock_shift_us;
    hardware_alarm_set_target(alarm_num, from_us_since_boot(next_event_time_us));
}


#endif // RP_RXCLOCK_H

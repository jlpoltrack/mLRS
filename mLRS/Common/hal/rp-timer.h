//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP Timer
//*******************************************************
#ifndef RP_TIMER_H
#define RP_TIMER_H

#include <pico/time.h>

//-------------------------------------------------------
// SysTask & millis32() functions
//-------------------------------------------------------

#define SYSTICK_DELAY_MS(x)       (uint16_t)(((uint32_t)(x)*(uint32_t)1000)/SYSTICK_TIMESTEP)

static uint32_t doSysTask_done = 0; // never changed in ISR; incremented when (uwTick != doSysTask_done)
static volatile uint32_t uwTick = 0; // only changed in ISR; incremented to signal tick

void resetSysTask(void) { doSysTask_done = uwTick; }

bool doSysTask(void)
{
    if (uwTick != doSysTask_done) {
        doSysTask_done++;
        return true;
    }
    return false;
}

inline void HAL_IncTick(void) { uwTick++; }

volatile uint32_t millis32(void) { return uwTick; }

//-------------------------------------------------------
// Micros functions
//-------------------------------------------------------

void micros_init(void) {}

uint16_t micros16(void) { return (uint16_t)micros(); }

uint64_t micros64(void) { return to_us_since_boot(get_absolute_time()); }

//-------------------------------------------------------
// Callback
//-------------------------------------------------------

static struct repeating_timer rp_systick_timer;

bool __not_in_flash_func(rp_systick_callback)(struct repeating_timer *t)
{
    HAL_IncTick();
    return true;
}

//-------------------------------------------------------
// Init function
//-------------------------------------------------------

void systick_millis_init(void)
{
    static bool initialized = false;
    if (initialized) return;
    add_repeating_timer_ms(-1, rp_systick_callback, NULL, &rp_systick_timer);
    initialized = true;
}

void timer_init(void)
{
    uwTick = 0;
    systick_millis_init();
    micros_init();
    resetSysTask();
}


//-------------------------------------------------------
// Tx Clock Class
// Uses Pico SDK's alarm pool for precise timing
//-------------------------------------------------------

#ifdef DEVICE_IS_TRANSMITTER

class tTxClock
{
  public:
    void Init(void);

    void SetCC1Callback(void (*callback)(void));
    bool HasCC1Callback(void) { return (cc1_callback_ptr != nullptr); }
    void StartCC1Delay(uint16_t delay_us);

  private:
    void (*cc1_callback_ptr)(void);
    alarm_id_t alarm_id;
    static int64_t alarm_callback(alarm_id_t id, void* user_data);
};


void tTxClock::Init(void)
{
    cc1_callback_ptr = nullptr;
    alarm_id = -1;
}


void tTxClock::SetCC1Callback(void (*callback)(void)) { cc1_callback_ptr = callback; }


// one-shot alarm for cc1 delay
void tTxClock::StartCC1Delay(uint16_t delay_us)
{
    if (cc1_callback_ptr == nullptr) return;

    if (alarm_id >= 0) { cancel_alarm(alarm_id); } // cancel any pending alarm

    alarm_id = add_alarm_in_us(delay_us, alarm_callback, this, true); // schedule one-shot alarm
}


int64_t __not_in_flash_func(tTxClock::alarm_callback)(alarm_id_t id, void* user_data)
{
    tTxClock* clock = static_cast<tTxClock*>(user_data);
    clock->alarm_id = -1;
    if (clock->cc1_callback_ptr != nullptr) {
        clock->cc1_callback_ptr();
    }
    return 0; // don't reschedule (one-shot)
}


tTxClock txclock;

#endif // DEVICE_IS_TRANSMITTER


#endif // RP_TIMER_H

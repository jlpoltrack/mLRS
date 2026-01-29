//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP2040 Timer utilities
// 2026-01-28
//*******************************************************
#ifndef RP_TIMER_H
#define RP_TIMER_H

#include <Arduino.h>

//-------------------------------------------------------
// SysTask & millis32() functions
//-------------------------------------------------------

#define SYSTICK_DELAY_MS(x)       (uint16_t)(((uint32_t)(x)*(uint32_t)1000)/SYSTICK_TIMESTEP)

extern volatile uint32_t uwTick;

// two variables allow to avoid race with HAL_IncTick() when tasks may take longer than one tick
uint32_t doSysTask_done = 0; // never changed in ISR; incremented when (uwTick != doSysTask_done)

void resetSysTask(void)
{
    doSysTask_done = uwTick;
}

volatile bool doSysTask(void)
{
    if (uwTick != doSysTask_done) {
        doSysTask_done++;
        return true;
    }
    return false;
}

inline void HAL_IncTick(void)
{
    uwTick++;
}

inline uint32_t HAL_GetTick(void)
{
    return millis();
}

volatile uint32_t millis32(void)
{
    return uwTick;
}

//-------------------------------------------------------
// Micros functions
//-------------------------------------------------------
// free running timer with 1us time base

void micros_init(void)
{
    // stub on RP2040, handled by Arduino init()
}

uint16_t micros16(void)
{
    return (uint16_t)micros();
}

//-------------------------------------------------------
// Init function
//-------------------------------------------------------

void systick_millis_init(void)
{
    // on RP2040 we use a repeating timer alarm
    // for simplicity, we use Arduino's built-in millis() and increment uwTick from loop
}

void timer_init(void)
{
    uwTick = 0;
    systick_millis_init();
    micros_init();
    resetSysTask();
}

#endif // RP_TIMER_H

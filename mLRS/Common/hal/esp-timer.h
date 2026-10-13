//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP Timer
//********************************************************
#ifndef ESP_TIMER_H
#define ESP_TIMER_H
#pragma once


//-------------------------------------------------------
// SysTask & millis32()  functions
//-------------------------------------------------------

#define SYSTICK_DELAY_MS(x)       (uint16_t)(((uint32_t)(x)*(uint32_t)1000)/SYSTICK_TIMESTEP)


// Two variables allow to avoid race with HAL_IncTick() when tasks may take longer than one tick
uint32_t doSysTask_done = 0; // Never changed in ISR; incremented when (uwTick != doSysTask_done)
volatile uint32_t uwTick = 0; // Only changed in ISR; incremented to signal tick

void resetSysTask()
{
    doSysTask_done = uwTick;
}

volatile bool doSysTask()
{
    if (uwTick != doSysTask_done) {
        doSysTask_done++;
        return true;
    }
    return false;
}

IRAM_ATTR void HAL_IncTick(void)
{
    uwTick++;
}


volatile uint32_t millis32(void)
{
    return uwTick;
}


#ifdef DEVICE_IS_TRANSMITTER
// for receiver it is done in esp-rxclock.h, for transmitter it is here
// transmitters are ESP32 only

static bool systick_millis_initialized = false;

IRQHANDLER(
void CLOCK_IRQHandler(void)
{
    HAL_IncTick();
})


void systick_millis_init(void)
{
    if (systick_millis_initialized) return;

    hw_timer_t* timer0_cfg = nullptr;
    timer0_cfg = timerBegin(0, 80, 1);  // Timer 0, APB clock is 80 Mhz | divide by 80 is 1 MHz / 1 us, count up
    timerAttachInterrupt(timer0_cfg, &CLOCK_IRQHandler, true);
    timerAlarmWrite(timer0_cfg, 1000, true);
    timerAlarmEnable(timer0_cfg);

    systick_millis_initialized = true;
}

#else

void systick_millis_init(void) {}

#endif


//-------------------------------------------------------
// Micros functions
//-------------------------------------------------------
// free running timer with 1us time base

void micros_init(void)
{
    // just a stub on ESP, handled by Arduino init()
}


uint16_t micros16(void)
{
    return (uint16_t)micros();
}


//-------------------------------------------------------
// Init function
//-------------------------------------------------------

void timer_init(void)
{
    uwTick = 0;
    systick_millis_init();
    micros_init();
    resetSysTask();
}


#endif // ESP_TIMER_H

//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Cooling Fan
//********************************************************
#ifndef FAN_H
#define FAN_H
#pragma once


#include <stdlib.h>
#include <ctype.h>
#include "hal/hal.h"


#ifndef USE_FAN

class tFan
{
  public:
    void Init(void) {}

    void SetPower(int8_t power_dbm) {}
    void Tick_ms(void) {}
};

#else

class tFan
{
  public:
    void Init(void);

    void SetPower(int8_t power_dbm);
    void Tick_ms(void);

  private:
    bool initialized;
    int8_t power_dbm_curr;
};


void tFan::Init(void)
{
    initialized = false;
    power_dbm_curr = POWER_MIN;

#ifdef DEVICE_HAS_FAN_ONOFF
    fan_init();
#elif defined DEVICE_HAS_FAN_PWM
    #define FAN_PWM_FREQ  25000  // 25 kHz is standard for cooling fans

    fan_gpio_init();
    fan_tim_clk_enable();

    LL_TIM_SetPrescaler(FAN_TIMx, 0);
    LL_TIM_SetAutoReload(FAN_TIMx, (SystemCoreClock / FAN_PWM_FREQ) - 1);
    LL_TIM_SetCounterMode(FAN_TIMx, LL_TIM_COUNTERMODE_UP);

    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {};
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitStruct.CompareValue = 0;  // start with fan off
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    LL_TIM_OC_Init(FAN_TIMx, FAN_TIM_CHANNEL, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(FAN_TIMx, FAN_TIM_CHANNEL);

    LL_TIM_EnableARRPreload(FAN_TIMx);
    LL_TIM_OC_EnablePreload(FAN_TIMx, FAN_TIM_CHANNEL);

    LL_TIM_CC_EnableChannel(FAN_TIMx, FAN_TIM_CHANNEL);
    LL_TIM_EnableCounter(FAN_TIMx);
#endif
}


void tFan::SetPower(int8_t power_dbm)
{
    if (power_dbm_curr == power_dbm && initialized) return;
    initialized = true;
    power_dbm_curr = power_dbm;

#ifdef DEVICE_HAS_FAN_ONOFF
    fan_set_power(power_dbm);
#elif defined DEVICE_HAS_FAN_PWM
    uint8_t duty_percent;
    
    if (power_dbm >= POWER_30_DBM) { duty_percent = 100; }
    else if (power_dbm >= POWER_27_DBM) { duty_percent = 75; }
    else if (power_dbm >= POWER_24_DBM) { duty_percent = 50; }
    else if (power_dbm >= POWER_20_DBM) { duty_percent = 25; }
    else { duty_percent = 0; }
    
    fan_set_compare(((uint32_t)duty_percent * (SystemCoreClock / FAN_PWM_FREQ)) / 100);
#endif
}


void tFan::Tick_ms(void)
{
#ifdef DEVICE_HAS_FAN_TEMPCONTROLLED_ONOFF
    int16_t temp_dC = fan_tempsensor_read_dC();

    if (temp_dC > 500) { // 50.0 C
        fan_on();
    } else
    if (temp_dC < 400) { // 40.0 C
        fan_off();
    }
#endif
}


#endif // USE_FAN

#endif // FAN_H




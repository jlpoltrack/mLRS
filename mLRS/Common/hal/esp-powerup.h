//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP Powerup Counter
//********************************************************
#ifndef ESP_POWERUP_CNT_H
#define ESP_POWERUP_CNT_H
#pragma once

// Needs more work to implement on ESP8266
// Needed for entering bind mode with rapid power cycles


#include <inttypes.h>


typedef enum {
    POWERUPCNT_TASK_NONE = 0,
    POWERUPCNT_TASK_BIND,
} POWERUPCNT_TASK_ENUM;


#ifdef ESP32
// the count is kept in its own nvs namespace, and not in the emulated eeprom, so that
// the setup data is not rewritten on each power up.

#include <Preferences.h>


extern volatile uint32_t millis32(void);


#define POWERUPCNT_NVS_NAMESPACE  "mlrs-powerup" // max 15 chars
#define POWERUPCNT_NVS_KEY        "cnt"

#define POWERUPCNT_BIND_CNT       4 // number of rapid power ups to enter bind mode

#define POWERUPCNT_TMO_MS         2000


class tPowerupCounter
{
  public:
    void Init(void);
    void Do(void);
    uint8_t Task(void);

  private:
    bool powerup_do;
    uint8_t task;

    Preferences nvs;
};


void tPowerupCounter::Init(void)
{
    powerup_do = false;
    task = POWERUPCNT_TASK_NONE;

    // check if this really was a power up, or just a reset
    if (esp_reset_reason() != ESP_RST_POWERON) return;

    if (!nvs.begin(POWERUPCNT_NVS_NAMESPACE, false)) return;

    uint8_t cnt = nvs.getUChar(POWERUPCNT_NVS_KEY, 0) + 1;
    if (cnt > POWERUPCNT_BIND_CNT) cnt = 1; // should not happen, but exit safely

    if (cnt >= POWERUPCNT_BIND_CNT) {
        task = POWERUPCNT_TASK_BIND;
        nvs.putUChar(POWERUPCNT_NVS_KEY, 0);
        return;
    }

    nvs.putUChar(POWERUPCNT_NVS_KEY, cnt);
    powerup_do = true; // count is cleared again if we stay powered for long enough
}


void tPowerupCounter::Do(void)
{
    if (!powerup_do) return;

    if (millis32() < POWERUPCNT_TMO_MS) return;

    powerup_do = false;

    nvs.putUChar(POWERUPCNT_NVS_KEY, 0);
}


uint8_t tPowerupCounter::Task(void)
{
    switch (task) {
    case POWERUPCNT_TASK_BIND:
        task = POWERUPCNT_TASK_NONE;
        return POWERUPCNT_TASK_BIND;
    }

    return POWERUPCNT_TASK_NONE;
}


#else
//-------------------------------------------------------
// ESP8266: not implemented
//-------------------------------------------------------

class tPowerupCounter
{
  public:
    void Init(void) {}
    void Do(void) {}
    uint8_t Task(void) { return POWERUPCNT_TASK_NONE; }
};

#endif


#endif // ESP_POWERUP_CNT

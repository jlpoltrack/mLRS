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


#include <inttypes.h>


typedef enum {
    POWERUPCNT_TASK_NONE = 0,
    POWERUPCNT_TASK_BIND,
} POWERUPCNT_TASK_ENUM;


// ESP32 only: EEPROM.commit() is NVS-backed with wear leveling.
// On ESP8266 commit() does a raw sector erase, so not implemented.
#ifdef ESP32

#define POWERUPCNT_TMO_MS             2000
#define POWERUPCNT_BIND_COUNT         4


class tPowerupCounter
{
  public:
    void Init(void);
    void Do(void);
    uint8_t Task(void);

  private:
    bool powerup_do;
    uint8_t task;
};


void tPowerupCounter::Init(void)
{
    powerup_do = true;
    task = POWERUPCNT_TASK_NONE;

    uint8_t count = EEPROM.read(POWERUPCNT_EE_ADDRESS);

    if (count > POWERUPCNT_BIND_COUNT) count = 0; // sanitize, e.g. first use is 0xFF

    count++;

    if (count >= POWERUPCNT_BIND_COUNT) {
        task = POWERUPCNT_TASK_BIND;
        powerup_do = false;
        count = 0;
    }

    EEPROM.write(POWERUPCNT_EE_ADDRESS, count);
    EEPROM.commit();
}


void tPowerupCounter::Do(void)
{
    if (!powerup_do) return;

    uint32_t tnow_ms = millis32();
    if (tnow_ms < POWERUPCNT_TMO_MS) return;

    powerup_do = false;

    EEPROM.write(POWERUPCNT_EE_ADDRESS, 0);
    EEPROM.commit();
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

#else // ESP8266

class tPowerupCounter
{
  public:
    void Init(void) {}
    void Do(void) {}
    uint8_t Task(void) { return POWERUPCNT_TASK_NONE; }
};

#endif


#endif // ESP_POWERUP_CNT_H

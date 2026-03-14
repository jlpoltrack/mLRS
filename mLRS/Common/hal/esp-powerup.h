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

// counts rapid power cycles and triggers bind after the threshold is reached.
// uses the Arduino EEPROM library which is available on both ESP32 and ESP8266.
// call Init() early during startup (after ee_init()), call Do() periodically
// so the stored counter can be cleared after the timeout.


// timeout after which an in-progress counter is cleared (ms)
#ifndef POWERUPCNT_TMO_MS
#define POWERUPCNT_TMO_MS           2000
#endif

// number of power cycles required to trigger bind
#ifndef POWERUPCNT_REQUIRED
#define POWERUPCNT_REQUIRED         4
#endif

// eeprom offset for the powerup counter (page 2, after the two setup pages)
#define POWERUPCNT_EE_OFFSET        (EE_PAGE_SIZE * 2)


typedef enum {
    POWERUPCNT_TASK_NONE = 0,
    POWERUPCNT_TASK_BIND,
} POWERUPCNT_TASK_ENUM;


class tPowerupCounter
{
  public:
    void Init(void);
    void Do(void);
    uint8_t Task(void);

  private:
    bool powerup_do;
    uint8_t task;
    uint32_t start_ms;
};


void tPowerupCounter::Init(void)
{
    powerup_do = false;
    task = POWERUPCNT_TASK_NONE;

    uint16_t cnt = 0;
    EEPROM.get(POWERUPCNT_EE_OFFSET, cnt);

    // treat uninitialised flash (0xFFFF) or corrupt values as zero
    if (cnt > POWERUPCNT_REQUIRED) cnt = 0;

    cnt++;

    if (cnt >= POWERUPCNT_REQUIRED) {
        // threshold reached, trigger bind and reset counter
        task = POWERUPCNT_TASK_BIND;
        cnt = 0;
    } else {
        // wait for timeout, Do() will clear the counter if device stays on
        powerup_do = true;
        start_ms = millis();
    }

    EEPROM.put(POWERUPCNT_EE_OFFSET, cnt);
    EEPROM.commit();
}


void tPowerupCounter::Do(void)
{
    if (!powerup_do) return;
    if ((uint32_t)(millis() - start_ms) < (uint32_t)POWERUPCNT_TMO_MS) return;

    // timeout passed, clear counter so next boot starts fresh
    uint16_t cnt = 0;
    EEPROM.put(POWERUPCNT_EE_OFFSET, cnt);
    EEPROM.commit();
    powerup_do = false;
}


uint8_t tPowerupCounter::Task(void)
{
    uint8_t ret = task;
    task = POWERUPCNT_TASK_NONE;
    return ret;
}


#endif // ESP_POWERUP_CNT_H

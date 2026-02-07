//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP Powerup Counter
//*******************************************************
#ifndef RP_POWERUP_H
#define RP_POWERUP_H
#pragma once

#include <inttypes.h>

typedef enum {
    POWERUPCNT_TASK_NONE = 0,
    POWERUPCNT_TASK_BIND,
} POWERUPCNT_TASK_ENUM;


class tPowerupCounter
{
  public:
    void Init(void) {}
    void Do(void) {}
    uint8_t Task(void) { return POWERUPCNT_TASK_NONE; }
};

#endif // RP_POWERUP_H

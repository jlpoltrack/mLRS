//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP Platform Glue
//*******************************************************
#ifndef RP_GLUE_H
#define RP_GLUE_H


//-------------------------------------------------------
// c-compatible definitions
// (can be included from .c files)
//-------------------------------------------------------

typedef enum {
    DISABLE = 0,
    ENABLE = !DISABLE
} FunctionalState;

typedef enum {
    HAL_OK       = 0x00U,
    HAL_ERROR    = 0x01U,
    HAL_BUSY     = 0x02U,
    HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

#define __NOP() __asm__ __volatile__ ("nop")

#define __REV16(x) __builtin_bswap16(x)
#define __REVSH(x) ((int16_t)__builtin_bswap16((uint16_t)(x)))
#define __REV(x)   __builtin_bswap32(x)


//-------------------------------------------------------
// c++ only: Arduino integration and Core 0/1 setup
//-------------------------------------------------------
#ifdef __cplusplus

#include <Arduino.h>

// undefine MIN/MAX from Pico SDK to prevent redefinition
#undef MIN
#undef MAX

#undef IRQHANDLER
#define IRQHANDLER(__Declaration__)  extern "C" { __Declaration__ }

inline void __disable_irq(void) { noInterrupts(); }
inline void __enable_irq(void)  { interrupts(); }

static uint8_t restart_controller = 0;
void main_loop(void);

#ifdef DEVICE_HAS_DRONECAN
extern "C" int16_t dc_hal_enable_isr(void);
#endif

void setup()
{
    // can2040 pio irq must be enabled from core 0 so the
    // handler runs on this core, keeping core 1 for the
    // main radio loop
#ifdef DEVICE_HAS_DRONECAN
    dc_hal_enable_isr();
#endif
}

void loop()
{
    // core 0: idle loop, CAN2040 PIO IRQ runs in background
    delay(1);
}

void setup1(void) {}
void loop1(void) { main_loop(); }

#define INITCONTROLLER_ONCE \
    if(restart_controller <= 1){ \
    if(restart_controller == 0){
#define RESTARTCONTROLLER \
    }
#define INITCONTROLLER_END \
    restart_controller = UINT8_MAX; \
    }
#define GOTO_RESTARTCONTROLLER \
    rp2040.reboot();

#endif // __cplusplus

#endif // RP_GLUE_H

//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP2040 Platform Glue
//*******************************************************
#ifndef RP_GLUE_H
#define RP_GLUE_H

#include <Arduino.h>

#define IRQHANDLER(__Declaration__)  extern "C" { __Declaration__ }

inline void __disable_irq(void) { noInterrupts(); }
inline void __enable_irq(void)  { interrupts(); }

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

// Byte swap macros
#define __REV16(x) __builtin_bswap16(x)
#define __REV(x)   __builtin_bswap32(x)

// Controller control
static uint8_t restart_controller = 0;

void main_loop(void);

// Early UART debug helper - available before normal debug system
// Uses Serial1 (UART0) on GPIO 0 (TX) / GPIO 1 (RX)
#define DBG_BOOT(s)  Serial1.println(s); Serial1.flush()
#define DBG_PRINT(s) Serial1.print(s)
#define DBG_PRINTLN(s) Serial1.println(s); Serial1.flush()

void setup() {
    // Initialize Serial1 (hardware UART) for early debug
    // TX = GPIO 0, RX = GPIO 1
    Serial1.setTX(0);
    Serial1.setRX(1);
    Serial1.begin(115200);
    delay(100);  // Brief delay for UART to stabilize

    DBG_BOOT("=== RP2040 mLRS Boot ===");

    // early debug: blink onboard LED 3x to show firmware is alive
    pinMode(25, OUTPUT);
    DBG_BOOT("LED pin configured");

    for (int i = 0; i < 3; i++) {
        digitalWrite(25, HIGH);
        delay(100);
        digitalWrite(25, LOW);
        delay(100);
    }
    DBG_BOOT("LED blink complete, entering main_loop");
}

void loop() { main_loop(); }

#define INITCONTROLLER_ONCE \
    if(restart_controller <= 1){ \
    if(restart_controller == 0){
#define RESTARTCONTROLLER \
    }
#define INITCONTROLLER_END \
    restart_controller = UINT8_MAX; \
    }
#define GOTO_RESTARTCONTROLLER \
    restart_controller = 1; \
    return;

// Dummy macros for STM32 compatibility
#define UNUSED(x) (void)(x)
#define __NOP() __asm__ __volatile__ ("nop")

// RP2040 at 133 MHz: ~7.5 ns per cycle, so 50 ns needs ~7 NOPs
// SX126x requires: t6=15ns, t1=32ns, t8=31.25ns around chip select
inline void delay_ns(uint32_t ns) {
    // Approximate: each NOP is ~7.5ns at 133MHz
    // For 50ns, do 8 NOPs to be safe
    volatile uint32_t count = (ns / 8) + 1;
    while (count--) {
        __NOP();
    }
}

#endif // RP_GLUE_H

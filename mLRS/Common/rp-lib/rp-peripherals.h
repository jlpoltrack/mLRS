//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP2040 Peripherals
//*******************************************************
#ifndef RP_PERIPHERALS_H
#define RP_PERIPHERALS_H

#include <Arduino.h>
#include <hardware/gpio.h>
#include <hardware/structs/sio.h>

typedef enum {
    MODE_ANALOG           = 0,
    MODE_GPIO_INPUT       = 1,
    MODE_GPIO_OUTPUT_LOW  = 2,
    MODE_GPIO_OUTPUT_HIGH = 3,
    MODE_GPIO_INPUT_PU    = 4,
    MODE_GPIO_INPUT_PD    = 5,
} IOMODEENUM;

// STM32-compatible mode aliases
#define IO_MODE_INPUT_ANALOG      MODE_GPIO_INPUT_PD  // use pull-down for safety when no hardware attached
#define IO_MODE_INPUT_PD          MODE_GPIO_INPUT_PD
#define IO_MODE_INPUT_PU          MODE_GPIO_INPUT_PU
#define IO_MODE_OUTPUT_PP_LOW     MODE_GPIO_OUTPUT_LOW
#define IO_MODE_OUTPUT_PP_HIGH    MODE_GPIO_OUTPUT_HIGH

// RP2040 / RP2350 Pin mapping
#define IO_P0  0
#define IO_P1  1
#define IO_P2  2
#define IO_P3  3
#define IO_P4  4
#define IO_P5  5
#define IO_P6  6
#define IO_P7  7
#define IO_P8  8
#define IO_P9  9
#define IO_P10 10
#define IO_P11 11
#define IO_P12 12
#define IO_P13 13
#define IO_P14 14
#define IO_P15 15
#define IO_P16 16
#define IO_P17 17
#define IO_P18 18
#define IO_P19 19
#define IO_P20 20
#define IO_P21 21
#define IO_P22 22
#define IO_P23 23
#define IO_P24 24
#define IO_P25 25
#define IO_P26 26
#define IO_P27 27
#define IO_P28 28
#define IO_P29 29

// GPIO functions - must be defined before gpio_init which uses them
inline void gpio_low(uint8_t pin) { sio_hw->gpio_clr = (1ul << pin); }
inline void gpio_high(uint8_t pin) { sio_hw->gpio_set = (1ul << pin); }
inline void gpio_toggle(uint8_t pin) { sio_hw->gpio_togl = (1ul << pin); }
inline uint8_t gpio_read(uint8_t pin) { return (sio_hw->gpio_in & (1ul << pin)) ? 1 : 0; }
inline uint8_t gpio_read_activehigh(uint8_t pin) { return gpio_read(pin); }
inline uint8_t gpio_read_activelow(uint8_t pin) { return !gpio_read(pin); }

inline void gpio_init(uint8_t pin, IOMODEENUM mode) {
    if (mode == MODE_GPIO_INPUT) {
        pinMode(pin, INPUT);
    } else if (mode == MODE_GPIO_INPUT_PU) {
        pinMode(pin, INPUT_PULLUP);
    } else if (mode == MODE_GPIO_INPUT_PD) {
        pinMode(pin, INPUT_PULLDOWN);
    } else if (mode == MODE_GPIO_OUTPUT_LOW) {
        pinMode(pin, OUTPUT);
        gpio_low(pin);
    } else if (mode == MODE_GPIO_OUTPUT_HIGH) {
        pinMode(pin, OUTPUT);
        gpio_high(pin);
    }
}

#endif // RP_PERIPHERALS_H

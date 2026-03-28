//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP Peripherals
//*******************************************************
#ifndef RP_PERIPHERALS_H
#define RP_PERIPHERALS_H

#include <hardware/gpio.h>

typedef enum {
    MODE_ANALOG           = 0,
    MODE_GPIO_OUTPUT_LOW  = 1,
    MODE_GPIO_OUTPUT_HIGH = 2,
    MODE_GPIO_INPUT_PU    = 3,
    MODE_GPIO_INPUT_PD    = 4,
    MODE_GPIO_INPUT       = 5,
} IOMODEENUM;

// STM32-compatible mode aliases
#define IO_MODE_INPUT             MODE_GPIO_INPUT
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
#define IO_P23 23  // note: Pico: SMPS mode control, Pico W: WL_ON (CYW43 power enable). not broken out.
#define IO_P24 24  // note: Pico: USB VBUS detect, Pico W: WL_D (CYW43 SPI data). not broken out.
#define IO_P25 25  // note: Pico: onboard LED, Pico W: WL_CS (CYW43 SPI chip select). LED is via CYW43 on W.
#define IO_P26 26
#define IO_P27 27
#define IO_P28 28
#define IO_P29 29  // note: Pico: ADC3, Pico W: WL_CLK (CYW43 SPI clock). not broken out on W.
// RP2350 extra pins
#define IO_P30 30
#define IO_P31 31
#define IO_P32 32
#define IO_P33 33
#define IO_P34 34
#define IO_P35 35
#define IO_P36 36
#define IO_P37 37
#define IO_P38 38
#define IO_P39 39
#define IO_P40 40
#define IO_P41 41
#define IO_P42 42
#define IO_P43 43
#define IO_P44 44
#define IO_P45 45
#define IO_P46 46
#define IO_P47 47

// GPIO functions - single register writes, handles both GPIO banks
inline void gpio_low(uint8_t pin)    { gpio_clr_mask_n(pin >> 5, 1u << (pin & 0x1f)); }
inline void gpio_high(uint8_t pin)   { gpio_set_mask_n(pin >> 5, 1u << (pin & 0x1f)); }
inline void gpio_toggle(uint8_t pin) { gpio_xor_mask_n(pin >> 5, 1u << (pin & 0x1f)); }
inline uint8_t gpio_read(uint8_t pin) { return gpio_get(pin); }
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

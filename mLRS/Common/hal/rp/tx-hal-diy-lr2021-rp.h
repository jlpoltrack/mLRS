//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************

//-------------------------------------------------------
// RP2350, DIY LR2021 TX, Raspberry Pi Pico 2 W
//-------------------------------------------------------

// single external LED, com on USB, no serial port
#define DEVICE_HAS_SINGLE_LED
#define DEVICE_HAS_JRPIN5
#define DEVICE_HAS_NO_SERIAL


//-- UARTS
// UARTC = COM (CLI)
// UART = JR pin5
// UARTF = debug port

#define UARTC_USE_SERIAL          // com via USB

// no UARTD, the CYW wifi bridge is on-chip and does not use the serial2 port

#define UART_USE_PIO_HALF_DUPLEX  // JR Pin5 UART
#define UART_BAUD                 400000
#define UART_TX_PIN               IO_P9

#define UARTF_USE_SERIAL2         // debug port
#define UARTF_BAUD                115200
#define UARTF_TXBUFSIZE           1024
#define UARTF_TX_PIN              IO_P4


//-- SX1: LR20xx & SPI
// SPI1 pins: SCK=10, MOSI=11, MISO=12

#define SPI_USE_SPI1
#define SPI_MISO                  IO_P12
#define SPI_MOSI                  IO_P11
#define SPI_SCK                   IO_P10
#define SPI_CS_IO                 IO_P13
#define SPI_FREQUENCY             16000000L  // 16 MHz max per datasheet

#define SX_RESET                  IO_P7
#define SX_BUSY                   IO_P8
#define SX_DIO                    IO_P6 // labeled IRQ on board, DIO9 on LR2021F33

#define SX_USE_TCXO_VOLTAGE       LR20XX_TCXO_SUPPLY_VOLTAGE_3_3

#define SX_USE_IRQ_DIO_NO         LR20XX_DIO_9

#define SX_USE_RFSW_DIO_CONFIG  {\
    { LR20XX_DIO_5, LR20XX_DIO_RF_SWITCH_CONFIG_RX_HF }, \
    { LR20XX_DIO_6, LR20XX_DIO_RF_SWITCH_CONFIG_TX_LF }, \
    { LR20XX_DIO_7, LR20XX_DIO_RF_SWITCH_CONFIG_RX_HF | LR20XX_DIO_RF_SWITCH_CONFIG_TX_HF }, \
    { LR20XX_DIO_8, LR20XX_DIO_RF_SWITCH_CONFIG_TX_HF } }


IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH);
    gpio_init(SX_DIO, IO_MODE_INPUT_PD);
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU);
}

bool sx_busy_read(void) { return (gpio_read_activehigh(SX_BUSY)) ? true : false; }

void sx_amp_transmit(void) {}
void sx_amp_receive(void) {}

void sx_band(bool high_band) {} // band switching is done by the LR20xx DIO rf switch config

void sx_dio_init_exti_isroff(void) { detachInterrupt(SX_DIO); }
void sx_dio_enable_exti_isr(void) { attachInterrupt(digitalPinToInterrupt(SX_DIO), SX_DIO_EXTI_IRQHandler, RISING); }
void sx_dio_exti_isr_clearflag(void) {}


//-- Button

void button_init(void) {}
bool button_pressed(void) { return false; }


//-- LEDs

#define LED_RED                   IO_P16  // external LED (Pico 2 W: on-board LED is behind CYW43)

void leds_init(void) { gpio_init(LED_RED, IO_MODE_OUTPUT_PP_LOW); }
void led_red_off(void) { gpio_low(LED_RED); }
void led_red_on(void) { gpio_high(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }


//-- POWER

#define POWER_PA_NICERF_LR2021F33
#include "../hal-power-pa.h"

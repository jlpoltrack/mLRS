//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// RX DIY WeAct STM32H503Cx CoreBoard + E22 (SX126x), 915 MHz FCC
//-------------------------------------------------------
// Board: WeActStudio WeAct-STM32H503Cx_CoreBoard_V10, STM32H503CBU6 (UFQFPN48)
//   https://github.com/WeActStudio/WeActStudio.STM32H503CoreBoard
//
// Occupied by the board, do not use:
//   PH0/PH1     8 MHz HSE crystal (Y2)
//   PC14/PC15   32.768 kHz LSE crystal (Y1)
//   PA13/PA14   SWDIO/SWCLK, on the 4 pin SWD header P3
//   PA11/PA12   USB DN/DP, wired to the Type-C connector
//   PA4..PA7    SPI1 + CS of the W25Qxx footprint (U3); used here for the E22
//   PC13        blue user LED D2, via R5 5.1k from 3V3 -> LED lights when PC13 is LOW
//   PA0         user KEY SW1, to GND (R1 pull-up is NC, so the internal pull-up is used)
//
//
// Wiring to the E22-900M / SX1262 module:
//   module NSS  -> PA4    (GPIO out, software NSS)
//   module SCK  -> PA5    (SPI1_SCK,  AF5)
//   module MISO -> PA6    (SPI1_MISO, AF5, input)
//   module MOSI -> PA7    (SPI1_MOSI, AF5)
//   module NRST -> PB0    module BUSY -> PB1    module DIO1 -> PB2
//   module RXEN -> PB4    module TXEN -> PB5
//
// The board has only one user LED, so DEVICE_HAS_SINGLE_LED is used and only the
// led_red_xxx() functions are provided.

#define DEVICE_HAS_OUT
#define DEVICE_HAS_SINGLE_LED


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

// H5 erases in 8 kB sectors, 8 sectors per bank, 2 banks = 16 sectors on the 128 kB H503CB.
// Sectors 13 and 14 are the emulated eeprom and sector 15 is the powerup counter page
// (POWERUPCNT_EE_PAGE = EE_START_PAGE + 2), i.e. 0x0801A000 .. 0x0801FFFF is reserved
// and the firmware must stay below 104 kB.
#define EE_START_PAGE             13 // 128 kB flash, 8 kB sector

#define MICROS_TIMx               TIM3

#define CLOCK_TIMx                TIM2
#define CLOCK_IRQn                TIM2_IRQn
#define CLOCK_IRQHandler          TIM2_IRQHandler
//#define CLOCK_IRQ_PRIORITY        10


//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_UART1_PA9PA10 // serial
#define UARTB_BAUD                RX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           RX_SERIAL_TXBUFSIZE
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           RX_SERIAL_RXBUFSIZE

#define UART_USE_UART2_PA2PA3 // out pin
#define UART_BAUD                 100000 // SBus normal baud rate, is being set later anyhow
#define UART_USE_TX
#define UART_TXBUFSIZE            256
#define UART_USE_TX_ISR
//#define UART_USE_RX
//#define UART_RXBUFSIZE            512
#define OUT_UARTx                 USART2 // UART_UARTx is not known yet, so define by hand

#define UARTF_USE_LPUART1_PB6PB7 // debug
#define UARTF_BAUD                115200
#define UARTF_USE_TX
#define UARTF_TXBUFSIZE           512
#define UARTF_USE_TX_ISR
//#define UARTF_USE_RX
//#define UARTF_RXBUFSIZE           512


//-- SX1: SX12xx & SPI

#define SPI_USE_SPI1              // SCK = PA5, MISO = PA6, MOSI = PA7, all AF5
#define SPI_CS_IO                 IO_PA4
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_9MHZ   // 250 MHz kernel clock / 32 = 7.8125 MHz

#define SX_RESET                  IO_PB0
#define SX_BUSY                   IO_PB1
#define SX_DIO                    IO_PB2
#define SX_RX_EN                  IO_PB4
#define SX_TX_EN                  IO_PB5

#define SX_DIO_EXTI               EXTI_IO_PB2
#define SX_DIO_EXTI_IRQn          EXTI2_IRQn
#define SX_DIO_EXTI_IRQHandler    EXTI2_IRQHandler
//#define SX_DIO_EXTI_IRQ_PRIORITY    11

void sx_init_gpio(void)
{
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
    gpio_init(SX_DIO, IO_MODE_INPUT_PD, IO_SPEED_VERYFAST);
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU, IO_SPEED_VERYFAST);
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
    gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
}

bool sx_busy_read(void)
{
    return (gpio_read_activehigh(SX_BUSY)) ? true : false;
}

void sx_amp_transmit(void)
{
    gpio_low(SX_RX_EN);
    gpio_high(SX_TX_EN);
}

void sx_amp_receive(void)
{
    gpio_low(SX_TX_EN);
    gpio_high(SX_RX_EN);
}

void sx_dio_init_exti_isroff(void)
{
    exti_init_isroff(SX_DIO_EXTI, EXTI_TRIG_RISING);

    NVIC_SetPriority(SX_DIO_EXTI_IRQn, SX_DIO_EXTI_IRQ_PRIORITY);
    NVIC_EnableIRQ(SX_DIO_EXTI_IRQn);
}

void sx_dio_enable_exti_isr(void)
{
    exti_enableisr(SX_DIO_EXTI);
}

void sx_dio_exti_isr_clearflag(void)
{
    exti_clearisrflag(SX_DIO_EXTI);
}


//-- Out port

void out_init_gpio(void)
{
}

void out_set_normal(void)
{
    LL_USART_Disable(OUT_UARTx);
    LL_USART_SetTXPinLevel(OUT_UARTx, LL_USART_TXPIN_LEVEL_STANDARD);
    LL_USART_Enable(OUT_UARTx);
}

void out_set_inverted(void)
{
    LL_USART_Disable(OUT_UARTx);
    LL_USART_SetTXPinLevel(OUT_UARTx, LL_USART_TXPIN_LEVEL_INVERTED);
    LL_USART_Enable(OUT_UARTx);
}


//-- Button

#define BUTTON                    IO_PA0 // user KEY SW1

void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
}

bool button_pressed(void)
{
    return gpio_read_activelow(BUTTON);
}


//-- LEDs
// the board has one user LED only, the blue D2 on PC13, and it is active LOW

#define LED_RED                   IO_PC13

void leds_init(void)
{
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT);
    gpio_high(LED_RED); // LED_RED_OFF
}

void led_red_off(void) { gpio_high(LED_RED); }
void led_red_on(void) { gpio_low(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }


//-- POWER

#define POWER_PA_NONE_SX126X
#include "../hal-power-pa.h"


//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_15,
};

uint32_t portb[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6,
    LL_GPIO_PIN_7, LL_GPIO_PIN_8, LL_GPIO_PIN_10, LL_GPIO_PIN_12, LL_GPIO_PIN_13, LL_GPIO_PIN_14, LL_GPIO_PIN_15,
};

uint32_t portc[] = {
    LL_GPIO_PIN_13,
};

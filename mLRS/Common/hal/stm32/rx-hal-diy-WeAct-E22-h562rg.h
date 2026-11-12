//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// RX DIY WeAct STM32H5 64pin CoreBoard + E22 (SX126x), 915 MHz FCC
//-------------------------------------------------------
// Board: WeActStudio WeAct-STM32H5_64PIN-CoreBoard_V11, STM32H562RGT6 (LQFP64)
//   https://github.com/WeActStudio/WeActStudio.STM32H5_64Pin_CoreBoard
//
// Occupied by the board, do not use:
//   PH0/PH1     8 MHz HSE crystal (X2)
//   PC14/PC15   32.768 kHz LSE crystal (X1)
//   PA13/PA14   SWDIO/SWCLK, on the SWD header P5, which also carries PA9/PA10
//   PA11/PA12   USB DM/DP, wired to the Type-C connector
//   PC8..PC12, PD2, PA8   microSD slot (PA8 = card detect)
//   PB9, PB11   not available, VCAP on the H562
//   PB2         blue user LED D1, via R1 5.1k to GND -> LED lights when PB2 is HIGH
//   PC13        user KEY, via R5 330R to 3V3 -> pressed is HIGH, internal pull-down is used
//
//
// Wiring to the E22-900M / SX1262 module. The headers are double rows, the pins are chosen so
// that only outer row pads are soldered, or an inner row pad whose outer neighbor stays unused:
//   module SCK  -> PB10   (SPI2_SCK,  AF5)  inner, outer neighbor PB11 is VCAP
//   module MISO -> PC2    (SPI2_MISO, AF5)  outer
//   module MOSI -> PC3    (SPI2_MOSI, AF5)  inner, outer neighbor PA0 unused
//   module NSS  -> PB0    (GPIO out, software NSS)  outer
//   module NRST -> PC4    outer
//   module BUSY -> PA6    outer
//   module DIO1 -> PC0    outer
//   module RXEN -> PB1    inner, outer neighbor PB2 is the on-board LED
//   module TXEN -> PB4    outer, on the other header
// PB13 would give an all outer SPI2, but it is a UCPD pin and specd for 10 MHz max.
//
// Wiring to the CAN transceiver (not on the board, e.g. a SN65HVD230 or TCAN332 breakout):
//   transceiver RX -> PB8  (FDCAN1_RX, AF9)
//   transceiver TX -> PB7  (FDCAN1_TX, AF9)
//
// The board has only one user LED, so DEVICE_HAS_SINGLE_LED is used and only the
// led_red_xxx() functions are provided.

#define DEVICE_HAS_OUT
#define DEVICE_HAS_SINGLE_LED
#define DEVICE_HAS_DRONECAN_FD


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

// H5 erases in 8 kB sectors, 64 sectors per bank, 2 banks = 128 sectors on the 1 MB H562RG.
// Sectors 125 and 126 are the emulated eeprom and sector 127 is the powerup counter page
// (POWERUPCNT_EE_PAGE = EE_START_PAGE + 2), all in bank 2.
#define EE_FLASH_SIZE_KB          1024
#define EE_START_PAGE             125 // 1 MB flash, 8 kB sector

#define MICROS_TIMx               TIM3

#define CLOCK_TIMx                TIM2
#define CLOCK_IRQn                TIM2_IRQn
#define CLOCK_IRQHandler          TIM2_IRQHandler
//#define CLOCK_IRQ_PRIORITY        10


//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_UART2_PA2PA3 // serial
#define UARTB_BAUD                RX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           RX_SERIAL_TXBUFSIZE
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           RX_SERIAL_RXBUFSIZE

#define UART_USE_UART6_PC6PC7 // out pin
#define UART_BAUD                 100000 // SBus normal baud rate, is being set later anyhow
#define UART_USE_TX
#define UART_TXBUFSIZE            256
#define UART_USE_TX_ISR
//#define UART_USE_RX
//#define UART_RXBUFSIZE            512
#define OUT_UARTx                 USART6 // UART_UARTx is not known yet, so define by hand

#define UARTF_USE_UART1_PA9PA10 // debug, on the SWD header P5
#define UARTF_BAUD                115200
#define UARTF_USE_TX
#define UARTF_TXBUFSIZE           512
#define UARTF_USE_TX_ISR
//#define UARTF_USE_RX
//#define UARTF_RXBUFSIZE           512


//-- CAN
// FDCAN1 on LQFP64: RX is PA11 or PB8, TX is PA12 or PB7. PA11/PA12 are USB on this board.

#define CAN_USE_FDCAN1_PB8PB7


//-- SX1: SX12xx & SPI

#define SPI_USE_SPI2              // SCK = PB10, MISO = PC2, MOSI = PC3, all AF5
#define SPI_USE_SCK_IO            IO_PB10
#define SPI_USE_MISO_IO           IO_PC2
#define SPI_USE_MOSI_IO           IO_PC3
#define SPI_CS_IO                 IO_PB0
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_9MHZ   // 250 MHz kernel clock / 32 = 7.8125 MHz

#define SX_RESET                  IO_PC4
#define SX_BUSY                   IO_PA6
#define SX_DIO                    IO_PC0
#define SX_RX_EN                  IO_PB1
#define SX_TX_EN                  IO_PB4

#define SX_DIO_EXTI               EXTI_IO_PC0
#define SX_DIO_EXTI_IRQn          EXTI0_IRQn
#define SX_DIO_EXTI_IRQHandler    EXTI0_IRQHandler
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

#define BUTTON                    IO_PC13 // user KEY, to 3V3

void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PD, IO_SPEED_DEFAULT);
}

bool button_pressed(void)
{
    return gpio_read_activehigh(BUTTON);
}


//-- LEDs
// the board has one user LED only, the blue D1 on PB2, and it is active HIGH

#define LED_RED                   IO_PB2

void leds_init(void)
{
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
    gpio_low(LED_RED); // LED_RED_OFF
}

void led_red_off(void) { gpio_low(LED_RED); }
void led_red_on(void) { gpio_high(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }


//-- POWER

#define POWER_PA_NONE_SX126X
#include "../hal-power-pa.h"


//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_7,
    LL_GPIO_PIN_15,
};

uint32_t portb[] = {
    LL_GPIO_PIN_3, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_12, LL_GPIO_PIN_13, LL_GPIO_PIN_14, LL_GPIO_PIN_15,
};

uint32_t portc[] = {
    LL_GPIO_PIN_1, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
};

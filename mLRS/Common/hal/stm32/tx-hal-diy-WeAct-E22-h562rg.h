//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// TX DIY WeAct STM32H5 64pin CoreBoard + E22 (SX126x), 915 MHz FCC
//-------------------------------------------------------
// Board: WeActStudio WeAct-STM32H5_64PIN-CoreBoard_V11, STM32H562RGT6 (LQFP64)
//   https://github.com/WeActStudio/WeActStudio.STM32H5_64Pin_CoreBoard
//
// This is the Tx counterpart of rx-hal-diy-WeAct-E22-h562rg.h. Radio wiring, LED and
// button are identical to the rx target, so the same hardware can be flashed either way.
//
// Occupied by the board, do not use:
//   PH0/PH1     8 MHz HSE crystal (X2)
//   PC14/PC15   32.768 kHz LSE crystal (X1)
//   PA13/PA14   SWDIO/SWCLK, on the SWD header P5, which also carries PA9/PA10
//   PA11/PA12   USB DM/DP, wired to the Type-C connector, used here for the Com port
//   PC8..PC12, PD2   microSD slot, PA8 is its card detect but the slot is never used
//   PB9, PB11   not available, VCAP on the H562
//   PB2         blue user LED D1, via R1 5.1k to GND -> LED lights when PB2 is HIGH
//   PC13        user KEY, via R5 330R to 3V3 -> pressed is HIGH, internal pull-down is used
//
//
// The headers are double rows, the pins are chosen so that only outer row pads are soldered,
// or an inner row pad whose outer neighbor stays unused.
//
// Wiring to the E22-900M / SX1262 module, same as on the rx target:
//   module SCK  -> PB10   (SPI2_SCK,  AF5)  inner, outer neighbor PB11 is VCAP
//   module MISO -> PC2    (SPI2_MISO, AF5)  outer
//   module MOSI -> PC3    (SPI2_MOSI, AF5)  inner, outer neighbor PA0 unused
//   module NSS  -> PB0    (GPIO out, software NSS)  outer
//   module NRST -> PC4    outer
//   module BUSY -> PA6    outer
//   module DIO1 -> PC0    outer
//   module RXEN -> PB1    inner, outer neighbor PB2 is the on-board LED
//   module TXEN -> PB4    outer, on the other header
//
// Wiring to the radio:
//   JR bay pin5 -> PC7    (USART6_RX, AF7)  outer, also serves as the IN pin
//   JR bay GND  -> PC9    (GPIO out low, signal ground)  outer
//   serial Tx   -> PB13   (UART5_TX, AF14)  outer
//   serial Rx   -> PB15   (UART5_RX, AF14)  outer
//   debug Tx    -> PA9    (USART1_TX, AF7)  outer, also on the SWD header P5
//
// The JR pin5 signal sits on the USART6 *Rx* pin and is driven by the peripheral's own
// invert + swap, so it needs no diode and no external inverter. PC6 is USART6_TX and stays
// unsoldered.
//
// Wiring to the display and 5-way switch:
//   display SCL -> PB6    (I2C1_SCL, AF4)  outer
//   display SDA -> PB7    (I2C1_SDA, AF4)  inner, outer neighbor PB8 unused
//   5-way resistor chain -> PA4  (ADC12_INP18)  outer, bridged to PA3 for its pull-up
//   5-way GND            -> PA2  (GPIO out low)  outer
//
// Wiring to the WS2812 RGB LED:
//   LED DIN     -> PA8    (TIM1_CH1, AF1)  the microSD card detect, the slot is not used

#define DEVICE_HAS_JRPIN5
#define DEVICE_HAS_IN_ON_JRPIN5_RX
#define DEVICE_HAS_COM_ON_USB
#define DEVICE_HAS_I2C_DISPLAY
#define DEVICE_HAS_SINGLE_LED_RGB


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

// H5 erases in 8 kB sectors, 64 sectors per bank, 2 banks = 128 sectors on the 1 MB H562RG.
// Sectors 126 and 127 are the emulated eeprom, in bank 2. The rx target additionally reserves
// a sector for the powerup counter, but that is a CommonRx feature.
#define EE_FLASH_SIZE_KB          1024
#define EE_START_PAGE             126 // 1 MB flash, 8 kB sector

#define MICROS_TIMx               TIM3
#define MICROS_TIM_NAMEPREFIX     TIM3_

#define CLOCK_TIMx                TIM2
#define CLOCK_IRQn                TIM2_IRQn
#define CLOCK_IRQHandler          TIM2_IRQHandler
//#define CLOCK_IRQ_PRIORITY        10


//-- UARTS
// UARTB = serial port
// UARTC (or USB) = com (CLI) port
// UARTD = serial2 port or wireless bridge port
// UART  = JR bay pin5
// UARTE = in port, SBus or whatever
// UARTF or SWUART = debug port

#define UARTB_USE_UART5 // serial
#define UARTB_USE_TX_IO           IO_PB13
#define UARTB_USE_RX_IO           IO_PB15
#define UARTB_USE_IO_AF           IO_AF_14
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

// com is on USB, see DEVICE_HAS_COM_ON_USB, so no UARTC

#define UART_USE_UART6_PC6PC7 // JR pin5, MBridge // PC6 = Tx, PC7 = Rx, both AF7
#define UART_BAUD                 400000
#define UART_USE_TX
#define UART_TXBUFSIZE            512
#define UART_USE_TX_ISR
#define UART_USE_RX
#define UART_RXBUFSIZE            512

#define JRPIN5_FULL_INTERNAL_ON_RX // does not require an external diode, signal is on PC7

// the in port shares the pin5 pin, see DEVICE_HAS_IN_ON_JRPIN5_RX, so no UARTE

#define UARTF_USE_UART1_PA9PA10 // debug, on the SWD header P5
#define UARTF_BAUD                115200
#define UARTF_USE_TX
#define UARTF_TXBUFSIZE           512
#define UARTF_USE_TX_ISR
//#define UARTF_USE_RX
//#define UARTF_RXBUFSIZE           512


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


//-- In port
// the in port shares the pin on the JR bay pin5, so it is fully handled by in_interface.h,
// no in_init_gpio(), in_set_normal(), in_set_inverted() are needed here


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
// a single external WS2812 RGB LED on PA8, the on-board blue D1 on PB2 is kept off

#define WS2812_NUMBER_OF_LEDS     1
#define WS2812_IO                 IO_PA8
#define WS2812_IO_AF              IO_AF_1
#define WS2812_TIMx               TIM1
#define WS2812_TIMno              1
#define WS2812_CHno               1
#define WS2812_DMA_CHANNEL_x      LL_DMA_CHANNEL_0 // I2C uses channels 6 and 7
#define WS2812_FREQUENCY          250 // TIM1 clock = APB2 = 250 MHz
#include "../../thirdparty/stdstm32-ws2812.h"

tWs2812Color leds_current_color;

void leds_set_color(tWs2812Color color)
{
    if (color == leds_current_color) return;
    leds_current_color = color;
    ws2812_fill(0, color);
    ws2812_send();
}

void leds_init(void)
{
    gpio_init(IO_PB2, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT); // on-board LED off
    ws2812_init();
    leds_current_color = 0;
    ws2812_send();

    // signal ground for JR pin5, done here in leds_init() to ensure it is called
    gpio_init(IO_PC9, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
    // ground for the 5-way switch
    gpio_init(IO_PA2, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
}

void led_red_off(void) { leds_set_color(0); }
void led_red_on(void) { leds_set_color(WS2812_RED); }
void led_red_toggle(void) { (leds_current_color == WS2812_RED) ? led_red_off() : led_red_on(); }

void led_green_off(void) { leds_set_color(0); }
void led_green_on(void) { leds_set_color(WS2812_GREEN); }
void led_green_toggle(void) { (leds_current_color == WS2812_GREEN) ? led_green_off() : led_green_on(); }

void led_blue_off(void) { leds_set_color(0); }
void led_blue_on(void) { leds_set_color(WS2812_BLUE); }
void led_blue_toggle(void) { (leds_current_color == WS2812_BLUE) ? led_blue_off() : led_blue_on(); }

void led_purple_off(void) { leds_set_color(0); }
void led_purple_on(void) { leds_set_color(WS2812_PURPLE); }
void led_purple_toggle(void) { (leds_current_color == WS2812_PURPLE) ? led_purple_off() : led_purple_on(); }


//-- 5 Way Switch
// PA4: two-wire 5-way module, all five keys on one resistor ladder to GND (PA2)
// PA3: pull-up for PA4, analog mode disables the pin's own pull-up; PA3 is bridged to PA4

#define FIVEWAY_ADCx              ADC1
#define FIVEWAY_ADC_IO            IO_PA4 // ADC12_INP18
#define FIVEWAY_ADC_CHANNELx      LL_ADC_CHANNEL_18

void fiveway_init(void)
{
    gpio_init(IO_PA3, IO_MODE_INPUT_PU, IO_SPEED_DEFAULT);
    adc_init_begin(FIVEWAY_ADCx);
    adc_init_one_channel(FIVEWAY_ADCx);
    adc_config_channel(FIVEWAY_ADCx, LL_ADC_REG_RANK_1, FIVEWAY_ADC_CHANNELx, FIVEWAY_ADC_IO);
    adc_enable(FIVEWAY_ADCx);
    delay_us(100);
    adc_start_conversion(FIVEWAY_ADCx);
}

uint16_t fiveway_adc_read(void)
{
    return LL_ADC_REG_ReadConversionData12(FIVEWAY_ADCx);
}

uint8_t fiveway_read(void)
{
    uint16_t adc = LL_ADC_REG_ReadConversionData12(FIVEWAY_ADCx);
    if (adc < (65+200)) return (1 << KEY_DOWN); // 65
    if (adc > (640-200) && adc < (640+200)) return (1 << KEY_RIGHT); // 640
    if (adc > (1150-200) && adc < (1150+200)) return (1 << KEY_UP); // 1150
    if (adc > (1676-200) && adc < (1676+200)) return (1 << KEY_LEFT); // 1676
    if (adc > (2193-200) && adc < (2193+200)) return (1 << KEY_CENTER); // 2193
    return 0;
}


//-- Display I2C

#define I2C_USE_I2C1              // PB6, PB7
#define I2C_CLOCKSPEED_400KHZ     // not all displays seem to work well with I2C_CLOCKSPEED_1000KHZ
#define I2C_USE_DMAMODE
#define I2C_DEVICE_READY_TMO_MS   10 // the display is optional on this DIY board, don't hang without it
#define DISP_CONTENT_Y_BASE       22 // content 2 pixels lower than the default 20


//-- Buzzer
// has none


//-- POWER

#define POWER_PA_NONE_SX126X
#include "../hal-power-pa.h"


//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_15,
};

uint32_t portb[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6, LL_GPIO_PIN_7,
    LL_GPIO_PIN_8, LL_GPIO_PIN_10, LL_GPIO_PIN_12, LL_GPIO_PIN_13, LL_GPIO_PIN_14, LL_GPIO_PIN_15,
};

uint32_t portc[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5,
    LL_GPIO_PIN_6, LL_GPIO_PIN_7,
};

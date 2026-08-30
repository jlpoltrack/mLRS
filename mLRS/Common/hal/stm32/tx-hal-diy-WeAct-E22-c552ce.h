//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// TX DIY WeAct STM32C552Cx CoreBoard + E22 (SX126x), 915 MHz FCC
//-------------------------------------------------------
// Board: WeActStudio WeAct-STM32C5xxCxTx_Coreboard_V10, STM32C552CET6 (LQFP48)
//   https://github.com/WeActStudio/WeActStudio.STM32C5_48Pin_CoreBoard
//
// This is the Tx twin of rx-hal-diy-WeAct-E22-c552ce.h. The SX126x wiring, the clock
// tree, the LED and the button are identical, only the UART assignment differs.
//
// Occupied by the board, do not use:
//   PH0/PH1     8 MHz HSE crystal (Y2)
//   PC14/PC15   32.768 kHz LSE crystal (Y1)
//   PA13/PA14   SWDIO/SWCLK, on the 4 pin SWD header P1
//   PA11/PA12   USB DN/DP, wired to the Type-C connector
//   PA4..PA7    SPI1 + CS of the W25Qxx footprint (U3); used here for the E22
//   PE2         blue user LED D1, via R5 5.1k from 3V3 -> LED lights when PE2 is LOW
//   PA0         user KEY SW1, to GND (via R1 330R); used here as the 5 way ADC input
//
// Wiring to the E22-900M / SX1262 module, same as on the receiver:
//   module NSS  -> PA4    (GPIO out, software NSS)
//   module SCK  -> PA5    (SPI1_SCK,  AF5)
//   module MISO -> PA6    (SPI1_MISO, AF5, input)
//   module MOSI -> PA7    (SPI1_MOSI, AF5)
//   module NRST -> PB0    module BUSY -> PB1    module DIO1 -> PB2
//   module RXEN -> PB4    module TXEN -> PB5
//
// Wiring of the ports, all on pins broken out to the board's headers:
//   Serial  Tx -> PA9    Rx -> PA10   (USART1, AF7)
//   JR Pin5 -> PA2                    (USART2 Tx pin, AF7, half duplex)
//   JR GND  -> PA1                    (GPIO out low, sits next to PA2 on the header)
//   5 way   -> PA0                    (ADC1_IN0, one pin, no external pull-up)
//   Debug   Tx -> PB6                 (LPUART1, AF8)
//   Com/CLI -> the Type-C connector    (USB, PA11/PA12)
//
// The Com port is on the native USB, so the board's Type-C connector is the CLI. That
// frees LPUART1 for the Debug port. This matters because on the C552 in LQFP48 only
// USART1, USART2, LPUART1 and UART4 are usable at all (USART3 is on PC10/PC11 and
// UART5 on PC12/PD2, neither is bonded in this package), and UART4 needs PA0/PA1,
// where PA0 is the user KEY, so there are only three UARTs to hand out.
//
// The board has only one user LED, so DEVICE_HAS_SINGLE_LED is used and only the
// led_red_xxx() functions are provided.

//#define DEVICE_HAS_DIVERSITY
#define DEVICE_HAS_JRPIN5
#define DEVICE_HAS_IN_ON_JRPIN5_TX
#define DEVICE_HAS_COM_ON_USB
#define DEVICE_HAS_I2C_DISPLAY
#define DEVICE_HAS_FIVEWAY
#define DEVICE_HAS_SINGLE_LED


//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

// C5 erases in 8 kB pages, 32 pages per bank, 2 banks = 64 pages on the 512 kB C552CE.
// Pages 61 and 62 are the emulated eeprom and page 63 is the powerup counter page
// (POWERUPCNT_EE_PAGE = EE_START_PAGE + 2), i.e. 0x0807A000 .. 0x0807FFFF is reserved
// and the firmware must stay below 488 kB.
#define EE_START_PAGE             61 // 512 kB flash, 8 kB page

// C55x has no TIM3, so the micros timer is TIM16 here, as on the other TIM16 targets.
// It MUST be a 16 bit timer. The tx clock shares the micros timer and puts its CC1 delay on
// it, and clock_tx.h writes that compare as (uint16_t)(CNT + delay_us). On a 32 bit timer the
// counter runs past 0xFFFF after 65 ms and never comes back down, so the compare would match
// once and then never again, and the JR pin5 bridge would hang in TRANSMIT_PENDING forever.
// The C5's 32 bit timers are TIM2 and TIM5, and TIM6/TIM7 have no capture/compare unit at all.
#define MICROS_TIMx               TIM16
#define MICROS_TIM_NAMEPREFIX     TIM16_ // the tx clock uses the micros timer's CC1

#define CLOCK_TIMx                TIM2
#define CLOCK_IRQn                TIM2_IRQn
#define CLOCK_IRQHandler          TIM2_IRQHandler
//#define CLOCK_IRQ_PRIORITY        10


//-- UARTS
// UARTB = serial port
// UART  = JR bay pin5
// UARTE = in port, is shared with the JR bay pin5 uart here
// UARTF = debug port
// the com (CLI) port is on the native USB, so there is no UARTC

#define UARTB_USE_UART1_PA9PA10 // serial
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UART_USE_UART2_PA2PA3 // JR pin5, MBridge
#define UART_BAUD                 400000
#define UART_USE_TX
#define UART_TXBUFSIZE            512
#define UART_USE_TX_ISR
#define UART_USE_RX
#define UART_RXBUFSIZE            512

#define JRPIN5_FULL_INTERNAL_ON_TX // does not require an external diode, signal is on PA2

#define UARTF_USE_LPUART1_PB6PB7 // debug
#define UARTF_BAUD                115200
#define UARTF_USE_TX
#define UARTF_TXBUFSIZE           512
#define UARTF_USE_TX_ISR
//#define UARTF_USE_RX
//#define UARTF_RXBUFSIZE           512


//-- I2C
// the OLED, an SSD1306 on a 4 pin module. External pull-ups are on the module.

#define I2C_USE_I2C1              // SCL = PB8, SDA = PB9, both AF4
#define I2C_CLOCKSPEED_400KHZ     // not all displays seem to work well with 1000 kHz


//-- SX1: SX12xx & SPI

#define SPI_USE_SPI1              // SCK = PA5, MISO = PA6, MOSI = PA7, all AF5
#define SPI_CS_IO                 IO_PA4
#define SPI_USE_CLK_LOW_1EDGE     // datasheet says CPHA = 0  CPOL = 0
#define SPI_USE_CLOCKSPEED_18MHZ  // 144 MHz PCLK2 / 8 = 18 MHz

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
    if (gpio_read_activehigh(SX_BUSY)) return true;
    uint32_t t = DWT->CYCCNT;
    while ((DWT->CYCCNT - t) < 58) {} // 400 ns at 144 MHz
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
// the in port is shared with the JR pin5 uart, so nothing to do here


//-- Button
// has none. PA0 carries the 5 way chain now, see below. button_pressed() is not called by
// the Tx code, only button_init() is, so nothing is lost here.

void button_init(void) {}

bool button_pressed(void) { return false; }


//-- LEDs
// the board has one user LED only, the blue D1 on PE2, and it is active LOW

#define LED_RED                   IO_PE2

void leds_init(void)
{
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_DEFAULT);
    gpio_high(LED_RED); // LED_RED_OFF
}

void led_red_off(void) { gpio_high(LED_RED); }
void led_red_on(void) { gpio_low(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }


//-- 5 Way Switch
// one ADC pin for all five keys, as on the G4 Tx boards:
//   PA0 - 5 switches to GND, through 0R / 10k / 22k / 47k / 100k, plus one pull-up to 3V3
//
// The pull-up must be an external resistor. PCSEL is an additional switch in front of the ADC
// on the C5, it does not replace GPIO MODER: with PA0 in digital input mode the pad is still
// cut off from the ADC and every conversion returns 0, measured on the board (IDR read high
// while DR stayed 0 with ADEN/ADSTART set). So the pin stays in analog mode, and analog mode
// disables the internal pull-up, exactly as on G4.
// The board's KEY SW1 hangs on PA0 too, through R1 330R to GND, i.e. it is the 0R key.
// PA0 is the only ADC pin left anyway: the C552 in LQFP48 has ADC1 on PA0..PA7 only
// (PC0..PC3, ADC1_IN8..11, are not bonded) and PA1..PA7 are all taken.
//
// FIVEWAY_PULLUP_KOHM is the fitted resistor. The key codes are 4095*R/(R+R_pu) and the
// thresholds are the midpoints between them, so retuning for another resistor is a one line
// change. 47k spreads the upper keys much wider than 10k does. Use fiveway_adc_read() to check.
//
// PA1 is driven low, so that the JR pin5 lead can pick up its ground on the pin right
// next to PA2 rather than being run to a GND pad.

#define JRPIN5_GND                IO_PA1

#define FIVEWAY_ADCx              ADC1
#define FIVEWAY_ADC_IO            IO_PA0 // ADC1_IN0
#define FIVEWAY_ADC_CHANNELx      LL_ADC_CHANNEL_0

#define FIVEWAY_PULLUP_KOHM       47 // external pull-up from PA0 to 3V3

#define FIVEWAY_CODE(r_kohm)      ((4095 * (r_kohm)) / ((r_kohm) + FIVEWAY_PULLUP_KOHM))
#define FIVEWAY_THRESH(r1, r2)    ((FIVEWAY_CODE(r1) + FIVEWAY_CODE(r2)) / 2)

#define KEY_DOWN_THRESH           FIVEWAY_THRESH(0, 10)   // 0R
#define KEY_LEFT_THRESH           FIVEWAY_THRESH(10, 22)  // 10k
#define KEY_RIGHT_THRESH          FIVEWAY_THRESH(22, 47)  // 22k
#define KEY_UP_THRESH             FIVEWAY_THRESH(47, 100) // 47k
#define KEY_CENTER_THRESH         ((FIVEWAY_CODE(100) + 4095) / 2) // 100k vs released

void fiveway_init(void)
{
    gpio_init(JRPIN5_GND, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_DEFAULT);
    gpio_low(JRPIN5_GND);

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
    if (adc < KEY_DOWN_THRESH) return (1 << KEY_DOWN);
    if (adc < KEY_LEFT_THRESH) return (1 << KEY_LEFT);
    if (adc < KEY_RIGHT_THRESH) return (1 << KEY_RIGHT);
    if (adc < KEY_UP_THRESH) return (1 << KEY_UP);
    if (adc < KEY_CENTER_THRESH) return (1 << KEY_CENTER);
    return 0;
}


//-- Buzzer
// has none


//-- POWER

#define POWER_PA_NONE_SX126X
#include "../hal-power-pa.h"


//-- TEST

uint32_t porta[] = {
    LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_15,
};

uint32_t portb[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_4, LL_GPIO_PIN_5, LL_GPIO_PIN_6,
    LL_GPIO_PIN_7, LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_12, LL_GPIO_PIN_13, LL_GPIO_PIN_14,
    LL_GPIO_PIN_15,
};

uint32_t portc[] = {
    LL_GPIO_PIN_13,
};

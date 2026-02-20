//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************

//-------------------------------------------------------
// RP2040/RP2350, DIY 2400 RX
//-------------------------------------------------------

#define DEVICE_HAS_SINGLE_LED
//#define DEVICE_HAS_SINGLE_LED_RGB
#define DEVICE_HAS_OUT
//#define DEVICE_HAS_DIVERSITY


//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_SERIAL1
#define UARTB_BAUD                RX_SERIAL_BAUDRATE
#define UARTB_TXBUFSIZE           RX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           RX_SERIAL_RXBUFSIZE
#define UARTB_TX_PIN              IO_P0
#define UARTB_RX_PIN              IO_P1

#define UART_USE_SERIAL2
#define UART_BAUD                 416666
#define UART_TXBUFSIZE            1024
#define UART_TX_PIN               IO_P8

#define UARTF_USE_SERIAL          // usb


//-- SX1: SX128x & SPI
// SPI0 default pins: SCK=18, MOSI=19, MISO=16

#define SPI_MISO                  IO_P16
#define SPI_MOSI                  IO_P19
#define SPI_SCK                   IO_P18
#define SPI_CS_IO                 IO_P17
#define SPI_FREQUENCY             18000000L  // 18 MHz
#define SX_RESET                  IO_P3
#define SX_BUSY                   IO_P2
#define SX_DIO1                   IO_P4
#define SX_TX_EN                  IO_P5
#define SX_RX_EN                  IO_P6

IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH);
    gpio_init(SX_DIO1, IO_MODE_INPUT_PD);
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU);  // when testing with no Sx chip, change to pull down
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW);
}

bool sx_busy_read(void) { return (gpio_read_activehigh(SX_BUSY)) ? true : false; }

void sx_amp_transmit(void) {
    gpio_low(SX_RX_EN);
    gpio_high(SX_TX_EN);
}

void sx_amp_receive(void)
{
    gpio_low(SX_TX_EN);
    gpio_high(SX_RX_EN);
}

void sx_dio_init_exti_isroff(void) { detachInterrupt(SX_DIO1); }
void sx_dio_enable_exti_isr(void) { attachInterrupt(digitalPinToInterrupt(SX_DIO1), SX_DIO_EXTI_IRQHandler, RISING); }
void sx_dio_exti_isr_clearflag(void) {}


//-- SX2: SX128x & SPIB
// SPI1 pins: SCK=10, MOSI=11, MISO=12, CS=13

#ifdef DEVICE_HAS_DIVERSITY

#define SPIB_USE_SPI1
#define SPIB_MISO                 IO_P12
#define SPIB_MOSI                 IO_P11
#define SPIB_SCK                  IO_P10
#define SPIB_CS_IO                IO_P13
#define SPIB_FREQUENCY            18000000L  // 18 MHz
#define SX2_RESET                 IO_P7
#define SX2_BUSY                  IO_P9
#define SX2_DIO1                  IO_P21
#define SX2_TX_EN                 IO_P26
#define SX2_RX_EN                 IO_P27

IRQHANDLER(void SX2_DIO_EXTI_IRQHandler(void);)

void sx2_init_gpio(void)
{
    gpio_init(SX2_RESET, IO_MODE_OUTPUT_PP_HIGH);
    gpio_init(SX2_DIO1, IO_MODE_INPUT_PD);
    gpio_init(SX2_BUSY, IO_MODE_INPUT_PU);
    gpio_init(SX2_TX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX2_RX_EN, IO_MODE_OUTPUT_PP_LOW);
}

bool sx2_busy_read(void) { return (gpio_read_activehigh(SX2_BUSY)) ? true : false; }

void sx2_amp_transmit(void)
{
    gpio_low(SX2_RX_EN);
    gpio_high(SX2_TX_EN);
}

void sx2_amp_receive(void)
{
    gpio_low(SX2_TX_EN);
    gpio_high(SX2_RX_EN);
}

void sx2_dio_init_exti_isroff(void) { detachInterrupt(SX2_DIO1); }
void sx2_dio_enable_exti_isr(void) { attachInterrupt(digitalPinToInterrupt(SX2_DIO1), SX2_DIO_EXTI_IRQHandler, RISING); }
void sx2_dio_exti_isr_clearflag(void) {}

#endif // DEVICE_HAS_DIVERSITY


//-- CAN bus (DroneCAN via can2040 PIO)

#ifdef DEVICE_HAS_DRONECAN

#define CAN_RX_PIN                IO_P14
#define CAN_TX_PIN                IO_P15

#endif  // DEVICE_HAS_DRONECAN


//-- Out port

void out_init_gpio(void) {}
void out_set_normal(void) { gpio_set_outover(UART_TX_PIN, GPIO_OVERRIDE_NORMAL); }
void out_set_inverted(void) { gpio_set_outover(UART_TX_PIN, GPIO_OVERRIDE_INVERT); }


//-- Button

#define BUTTON                    IO_P22

void button_init(void) { gpio_init(BUTTON, IO_MODE_INPUT_PU); }
bool button_pressed(void) { return gpio_read_activelow(BUTTON) ? true : false; }


//-- LEDs
#ifdef DEVICE_HAS_SINGLE_LED

#define LED_RED                   IO_P20

void leds_init(void) { gpio_init(LED_RED, IO_MODE_OUTPUT_PP_LOW); }
void led_red_off(void) { gpio_low(LED_RED); }
void led_red_on(void) { gpio_high(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }

#elif defined DEVICE_HAS_SINGLE_LED_RGB

#define LED_RGB                   IO_P23
#define LED_RGB_PIXEL_NUM         1
#include "../rp-hal-led-rgb.h"

#endif


//-- POWER

#define POWER_PA_E28_2G4M27SX
#include "../hal-power-pa.h"

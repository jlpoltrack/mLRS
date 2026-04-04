//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************

//-------------------------------------------------------
// RP2040/RP2350, DIY LR2021 RX
//-------------------------------------------------------

#define DEVICE_HAS_SINGLE_LED
//#define DEVICE_HAS_SINGLE_LED_RGB
//#define DEVICE_HAS_OUT
//#define DEVICE_HAS_DRONECAN


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


//-- SX1: LR20xx & SPI
// SPI0 default pins: SCK=18, MOSI=19, MISO=16

#define SPI_MISO                  IO_P16
#define SPI_MOSI                  IO_P19
#define SPI_SCK                   IO_P18
#define SPI_CS_IO                 IO_P17
#define SPI_FREQUENCY             16000000L  // 16 MHz max per datasheet

#define SX_RESET                  IO_P3
#define SX_BUSY                   IO_P2
#define SX_DIO1                   IO_P4

#define LR_DIO_IRQ_NO             7 // LR20XX_DIO_7

#define LR_DIO_RFSW
const uint8_t lr_dio_rfsw[] = { 5, 6, 8 }; // LR20XX DIO5, DIO6, DIO8
const uint8_t lr_dio_rfsw_config[] = {
    0x08, // DIO5: RX_HF
    0x04, // DIO6: TX_LF
    0x10, // DIO8: TX_HF
};
#define LR_DIO_RFSW_NUM  (sizeof(lr_dio_rfsw) / sizeof(lr_dio_rfsw[0]))

IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH);
    gpio_init(SX_DIO1, IO_MODE_INPUT_PD);
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU);
}

bool sx_busy_read(void) { return (gpio_read_activehigh(SX_BUSY)) ? true : false; }

void sx_amp_transmit(void) {}

void sx_amp_receive(void) {}

void sx_dio_init_exti_isroff(void) { detachInterrupt(SX_DIO1); }
void sx_dio_enable_exti_isr(void) { attachInterrupt(digitalPinToInterrupt(SX_DIO1), SX_DIO_EXTI_IRQHandler, RISING); }
void sx_dio_exti_isr_clearflag(void) {}


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

#define BUTTON                    IO_P27

void button_init(void) { gpio_init(BUTTON, IO_MODE_INPUT_PU); }
bool button_pressed(void) { return gpio_read_activelow(BUTTON) ? true : false; }


//-- LEDs
#ifdef DEVICE_HAS_SINGLE_LED

#define LED_RED                   IO_P25

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

#define POWER_USE_DEFAULT_RFPOWER_CALC
#define POWER_GAIN_DBM            0

#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_0_DBM, .mW = 1 },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_20_DBM, .mW = 100 },
};

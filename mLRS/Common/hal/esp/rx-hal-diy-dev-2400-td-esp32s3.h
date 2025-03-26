//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// ESP32S3 DIY DEV 2.4 GHz True Diversity Receiver
//-------------------------------------------------------

#define DEVICE_HAS_OUT
#define DEVICE_HAS_DIVERSITY_SINGLE_SPI // must be set, doesn't work without it
#define DEVICE_HAS_SINGLE_LED_RGB


//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_SERIAL
#define UARTB_USE_TX_IO           IO_P43
#define UARTB_USE_RX_IO           IO_P44
#define UARTB_BAUD                RX_SERIAL_BAUDRATE
#define UARTB_TXBUFSIZE           RX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           RX_SERIAL_RXBUFSIZE

#define UART_USE_SERIAL1 
#define UART_BAUD                 416666   // CRSF baud rate
#define UART_USE_TX_IO            IO_P37   // TX3
#define UART_USE_RX_IO            -1       // no Rx pin needed
#define UART_TXBUFSIZE            256

#define UARTF_USE_SERIAL2
#define UARTF_USE_TX_IO           IO_P35
#define UARTF_USE_RX_IO           -1
#define UARTF_BAUD                115200
#define UARTF_TXBUFSIZE           0  // TX FIFO = 128


//-- SX1: SX128x & SPI

#define SPI_CS_IO                 IO_P2
#define SPI_MISO                  IO_P11
#define SPI_MOSI                  IO_P12
#define SPI_SCK                   IO_P10
#define SPI_FREQUENCY             18000000L  // 18 MHz max per datasheet
#define SX_BUSY                   IO_P3
#define SX_DIO1                   IO_P4
#define SX_RESET                  IO_P5
#define SX_TX_EN                  IO_P6
#define SX_RX_EN                  IO_P7

#define SX_USE_REGULATOR_MODE_DCDC

IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_DIO1, IO_MODE_INPUT_ANALOG);
    gpio_init(SX_BUSY, IO_MODE_INPUT_ANALOG);
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW);
}

IRAM_ATTR bool sx_busy_read(void)
{
    return (gpio_read_activehigh(SX_BUSY)) ? true : false;
}

IRAM_ATTR void sx_amp_transmit(void)
{
    gpio_low(SX_RX_EN);
    gpio_high(SX_TX_EN);
}

IRAM_ATTR void sx_amp_receive(void)
{
    gpio_low(SX_TX_EN);
    gpio_high(SX_RX_EN);
}

void sx_dio_init_exti_isroff(void)
{
    detachInterrupt(SX_DIO1);
}

void sx_dio_enable_exti_isr(void)
{
    attachInterrupt(SX_DIO1, SX_DIO_EXTI_IRQHandler, RISING);
}

IRAM_ATTR void sx_dio_exti_isr_clearflag(void) {}


//-- SX2: SX128x & SPI

#define SX2_CS_IO                 IO_P17
#define SX2_BUSY                  IO_P18
#define SX2_DIO1                  IO_P14
#define SX2_RESET                 IO_P13
#define SX2_RX_EN                 IO_P8
#define SX2_TX_EN                 IO_P9

#define SX2_USE_REGULATOR_MODE_DCDC

IRQHANDLER(void SX2_DIO_EXTI_IRQHandler(void);)

void sx2_init_gpio(void)
{
    gpio_init(SX2_CS_IO, IO_MODE_OUTPUT_PP_HIGH);
    gpio_init(SX2_DIO1, IO_MODE_INPUT_ANALOG);
    gpio_init(SX2_BUSY, IO_MODE_INPUT_ANALOG);
    gpio_init(SX2_RESET, IO_MODE_OUTPUT_PP_LOW);
}

IRAM_ATTR void spib_select(void)
{
    gpio_low(SX2_CS_IO);
}

IRAM_ATTR void spib_deselect(void)
{
    gpio_high(SX2_CS_IO);
}

IRAM_ATTR bool sx2_busy_read(void)
{
    return (gpio_read_activehigh(SX2_BUSY)) ? true : false;
}

IRAM_ATTR void sx2_amp_transmit(void)
{
    gpio_low(SX2_RX_EN);
    gpio_high(SX2_TX_EN);
}

IRAM_ATTR void sx2_amp_receive(void)
{
    gpio_low(SX2_TX_EN);
    gpio_high(SX2_RX_EN);
}

void sx2_dio_init_exti_isroff(void)
{
    detachInterrupt(SX2_DIO1);
}

void sx2_dio_enable_exti_isr(void)
{
    attachInterrupt(SX2_DIO1, SX2_DIO_EXTI_IRQHandler, RISING);
}

void sx2_dio_exti_isr_clearflag(void) {}


//-- Out port

void out_init_gpio(void) {}

void out_set_normal(void) { gpio_matrix_out((gpio_num_t)UART_USE_TX_IO, U1TXD_OUT_IDX, false, false); }

void out_set_inverted(void) { gpio_matrix_out((gpio_num_t)UART_USE_TX_IO, U1TXD_OUT_IDX, true, false); }


//-- Button

#define BUTTON                    IO_P0

void button_init(void) { gpio_init(BUTTON, IO_MODE_INPUT_PU); }

IRAM_ATTR bool button_pressed(void) { return gpio_read_activelow(BUTTON) ? true : false; }


//-- LEDs
#include <NeoPixelBus.h>
#define LED_RED                    IO_P38
bool ledRedState;
bool ledGreenState;
bool ledBlueState;

NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt0Ws2812xMethod> ledRGB(1, LED_RED);

void leds_init(void)
{
    ledRGB.Begin();
    ledRGB.Show();
}

IRAM_ATTR void led_red_off(void)
{
    if (!ledRedState) return;
    ledRGB.SetPixelColor(0, RgbColor(0, 0, 0));
    ledRGB.Show();
    ledRedState = 0;
}

IRAM_ATTR void led_red_on(void)
{
    if (ledRedState) return;
    ledRGB.SetPixelColor(0, RgbColor(255, 0, 0));
    ledRGB.Show();
    ledRedState = 1;
}

IRAM_ATTR void led_red_toggle(void)
{
    if (ledRedState) { led_red_off(); } else { led_red_on(); }
}

IRAM_ATTR void led_green_off(void)
{
    if (!ledGreenState) return;
    ledRGB.SetPixelColor(0, RgbColor(0, 0, 0));
    ledRGB.Show();
    ledGreenState = 0;
}

IRAM_ATTR void led_green_on(void)
{
    if (ledGreenState) return;
    ledRGB.SetPixelColor(0, RgbColor(0, 255, 0));
    ledRGB.Show();
    ledGreenState = 1;
}

IRAM_ATTR void led_green_toggle(void)
{
    if (ledGreenState) { led_green_off(); } else { led_green_on(); }
}

IRAM_ATTR void led_blue_off(void)
{
    if (!ledBlueState) return;
    ledRGB.SetPixelColor(0, RgbColor(0, 0, 0));
    ledRGB.Show();
    ledBlueState = 0;
}

IRAM_ATTR void led_blue_on(void)
{
    if (ledBlueState) return;
    ledRGB.SetPixelColor(0, RgbColor(0, 0, 255));
    ledRGB.Show();
    ledBlueState = 1;
}

IRAM_ATTR void led_blue_toggle(void)
{
    if (ledBlueState) { led_blue_off(); } else { led_blue_on(); }
}


//-- POWER
#ifndef POWER_OVERLAY

#define POWER_GAIN_DBM            32 // gain of a PA stage if present
#define POWER_SX1280_MAX_DBM      SX1280_POWER_0_DBM // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_14_DBM, .mW = 25 },
    { .dbm = POWER_17_DBM, .mW = 50 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
};

#endif // !POWER_OVERLAY

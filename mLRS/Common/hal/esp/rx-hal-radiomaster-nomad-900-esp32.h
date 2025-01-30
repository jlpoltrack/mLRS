//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//*******************************************************

//-------------------------------------------------------
// ESP32, Radiomaster Nomad as Rx
//-------------------------------------------------------

// https://github.com/ExpressLRS/targets/blob/master/TX/Radiomaster%20Nomad.json

#define DEVICE_HAS_OUT
#define DEVICE_HAS_NO_DEBUG
#define DEVICE_HAS_DIVERSITY_SINGLE_SPI
#define DEVICE_HAS_SINGLE_LED_RGB
#define DEVICE_HAS_FAN_ONOFF


//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_SERIAL
#define UARTB_BAUD                RX_SERIAL_BAUDRATE
#define UARTB_TXBUFSIZE           RX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           RX_SERIAL_RXBUFSIZE

#define UART_USE_SERIAL1 
#define UART_BAUD                 416666   // CRSF baud rate
#define UART_USE_TX_IO            IO_P4    // JRPin5
#define UART_USE_RX_IO            -1       // no Rx pin needed
#define UART_TXBUFSIZE            256


//-- Out port

void out_init_gpio(void) {}

void out_set_normal(void) { gpio_matrix_out((gpio_num_t)UART_USE_TX_IO, U1TXD_OUT_IDX, false, false); }

void out_set_inverted(void) { gpio_matrix_out((gpio_num_t)UART_USE_TX_IO, U1TXD_OUT_IDX, true, false); }


//-- SX1: LR11xx & SPI

#define SPI_CS_IO                 IO_P27
#define SPI_MISO                  IO_P33
#define SPI_MOSI                  IO_P32
#define SPI_SCK                   IO_P25
#define SPI_FREQUENCY             16000000L
#define SX_RESET                  IO_P15
#define SX_BUSY                   IO_P36
#define SX_DIO1                   IO_P37

#define SX_USE_REGULATOR_MODE_DCDC

IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_DIO1, IO_MODE_INPUT_ANALOG);
    gpio_init(SX_BUSY, IO_MODE_INPUT_ANALOG);
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_LOW);
}

IRAM_ATTR bool sx_busy_read(void) { return (gpio_read_activehigh(SX_BUSY)) ? true : false; }

IRAM_ATTR void sx_amp_transmit(void) {}

IRAM_ATTR void sx_amp_receive(void) {}

void sx_dio_enable_exti_isr(void) { attachInterrupt(SX_DIO1, SX_DIO_EXTI_IRQHandler, RISING); }

void sx_dio_init_exti_isroff(void) { detachInterrupt(SX_DIO1); }

void sx_dio_exti_isr_clearflag(void) {}


//-- SX2: LR11xx & SPI

#define SX2_CS_IO                 IO_P13
#define SX2_BUSY                  IO_P39
#define SX2_DIO1                  IO_P34
#define SX2_RESET                 IO_P21

#define SX2_USE_REGULATOR_MODE_DCDC

IRQHANDLER(void SX2_DIO_EXTI_IRQHandler(void);)

void sx2_init_gpio(void)
{
    gpio_init(SX2_CS_IO, IO_MODE_OUTPUT_PP_HIGH);
    gpio_init(SX2_DIO1, IO_MODE_INPUT_ANALOG);
    gpio_init(SX2_BUSY, IO_MODE_INPUT_ANALOG);
    gpio_init(SX2_RESET, IO_MODE_OUTPUT_PP_LOW);
}

#define SX2_USE_REGULATOR_MODE_DCDC

IRAM_ATTR void spib_select(void) { gpio_low(SX2_CS_IO); }

IRAM_ATTR void spib_deselect(void) { gpio_high(SX2_CS_IO); }

IRAM_ATTR bool sx2_busy_read(void) { return (gpio_read_activehigh(SX2_BUSY)) ? true : false; }

IRAM_ATTR void sx2_amp_transmit(void) {}

IRAM_ATTR void sx2_amp_receive(void) {}

void sx2_dio_init_exti_isroff(void) { detachInterrupt(SX2_DIO1); }

void sx2_dio_enable_exti_isr(void) { attachInterrupt(SX2_DIO1, SX2_DIO_EXTI_IRQHandler, RISING); }

void sx2_dio_exti_isr_clearflag(void) {}


//-- Button

#define BUTTON                    IO_P14
#define BUTTON2                   IO_P12

void button_init(void)
{ 
    gpio_init(BUTTON, IO_MODE_INPUT_PU);
    gpio_init(BUTTON2, IO_MODE_INPUT_PU);
}

IRAM_ATTR bool button_pressed(void) { return (gpio_read_activelow(BUTTON) || gpio_read_activelow(BUTTON2)) ? true : false; }


//-- LEDs

#define LED_RGB                   IO_P22

#include <NeoPixelBus.h>
bool ledRedState;
bool ledGreenState;
bool ledBlueState;

uint8_t pixelNum = 2;

NeoPixelBus<NeoGrbFeature, NeoEsp32I2s0Ws2812xMethod> ledRGB(pixelNum, LED_RGB);

void leds_init(void)
{
    ledRGB.Begin();
    ledRGB.Show();
}

IRAM_ATTR void led_red_off(void)
{
    if (!ledRedState) return;
    ledRGB.SetPixelColor(0, RgbColor(0, 0, 0));
    ledRGB.SetPixelColor(1, RgbColor(0, 0, 0));
    ledRGB.Show();
    ledRedState = 0;
}

IRAM_ATTR void led_red_on(void)
{
    if (ledRedState) return;
    ledRGB.SetPixelColor(0, RgbColor(255, 0, 0));
    ledRGB.SetPixelColor(1, RgbColor(255, 0, 0));
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
    ledRGB.SetPixelColor(1, RgbColor(0, 0, 0));
    ledRGB.Show();
    ledGreenState = 0;
}

IRAM_ATTR void led_green_on(void)
{
    if (ledGreenState) return;
    ledRGB.SetPixelColor(0, RgbColor(0, 255, 0));
    ledRGB.SetPixelColor(1, RgbColor(0, 255, 0));
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
    ledRGB.SetPixelColor(1, RgbColor(0, 0, 0));
    ledRGB.Show();
    ledBlueState = 0;
}

IRAM_ATTR void led_blue_on(void)
{
    if (ledBlueState) return;
    ledRGB.SetPixelColor(0, RgbColor(0, 0, 255));
    ledRGB.SetPixelColor(1, RgbColor(0, 0, 255));
    ledRGB.Show();
    ledBlueState = 1;
}

IRAM_ATTR void led_blue_toggle(void)
{
    if (ledBlueState) { led_blue_off(); } else { led_blue_on(); }
}


//-- Cooling Fan

#define FAN_IO                    IO_P2

void fan_init(void) { gpio_init(FAN_IO, IO_MODE_OUTPUT_PP_LOW); }

IRAM_ATTR void fan_set_power(int8_t power_dbm)
{
    if (power_dbm >= POWER_23_DBM) { gpio_high(FAN_IO); } 
    else { gpio_low(FAN_IO); }
}


//-- POWER

#define SX_USE_LP_PA  // Nomad uses the low power amplifier for the 900 side

void lr11xx_rfpower_calc(const int8_t power_dbm, uint8_t* sx_power, int8_t* actual_power_dbm)
{
    uint8_t dac = 120;

    if (power_dbm > 28) { // -> 30
        dac = 95;
        *sx_power = 5;
        *actual_power_dbm = 30;
    } else if (power_dbm > 25) { // -> 27
        dac = 120;
        *sx_power = -3;
        *actual_power_dbm = 27;
    } else if (power_dbm > 22) { // -> 24
        dac = 120;
        *sx_power = -7;
        *actual_power_dbm = 24;
    } else if (power_dbm > 18) { // -> 20
        dac = 120;
        *sx_power = -11;
        *actual_power_dbm = 20;
    } else if (power_dbm > 15) { // -> 17
        dac = 120;
        *sx_power = -14;
        *actual_power_dbm = 17;
    } else if (power_dbm > 12) { // -> 14
        dac = 120;
        *sx_power = -16;
        *actual_power_dbm = 14;
    } else {
        dac = 150;
        *sx_power = -17;
        *actual_power_dbm = 10; // measures about 11 dBm
    }

    dacWrite(IO_P26, dac);
}

#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_14_DBM, .mW = 25 },
    //{ .dbm = POWER_17_DBM, .mW = 50 }, // 6 power levels allowed
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};

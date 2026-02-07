//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************

//-------------------------------------------------------
// RP2040/RP2350, GENERIC 2400 TX
//-------------------------------------------------------

#define DEVICE_HAS_SINGLE_LED
#define DEVICE_HAS_JRPIN5
#define DEVICE_HAS_SERIAL_OR_COM // hold 5-way in down direction at boot to enable CLI
#define DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL2
#define DEVICE_HAS_ESP_WIFI_BRIDGE_CONFIGURE

#ifdef ARDUINO_ARCH_RP2350  // Pico 2 W has CYW43439
#define DEVICE_HAS_WIFI_NATIVE
#endif
#define DEVICE_HAS_I2C_DISPLAY
#define DEVICE_HAS_FIVEWAY


//-- UARTS
// UARTB = serial port
// UARTC = COM (CLI)
// UARTD = serial2 BT/ESP port
// UART = JR pin5
// UARTF = debug port

#define UARTB_USE_SERIAL          // serial via USB

#define UARTC_USE_SERIAL          // COM (CLI) via USB

#define UARTD_USE_SERIAL1         // serial2 BT/ESP
#define UARTD_BAUD                115200
#define UARTD_TX_PIN              IO_P0
#define UARTD_RX_PIN              IO_P1
#define UARTD_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTD_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UART_USE_PIO_HALF_DUPLEX  // JR Pin5 UART
#define UART_BAUD                 400000
#define UART_TXBUFSIZE            512
#define UART_RXBUFSIZE            512
#define UART_TX_PIN               IO_P7

#define UARTF_USE_SERIAL2         // debug port
#define UARTF_BAUD                115200
#define UARTF_TXBUFSIZE           1024
#define UARTF_TX_PIN              IO_P8


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
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU);
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


//-- Button

#define BUTTON                    IO_P22

void button_init(void) { gpio_init(BUTTON, IO_MODE_INPUT_PU); }
bool button_pressed(void) { return gpio_read_activelow(BUTTON) ? true : false; }


//-- LEDs

#define LED_RED                   IO_P20

void leds_init(void)
{ 
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_LOW);
}
void led_red_off(void) { gpio_low(LED_RED); }
void led_red_on(void) { gpio_high(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }


//-- POWER

#define POWER_PA_E28_2G4M27SX
#include "../hal-power-pa.h"


//-- Display I2C

#if defined DEVICE_HAS_I2C_DISPLAY || defined DEVICE_HAS_I2C_DISPLAY_ROT180

#define I2C_USE_WIRE1             // gpio 10/11 are on i2c1
#define I2C_SDA_IO                IO_P10
#define I2C_SCL_IO                IO_P11
#define I2C_CLOCKSPEED            400000L
#define I2C_BUFFER_SIZE           1024


//-- 5 Way Switch
// single-wire analog joystick using resistor ladder on GP27 (ADC1)

#define FIVEWAY_ADC_IO            IO_P27  // GP27 = ADC1/A1

// thresholds based on measured ADC values (midpoints between adjacent buttons)
// down ~0, right ~500, up ~1100, left ~1800, center ~2450, none ~4000
#define KEY_DOWN_THRESH_HIGH      250
#define KEY_RIGHT_THRESH_LOW      251
#define KEY_RIGHT_THRESH_HIGH     800
#define KEY_UP_THRESH_LOW         801
#define KEY_UP_THRESH_HIGH        1450
#define KEY_LEFT_THRESH_LOW       1451
#define KEY_LEFT_THRESH_HIGH      2125
#define KEY_CENTER_THRESH_LOW     2126
#define KEY_CENTER_THRESH_HIGH    3200

void fiveway_init(void)
{
    analogReadResolution(12);                     // 12-bit ADC: 0-4095
    analogRead(FIVEWAY_ADC_IO);                   // trigger adc_gpio_init (disables pulls)
    gpio_set_pulls(FIVEWAY_ADC_IO, true, false);  // re-enable pull-up after arduino init
}

uint16_t fiveway_adc_read(void)
{
    return analogRead(FIVEWAY_ADC_IO);
}

uint8_t fiveway_read(void)
{
    uint16_t adc = fiveway_adc_read();

    // return bitmask format: (1 << KEY_xxx) for display code compatibility
    if (adc <= KEY_DOWN_THRESH_HIGH) return (1 << KEY_DOWN);
    if (adc >= KEY_RIGHT_THRESH_LOW && adc <= KEY_RIGHT_THRESH_HIGH) return (1 << KEY_RIGHT);
    if (adc >= KEY_UP_THRESH_LOW && adc <= KEY_UP_THRESH_HIGH) return (1 << KEY_UP);
    if (adc >= KEY_LEFT_THRESH_LOW && adc <= KEY_LEFT_THRESH_HIGH) return (1 << KEY_LEFT);
    if (adc >= KEY_CENTER_THRESH_LOW && adc <= KEY_CENTER_THRESH_HIGH) return (1 << KEY_CENTER);

    return 0; // no button pressed
}

#endif


//-- Serial or Com Switch
// use com if FIVEWAY is DOWN during power up, else use serial

#ifdef DEVICE_HAS_SERIAL_OR_COM
bool tx_ser_or_com_serial = true; // we use serial as default

void ser_or_com_init(void)
{
    // fiveway_init already set up pull-up after adc_gpio_init, so analogRead works correctly
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 16; i++) {
        uint16_t adc = fiveway_adc_read();
        if (adc <= KEY_DOWN_THRESH_HIGH) cnt++;
    }
    tx_ser_or_com_serial = !(cnt > 8);
}

bool ser_or_com_serial(void) { return tx_ser_or_com_serial; }

void ser_or_com_set_to_com(void) { tx_ser_or_com_serial = false; }

#endif // DEVICE_HAS_SERIAL_OR_COM


//-- ESP WiFi Bridge

#define ESP_RESET                 IO_P12
#define ESP_GPIO0                 IO_P13

#ifdef DEVICE_HAS_ESP_WIFI_BRIDGE_ON_SERIAL2

void esp_init(void)
{
    gpio_init(ESP_GPIO0, IO_MODE_OUTPUT_PP_HIGH); // high -> esp runs normally
    gpio_init(ESP_RESET, IO_MODE_OUTPUT_PP_LOW);  // low -> esp is in reset
}

void esp_reset_high(void) { gpio_high(ESP_RESET); }
void esp_reset_low(void) { gpio_low(ESP_RESET); }
void esp_gpio0_high(void) { gpio_high(ESP_GPIO0); }
void esp_gpio0_low(void) { gpio_low(ESP_GPIO0); }

#endif


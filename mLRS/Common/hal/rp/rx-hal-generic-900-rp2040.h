//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
// 2026-01-28
//********************************************************

//-------------------------------------------------------
// RP2040, GENERIC 900 RX
//-------------------------------------------------------

#define DEVICE_HAS_SINGLE_LED
//#define DEVICE_HAS_NO_DEBUG  // Commented out for debugging
#define USE_DEBUG  // Enable debug output via USB Serial


//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_SERIAL
#define UARTB_BAUD                RX_SERIAL_BAUDRATE
#define UARTB_TXBUFSIZE           RX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           RX_SERIAL_RXBUFSIZE

#define UARTF_USE_SERIAL1
#define UARTF_BAUD                115200
#define UARTF_TX_PIN              IO_P0  // GPIO 0 = TX for debug
#define UARTF_RX_PIN              IO_P1  // GPIO 1 = RX for debug


//-- SX1: SX12xx & SPI
// SPI0 default pins: SCK=18, MOSI=19, MISO=16

#define SPI_MISO                  IO_P16
#define SPI_MOSI                  IO_P19
#define SPI_SCK                   IO_P18
#define RX_SPI_NSS                IO_P17
#define SPI_FREQUENCY             1000000L  // reduced from 10 MHz for SPI debug
#define SX_RESET                  IO_P3
#define SX_BUSY                   IO_P2
#define SX_DIO1                   IO_P4
#define SX_TX_EN                  IO_P5
#define SX_RX_EN                  IO_P6

//#define SX126X_USE_DCDC           true
// E22-900M30S uses TCXO at 1.8V (default) - do NOT define SX_USE_CRYSTALOSCILLATOR

IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH);
    gpio_init(SX_DIO1, IO_MODE_INPUT_PD);
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU);
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW);

    // Debug: verify GPIO states after init
    Serial1.print("  GPIO after init: RESET=");
    Serial1.print(digitalRead(SX_RESET));
    Serial1.print(" BUSY=");
    Serial1.print(digitalRead(SX_BUSY));
    Serial1.print(" DIO1=");
    Serial1.println(digitalRead(SX_DIO1));
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

void sx_amp_off(void)
{
    gpio_low(SX_TX_EN);
    gpio_low(SX_RX_EN);
}

void sx_dio_init_exti_isroff(void) {}

void sx_dio_enable_exti_isr(void)
{
    Serial1.print("  sx_dio_enable_exti_isr: attaching interrupt to DIO1 pin ");
    Serial1.println(SX_DIO1);
    attachInterrupt(digitalPinToInterrupt(SX_DIO1), SX_DIO_EXTI_IRQHandler, RISING);
    Serial1.println("  sx_dio_enable_exti_isr: interrupt attached");
}

void sx_dio_exti_isr_clearflag(void) {}

// Debug helper to check DIO1 state
inline bool sx_dio1_read(void) {
    return (digitalRead(SX_DIO1) != LOW);
}

bool sx_busy_read(void)
{
    return (digitalRead(SX_BUSY) != LOW);
}


//-- Button

#define BUTTON                    IO_P22

void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU);
}

bool button_pressed(void)
{
    return gpio_read_activelow(BUTTON) ? true : false;
}


//-- LEDs

#define LED_RED                   IO_P25

void leds_init(void)
{
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_LOW);
}

void led_red_off(void) { gpio_low(LED_RED); }
void led_red_on(void) { gpio_high(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }


//-- EEPROM

#define EE_PAGE_SIZE              4096
#define EE_START_PAGE             0
#define EE_USE_WORD


//-- POWER

#define POWER_GAIN_DBM            0 // gain of a PA stage if present
#define POWER_SX126X_MAX_DBM      SX126X_POWER_MAX // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_14_DBM, .mW = 25 },
    { .dbm = POWER_17_DBM, .mW = 50 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_22_DBM, .mW = 158 },
};

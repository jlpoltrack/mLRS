//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************

//-------------------------------------------------------
// RP2354A, RP-mLRS Dual RX, NiceRF LR2021
//-------------------------------------------------------
// CAN build: can2040 + SN65HVD230, CANFD build: MCP251863

#define DEVICE_HAS_OUT


//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_SERIAL2
#define UARTB_BAUD                RX_SERIAL_BAUDRATE
#define UARTB_TXBUFSIZE           RX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           RX_SERIAL_RXBUFSIZE
#define UARTB_TX_PIN              IO_P20
#define UARTB_RX_PIN              IO_P21

#define UART_USE_SERIAL1
#define UART_BAUD                 416666
#define UART_TXBUFSIZE            1024
#define UART_TX_PIN               IO_P28

#define UARTF_USE_SERIAL          // USB serial


//-- SX1: LR20xx & SPI

#define SPI_USE_SPI1
#define SPI_MISO                  IO_P8
#define SPI_MOSI                  IO_P11
#define SPI_SCK                   IO_P14
#define SPI_CS_IO                 IO_P13
#define SPI_FREQUENCY             16000000L  // 16 MHz max per datasheet

#define SX_RESET                  IO_P7
#define SX_BUSY                   IO_P12
#define SX_DIO                    IO_P6 // IRQ on the module, DIO9 on LR2021F33

#define SX_USE_TCXO_VOLTAGE       LR20XX_TCXO_SUPPLY_VOLTAGE_3_3

#define SX_USE_IRQ_DIO_NO         LR20XX_DIO_9

#define SX_USE_RFSW_DIO_CONFIG  {\
    { LR20XX_DIO_5, LR20XX_DIO_RF_SWITCH_CONFIG_RX_HF }, \
    { LR20XX_DIO_6, LR20XX_DIO_RF_SWITCH_CONFIG_TX_LF }, \
    { LR20XX_DIO_7, LR20XX_DIO_RF_SWITCH_CONFIG_RX_HF | LR20XX_DIO_RF_SWITCH_CONFIG_TX_HF }, \
    { LR20XX_DIO_8, LR20XX_DIO_RF_SWITCH_CONFIG_TX_HF } }


IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH);
    gpio_init(SX_DIO, IO_MODE_INPUT_PD);
    gpio_init(SX_BUSY, IO_MODE_INPUT_PU);
}

bool sx_busy_read(void) { return (gpio_read_activehigh(SX_BUSY)) ? true : false; }

void sx_amp_transmit(void) {}
void sx_amp_receive(void) {}

void sx_band(bool high_band) {} // band switching is done by the LR20xx DIO rf switch config

void sx_dio_init_exti_isroff(void) { detachInterrupt(SX_DIO); }
void sx_dio_enable_exti_isr(void) { attachInterrupt(digitalPinToInterrupt(SX_DIO), SX_DIO_EXTI_IRQHandler, RISING); }
void sx_dio_exti_isr_clearflag(void) {}


#ifdef DEVICE_HAS_DRONECAN_FD
//-- CAN bus (DroneCAN via MCP251863 CAN FD controller on SPI0)

#define CAN_SPI_NUM               0
#define CAN_SPI_SCK               IO_P2
#define CAN_SPI_MOSI              IO_P3
#define CAN_SPI_MISO              IO_P4
#define CAN_SPI_CS_IO             IO_P5
#define CAN_SPI_FREQUENCY         16000000L  // 17 MHz max with the 40 MHz crystal
#define CAN_INT                   IO_P1  // INT pin (main interrupt), INT0 drives XSTBY

#else
//-- CAN bus (DroneCAN via can2040 PIO)

#define CAN_RX_PIN                IO_P1
#define CAN_TX_PIN                IO_P4

#endif


//-- Out port

void out_init_gpio(void) {}
void out_set_normal(void) { gpio_set_outover(UART_TX_PIN, GPIO_OVERRIDE_NORMAL); }
void out_set_inverted(void) { gpio_set_outover(UART_TX_PIN, GPIO_OVERRIDE_INVERT); }


//-- Button

#define BUTTON                    IO_P0  // BOOT button, also on QSPI_SS, never drive it

void button_init(void) { gpio_init(BUTTON, IO_MODE_INPUT_PU); }
bool button_pressed(void) { return gpio_read_activelow(BUTTON); }


//-- LEDs

#define LED_GREEN                 IO_P19
#define LED_RED                   IO_P29

void leds_init(void)
{
    gpio_init(LED_GREEN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_LOW);
}

void led_green_off(void) { gpio_low(LED_GREEN); }
void led_green_on(void) { gpio_high(LED_GREEN); }
void led_green_toggle(void) { gpio_toggle(LED_GREEN); }

void led_red_off(void) { gpio_low(LED_RED); }
void led_red_on(void) { gpio_high(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }


//-- POWER

#define POWER_PA_NICERF_LR2021F33
#include "../hal-power-pa.h"

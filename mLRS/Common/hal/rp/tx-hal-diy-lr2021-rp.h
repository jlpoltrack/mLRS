//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************

//-------------------------------------------------------
// RP2040/RP2350, DIY LR2021 TX
//-------------------------------------------------------

#define DEVICE_HAS_SINGLE_LED_RGB
#define DEVICE_HAS_JRPIN5
#define DEVICE_HAS_NO_COM


//-- UARTS
// UARTB = serial port
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
#define UART_TX_PIN               IO_P9

#define UARTF_USE_SERIAL2         // debug port
#define UARTF_BAUD                115200
#define UARTF_TXBUFSIZE           1024
#define UARTF_TX_PIN              IO_P4


//-- SX1: LR20xx & SPI
// SPI1 pins: SCK=10, MOSI=11, MISO=12

#define SPI_USE_SPI1
#define SPI_MISO                  IO_P12
#define SPI_MOSI                  IO_P11
#define SPI_SCK                   IO_P10
#define SPI_CS_IO                 IO_P13
#define SPI_FREQUENCY             16000000L  // 16 MHz max per datasheet

#define SX_RESET                  IO_P7
#define SX_BUSY                   IO_P8
#define SX_DIO1                   IO_P6 // labeled IRQ on board, DIO9 on LR2021F33

#define SX_USE_TCXO_VOLTAGE       LR20XX_TCXO_SUPPLY_VOLTAGE_3_3

#define SX_USE_IRQ_DIO_NO         LR20XX_DIO_9

#define SX_USE_RFSW_DIO_NOS      { LR20XX_DIO_5, LR20XX_DIO_6, LR20XX_DIO_7, LR20XX_DIO_8 }
#define SX_USE_RFSW_DIO_CONFIGS  { LR20XX_DIO_RF_SWITCH_CONFIG_RX_HF, \
                                  LR20XX_DIO_RF_SWITCH_CONFIG_TX_LF, \
                                  LR20XX_DIO_RF_SWITCH_CONFIG_RX_HF | LR20XX_DIO_RF_SWITCH_CONFIG_TX_HF, \
                                  LR20XX_DIO_RF_SWITCH_CONFIG_TX_HF }


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


//-- Button

#define BUTTON                    IO_P24  // usr button on YD-RP2040 clone board

void button_init(void) { gpio_init(BUTTON, IO_MODE_INPUT_PU); }
bool button_pressed(void) { return gpio_read_activelow(BUTTON) ? true : false; }


//-- LEDs

#define LED_RGB                   IO_P23  // RGB LED on YD-RP2040 clone board
#define LED_RGB_PIXEL_NUM         1
#include "../rp-hal-led-rgb.h"


//-- POWER

#include "../../setup_types.h" // needed for frequency band condition in rfpower calc

// sx_power is in LR20xx register units of 0.5 dBm, validated with IRC power meter
void lr20xx_rfpower_calc(const int8_t power_dbm, int8_t* sx_power, int8_t* actual_power_dbm, const uint8_t frequency_band)
{
    if (frequency_band == SX_FHSS_FREQUENCY_BAND_2P4_GHZ) {
        if (power_dbm >= POWER_30_DBM) {
            *sx_power = 8;
            *actual_power_dbm = 30;
        } else if (power_dbm >= POWER_27_DBM) {
            *sx_power = -4;
            *actual_power_dbm = 27;
        } else if (power_dbm >= POWER_24_DBM) {
            *sx_power = -13;
            *actual_power_dbm = 24;
        } else if (power_dbm >= POWER_20_DBM) {
            *sx_power = -23;
            *actual_power_dbm = 20;
        } else if (power_dbm >= POWER_17_DBM) {
            *sx_power = -28;
            *actual_power_dbm = 17;
        } else if (power_dbm >= POWER_14_DBM) {
            *sx_power = -39;
            *actual_power_dbm = 14;
        } else {
            *sx_power = -39;
            *actual_power_dbm = 10;  // about 12 dBm
        }
    } else { // measured at using 915 FCC band
        if (power_dbm >= POWER_30_DBM) {
            *sx_power = 44;
            *actual_power_dbm = 30;
        } else if (power_dbm >= POWER_27_DBM) {
            *sx_power = 29;
            *actual_power_dbm = 27;
        } else if (power_dbm >= POWER_24_DBM) {
            *sx_power = 21;
            *actual_power_dbm = 24;
        } else if (power_dbm >= POWER_20_DBM) {
            *sx_power = 12;
            *actual_power_dbm = 20;
        } else if (power_dbm >= POWER_17_DBM) {
            *sx_power = 5;
            *actual_power_dbm = 17;
        } else if (power_dbm >= POWER_14_DBM) {
            *sx_power = -1;
            *actual_power_dbm = 14;
        } else {
            *sx_power = -9;
            *actual_power_dbm = 10;
        }
    }
}

#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_14_DBM, .mW = 25 },
    { .dbm = POWER_17_DBM, .mW = 50 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};

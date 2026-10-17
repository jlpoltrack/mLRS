//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// ESP32, DIY Rx LR2021 2400 & 900, True Diversity
//-------------------------------------------------------
// LR2021 DIOs as ELRS default: DIO5 = 2.4 TX, DIO6 = 2.4 RX, DIO7 = 900 TX, DIO8 = 900 RX, DIO9 = IRQ, DIO10/11 = low

#define DEVICE_HAS_SINGLE_LED_RGB
#define DEVICE_HAS_DIVERSITY_SINGLE_SPI // must be set, doesn't work without it
#define DEVICE_HAS_NO_DEBUG
//#define DEVICE_HAS_NO_SERIAL


//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_SERIAL
#define UARTB_BAUD                RX_SERIAL_BAUDRATE
#define UARTB_TXBUFSIZE           RX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           RX_SERIAL_RXBUFSIZE

#define UARTF_USE_SERIAL
#define UARTF_BAUD                115200


//-- SX1: LR20xx & SPI

#define SPI_CS_IO                 IO_P27
#define SPI_MISO                  IO_P33
#define SPI_MOSI                  IO_P32
#define SPI_SCK                   IO_P25
#define SPI_FREQUENCY             16000000L  // 16 MHz max per datasheet
#define SX_RESET                  IO_P26
#define SX_DIO                    IO_P37
#define SX_BUSY                   IO_P36

#define SX_USE_IRQ_DIO_NO         LR20XX_DIO_9 // also used by SX2
#define SX_USE_REGULATOR_MODE_DCDC // also used by SX2
#define SX_USE_RFSW_DIO_CONFIG    { \
    { LR20XX_DIO_5, LR20XX_DIO_RF_SWITCH_CONFIG_TX_HF }, \
    { LR20XX_DIO_6, LR20XX_DIO_RF_SWITCH_CONFIG_RX_HF }, \
    { LR20XX_DIO_7, LR20XX_DIO_RF_SWITCH_CONFIG_TX_LF }, \
    { LR20XX_DIO_8, LR20XX_DIO_RF_SWITCH_CONFIG_RX_LF }, \
    { LR20XX_DIO_10, 0 }, \
    { LR20XX_DIO_11, 0 }, \
}

void sx_band(bool high_band) {}


//-- SX2: LR20xx & SPI

#define SX2_CS_IO                 IO_P13
#define SX2_RESET                 IO_P21
#define SX2_DIO                   IO_P34
#define SX2_BUSY                  IO_P39

void sx2_band(bool high_band) {}


//-- Button

#define BUTTON                    IO_P0


//-- LEDs

#define LED_RGB                   IO_P22
#define LED_RGB_PIXEL_NUM         1
#include "esp-hal-led-rgb.h"


//-- POWER

#include "../../setup_types.h" // needed for frequency band condition in rfpower calc

// LR20xx power unit is 0.5 dBm, values are ELRS hardware config corrected by measurement
// 900 MHz has no PA
void lr20xx_rfpower_calc(const int8_t power_dbm, int8_t* sx_power, int8_t* actual_power_dbm, const uint8_t frequency_band)
{
    if (frequency_band == SX_FHSS_FREQUENCY_BAND_2P4_GHZ) {
        if (power_dbm >= POWER_20_DBM) { // -> 20
            *sx_power = 6;
            *actual_power_dbm = 20;
        } else if (power_dbm >= POWER_17_DBM) { // -> 17
            *sx_power = -1;
            *actual_power_dbm = 17;
        } else if (power_dbm >= POWER_14_DBM) { // -> 14
            *sx_power = -7;
            *actual_power_dbm = 14;
        } else {
            *sx_power = -15;
            *actual_power_dbm = 10;
        }
    } else {
        if (power_dbm >= POWER_20_DBM) { // -> 20
            *sx_power = 40;
            *actual_power_dbm = 20;
        } else if (power_dbm >= POWER_17_DBM) { // -> 17
            *sx_power = 33;
            *actual_power_dbm = 17;
        } else if (power_dbm >= POWER_14_DBM) { // -> 14
            *sx_power = 27;
            *actual_power_dbm = 14;
        } else {
            *sx_power = 19;
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
};

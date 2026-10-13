//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// ESP32, ELRS GENERIC 2400 True Diversity PA RX
//-------------------------------------------------------

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


//-- SX1: SX12xx & SPI

#define SPI_CS_IO                 IO_P27
#define SPI_MISO                  IO_P33
#define SPI_MOSI                  IO_P32
#define SPI_SCK                   IO_P25
#define SPI_FREQUENCY             18000000L
#define SX_RESET                  IO_P26
#define SX_DIO                    IO_P37
#define SX_BUSY                   IO_P36
#define SX_RX_EN                  IO_P10
#define SX_TX_EN                  IO_P14


//-- SX2: SX12xx & SPI

#define SX2_CS_IO                 IO_P13
#define SX2_RESET                 IO_P21
#define SX2_DIO                   IO_P34
#define SX2_BUSY                  IO_P39
#define SX2_RX_EN                 IO_P9
#define SX2_TX_EN                 IO_P15


//-- Button

#define BUTTON                    IO_P0


//-- LEDs

#define LED_RGB                   IO_P22
#define LED_RGB_PIXEL_NUM         1
#include "esp-hal-led-rgb.h"


//-- POWER
#ifndef POWER_OVERLAY

#define POWER_GAIN_DBM            18 // gain of a PA stage if present
#define POWER_SX1280_MAX          SX1280_POWER_3_DBM  // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_0_DBM, .mW =  1 },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_20_DBM, .mW = 100 },
};

#endif // !POWER_OVERLAY

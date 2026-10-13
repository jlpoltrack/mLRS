//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// ESP8285, ELRS RADIOMASTER BR3 900 RX
//-------------------------------------------------------

#define DEVICE_HAS_SINGLE_LED
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
#define SPI_CS_IO                 IO_P15
#define SPI_FREQUENCY             10000000L
#define SX_RESET                  IO_P2
#define SX_DIO                    IO_P4
#define SX_TX_EN                  IO_P10
#define SX_ANT                    IO_P9

#define SX_USE_RFO


//-- Button
#define BUTTON                    IO_P0


//-- LEDs
#define LED_RED                   IO_P16


//-- POWER
#define POWER_GAIN_DBM            19 // gain of a PA stage if present
#define POWER_SX1276_MAX          SX1276_OUTPUT_POWER_MAX // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },  // really about 21 dBm
    { .dbm = POWER_27_DBM, .mW = 500 },    // 27 dBm
};

//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//*******************************************************

/*
  Info on DIP switches

  1,2 on:   update firmware on main ESP32, USB is connected to UARTO 
  3,4 on:   normal operation mode, USB isn't used, UARTO connected to ESP8285
  5,6,7 on: update firmware on ESP8285, USB is connected to ESP8285's UART

  Flashing ESP8285:
  - Board: Generic ESP8266 Module
  - Upload Speed: can be 921600
  - Reset Method: dtr (aka modemcu)
*/

//-------------------------------------------------------
// ESP32, BetaFpv Micro 1W TX, SX1280 2400
//-------------------------------------------------------
// https://github.com/ExpressLRS/targets/blob/master/TX/BETAFPV%202400%20Micro%201W.json

#define DEVICE_HAS_JRPIN5
#define DEVICE_HAS_IN
#define DEVICE_HAS_COM_ON_SERIAL // hold 5-way in down direction at boot to enable com (CLI)
#define DEVICE_HAS_NO_DEBUG
#define DEVICE_HAS_SINGLE_LED_RGB
#define DEVICE_HAS_I2C_DISPLAY_ROT180
#define DEVICE_HAS_FAN_ONOFF

// Note on SERIAL_OR_COM:
// The com uart is not initialized, the serial uart is, So, buffers are set as by the RX/TXBUFSIZE defines for serial.
// The TXBUFSIZE setting for the com affects however the CLI's chunkenizer behavior.


//-- UARTS
// UARTB = serial port
// UARTC (or USB) = com (CLI) port
// UARTD = serial2 port or wireless bridge port
// UART  = JR bay pin5
// UARTE = in port, SBus or whatever
// UARTF or SWUART = debug port

#define UARTB_USE_SERIAL // serial or com
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX_IO           IO_P1
#define UARTB_USE_RX_IO           IO_P3
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

/* #define UARTC_USE_SERIAL // com
#define UARTC_BAUD                115200
#define UARTC_USE_TX_IO           IO_P1
#define UARTC_USE_RX_IO           IO_P3
#define UARTC_TXBUFSIZE           0  // TX FIFO = 128
#define UARTC_RXBUFSIZE           TX_COM_RXBUFSIZE */

#define UART_USE_SERIAL1 // JR bay pin5
#define UART_BAUD                 400000
#define UART_USE_TX_IO            IO_P13
#define UART_USE_RX_IO            IO_P13
#define UART_TXBUFSIZE            0  // TX FIFO = 128
#define UART_RXBUFSIZE            0  // RX FIFO = 128 + 1

#define UARTE_USE_SERIAL1 // in port, uses JRPin5
#define UARTE_BAUD                 100000
#define UARTE_USE_TX_IO            -1
#define UARTE_USE_RX_IO            IO_P13
#define UARTE_RXBUFSIZE            0 // RX FIFO = 128 + 1


//-- SX1: SX12xx & SPI

#define SPI_CS_IO                 IO_P5
#define SPI_MISO                  IO_P19
#define SPI_MOSI                  IO_P23
#define SPI_SCK                   IO_P18
#define SPI_FREQUENCY             18000000L
#define SX_RESET                  IO_P14
#define SX_DIO                    IO_P4
#define SX_BUSY                   IO_P21
#define SX_TX_EN                  IO_P26
#define SX_RX_EN                  IO_P27


//-- LEDs

#define LED_RGB                   IO_P16
#define LED_RGB_PIXEL_NUM         1
#include "esp-hal-led-rgb.h"


//-- Display I2C

#define I2C_SDA_IO                IO_P22
#define I2C_SCL_IO                IO_P32
#define I2C_CLOCKSPEED            1000000L  // fix - rather too much, but helps with LQ, ESP32 max speed


//-- 5 Way Switch

#define FIVEWAY_ADC_IO            IO_P25
#define KEY_RIGHT_THRESH          3511 // 3600 // tom = ELRS: 3511, ow: 3712
#define KEY_UP_THRESH             2839 // 2800 // tom = ELRS: 2839, ow: 2966, marc: 2800 didn't work for him. 2839 did
#define KEY_DOWN_THRESH           2191 // 2200 // tom = ELRS: 2191, ow: 2282
#define KEY_LEFT_THRESH           1616 // 1650 // tom = ELRS: 1616, ow: 1685
#define KEY_CENTER_THRESH         0


//-- Serial or Com Switch
// use com if FIVEWAY is DOWN during power up, else use serial
// FIVEWAY-DONW becomes bind button later on

#ifdef DEVICE_HAS_COM_ON_SERIAL
bool ser_or_com_init(void) // return true if is_serial
{
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 16; i++) {
        uint16_t adc = analogRead(FIVEWAY_ADC_IO);
        if (adc > (KEY_DOWN_THRESH-250) && adc < (KEY_DOWN_THRESH+250)) cnt++;
    }
    return !(cnt > 8);
}
#endif


//-- Cooling Fan

#define FAN_IO                    IO_P17


//-- POWER

#define POWER_GAIN_DBM            32 // 28 // gain of a PA stage if present
#define POWER_SX1280_MAX          SX1280_POWER_3_DBM  // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_17_DBM, .mW = 50 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};

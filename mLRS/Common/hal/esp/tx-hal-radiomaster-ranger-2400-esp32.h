//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//*******************************************************

/*
  Flashing ESP8285:
  - change ser dest to serial2
  - change ser baudrate to 115200
  - put Tx module into FLASH_ESP mode from tools
  - Board: Generic ESP8266 Module
  - Upload Speed: 115200
  - Reset Method: no dtr (aka ck)
*/

//-------------------------------------------------------
// ESP32, Radiomaster Tx Ranger, SX1280 2400
//-------------------------------------------------------
// Ranger, "big" Ranger: https://github.com/ExpressLRS/targets/blob/master/TX/Radiomaster%20Ranger.json
// This could probably be expanded to also support Ranger Micro: https://github.com/ExpressLRS/targets/blob/master/TX/Radiomaster%20Ranger%20Micro.json

#define DEVICE_HAS_JRPIN5
#define DEVICE_HAS_COM_ON_SERIAL // hold 5-way in down direction at boot to enable com (CLI)
#define DEVICE_HAS_IN
#define DEVICE_HAS_NO_DEBUG
//#define DEVICE_HAS_NO_SERIAL
//#define DEVICE_HAS_NO_COM
#define DEVICE_HAS_I2C_DISPLAY
#define DEVICE_HAS_SINGLE_LED_RGB
#define DEVICE_HAS_FAN_ONOFF
#define DEVICE_HAS_ESP_WIFI_BRIDGE_ESP8266 // board has an ESP8285 wireless bridge with GPIO,RST
#define DEVICE_HAS_ESP_WIFI_BRIDGE_CONFIGURE

// Note on SERIAL_OR_COM:
// The com uart is not initialized, the serial uart is. So, buffers are set as by the RX/TXBUFSIZE defines for serial.
// The TXBUFSIZE setting for the com affects however the CLI's chunkenizer behavior.


//-- UARTS
// UARTB = serial port
// UARTC (or USB) = com (CLI) port
// UARTD = serial2 port or wireless bridge port
// UART  = JR bay pin5
// UARTE = in port, SBus or whatever
// UARTF or SWUART = debug port

#define UARTB_USE_SERIAL // serial or com, is connected to USB-C via USB<>UART
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX_IO           IO_P1
#define UARTB_USE_RX_IO           IO_P3
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

/* #define UARTC_USE_SERIAL // com, is connected to USB-C via USB<>UART
#define UARTC_BAUD                115200
#define UARTC_USE_TX_IO           IO_P1
#define UARTC_USE_RX_IO           IO_P3
#define UARTC_TXBUFSIZE           0 // TX FIFO = 128
#define UARTC_RXBUFSIZE           TX_COM_RXBUFSIZE */

#define UARTD_USE_SERIAL2 // serial2 or wireless bridge
#define UARTD_BAUD                115200
#define UARTD_USE_TX_IO           IO_P17
#define UARTD_USE_RX_IO           IO_P16
#define UARTD_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTD_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UART_USE_SERIAL1 // JR bay pin5
#define UART_BAUD                 400000
#define UART_USE_TX_IO            IO_P13
#define UART_USE_RX_IO            IO_P13
#define UART_TXBUFSIZE            0 // TX FIFO = 128
#define UART_RXBUFSIZE            0 // RX FIFO = 128 + 1

#define UARTE_USE_SERIAL1 // in port, uses JRPin5
#define UARTE_BAUD                 100000
#define UARTE_USE_TX_IO            -1
#define UARTE_USE_RX_IO            IO_P13
#define UARTE_RXBUFSIZE            0 // RX FIFO = 128 + 1

#define UARTF_USE_SERIAL // debug, if needed, debug on USB; need to disable serial and com
#define UARTF_BAUD                115200
#define UARTF_USE_TX_IO           IO_P1
#define UARTF_USE_RX_IO           -1
#define UARTF_TXBUFSIZE           512


//-- SX1: SX12xx & SPI

#define SPI_CS_IO                 IO_P4
#define SPI_MISO                  IO_P19
#define SPI_MOSI                  IO_P23
#define SPI_SCK                   IO_P18
#define SPI_FREQUENCY             18000000L
#define SX_RESET                  IO_P5
#define SX_DIO                    IO_P21
#define SX_BUSY                   IO_P22
#define SX_TX_EN                  IO_P33
#define SX_RX_EN                  IO_P32

#define SX_USE_REGULATOR_MODE_DCDC


//-- LEDs

#define LED_RGB                   IO_P15
#define LED_RGB_PIXEL_NUM         6
#include "esp-hal-led-rgb.h"


//-- Display I2C

#define I2C_SDA_IO                IO_P14
#define I2C_SCL_IO                IO_P12
#define I2C_CLOCKSPEED            1000000L  // fix - rather too much, but helps with LQ, ESP32 max speed


//-- 5 Way Switch

#define FIVEWAY_ADC_IO            IO_P39
#define KEY_UP_THRESH             3230
#define KEY_DOWN_THRESH           0
#define KEY_LEFT_THRESH           1890
#define KEY_RIGHT_THRESH          2623
#define KEY_CENTER_THRESH         1205


//-- Serial or Com Switch
// use com if FIVEWAY is DOWN during power up, else use serial
// FIVEWAY-DONW becomes bind button later on

#ifdef DEVICE_HAS_COM_ON_SERIAL
bool ser_or_com_init(void) // return true if is_serial
{
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 16; i++) {
        int16_t adc = analogRead(FIVEWAY_ADC_IO);
        if (adc > (KEY_DOWN_THRESH-200) && adc < (KEY_DOWN_THRESH+200)) cnt++;
    }
    return !(cnt > 8);
}
#endif


//-- Cooling Fan

#define FAN_IO                    IO_P2


//-- ESP8285 Wifi Bridge

#define ESP_RESET                 IO_P25 // backpack_en
#define ESP_GPIO0                 IO_P26 // backpack_boot, seems to be inverted


//-- POWER

#define POWER_GAIN_DBM            33 // gain of a PA stage if present
#define POWER_SX1280_MAX          SX1280_POWER_0_DBM  // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_17_DBM, .mW = 50 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};

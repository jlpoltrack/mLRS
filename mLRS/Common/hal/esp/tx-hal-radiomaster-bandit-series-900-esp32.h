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
// ESP32, Radiomaster Bandit Tx, Bandit Micro TX, SX1276 900
//-------------------------------------------------------
// Bandit, "big" Bandit: https://github.com/ExpressLRS/targets/blob/master/TX/Radiomaster%20Bandit.json
// Bandit Micro: https://github.com/ExpressLRS/targets/blob/master/TX/Radiomaster%20Bandit%20Micro.json

#define DEVICE_HAS_JRPIN5
#define DEVICE_HAS_IN
#define DEVICE_HAS_FAN_ONOFF

#ifdef TX_ELRS_RADIOMASTER_BANDIT_900_ESP32 // Bandit, "big" Bandit
#define DEVICE_HAS_SINGLE_LED_RGB
#define DEVICE_HAS_I2C_DISPLAY
#else // Bandit Micro
#define DEVICE_HAS_SINGLE_LED
#define DEVICE_HAS_I2C_DISPLAY_ROT180
#endif

#define DEVICE_HAS_COM_ON_SERIAL // hold 5-way in down direction at boot to enable com (CLI)
#define DEVICE_HAS_ESP_WIFI_BRIDGE_ESP8266 // board has an ESP8285 wireless bridge with GPIO,RST
#define DEVICE_HAS_ESP_WIFI_BRIDGE_CONFIGURE // requires backpack firmware to have AT mode enabled
#define DEVICE_HAS_NO_DEBUG
//#define DEVICE_HAS_NO_SERIAL
//#define DEVICE_HAS_NO_COM

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

#define UARTB_USE_SERIAL // serial or com, is connected to USB-C via USB<>UART also RT connector
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX_IO           IO_P1
#define UARTB_USE_RX_IO           IO_P3
#define UARTB_TXBUFSIZE           TX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

/* #define UARTC_USE_SERIAL // com, is connected to USB-C via USB<>UART also RT connector
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

#define UARTF_USE_SERIAL // debug, if needed, debug is either on the "RT" pad or on USB; need to disable serial and com
#define UARTF_BAUD                115200
#define UARTF_USE_TX_IO           IO_P1
#define UARTF_USE_RX_IO           -1
#define UARTF_TXBUFSIZE           512


//-- SX1: SX12xx & SPI

#define SPI_CS_IO                 IO_P4
#define SPI_MISO                  IO_P19
#define SPI_MOSI                  IO_P23
#define SPI_SCK                   IO_P18
#define SPI_FREQUENCY             10000000L
#define SX_RESET                  IO_P5
#define SX_DIO                    IO_P22
#define SX_TX_EN                  IO_P33

#define SX_USE_RFO


//-- LEDs

// Big Bandit has RGB LEDs, Bandit Micro has a single LED
#ifdef TX_ELRS_RADIOMASTER_BANDIT_900_ESP32
    #define LED_RGB                   IO_P15
    #define LED_RGB_PIXEL_NUM         6
    #include "esp-hal-led-rgb.h"
#else

#define LED_RED                   IO_P15 // pin for both Bandit and Bandit Micro, even though they have different functionality

#endif // TX_ELRS_RADIOMASTER_BANDIT_900_ESP32


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
#define ESP_GPIO0                 IO_P32 // backpack_boot, seems to be inverted


//-- POWER

#define SX_PA_DAC_IO      IO_P26

void sx1276_rfpower_calc(const int8_t power_dbm, int8_t* sx_power, int8_t* actual_power_dbm)
{
    // jr bay: 
    //   SX1276_MAX_POWER_15_DBM:   dac = 0,   sx_power = 15 => 30.5 dBm
    //                              dac = 0,   sx_power = 0  => 27.4 dBm
    //                              dac = 190, sx_power = 0  => -25 dBm !!!
    //                              dac = 180, sx_power = 0  => 18.8 dBm
    //                              dac = 150, sx_power = 0  => 24.7 dBm
    //                              dac = 100, sx_power = 0  => 27.3 dBm
    //   SX1276_MAX_POWER_10p8_DBM: dac = 0,   sx_power = 15 => 29.4 dBm
    //                              dac = 0,   sx_power = 0  => 21.0 dBm
    //                              dac = 180, sx_power = 0  => 10.7 dBm
    //   SX1276_MAX_POWER_11p4_DBM  dac = 0,   sx_power = 0  => 27.3 dBm
    //                              dac = 100, sx_power = 15 => 29.9 dBm
    //                              dac = 0,   sx_power = 15 => 30.0 dBm
    //                              dac = 180, sx_power = 0  => 10.1 dBm
    uint8_t dac = 100;
    if (power_dbm > 28) { // -> 30
        dac = 0;
        *sx_power = 15; // equals SX1276_OUTPUT_POWER_MAX
        *actual_power_dbm = 30;
    } else if (power_dbm > 25) { // -> 27
        dac = 140;
        *sx_power = 15;
        *actual_power_dbm = 27;
    } else if (power_dbm > 22) { // -> 24
        dac = 150;
        *sx_power = 6;
        *actual_power_dbm = 24;
    } else if (power_dbm > 18) { // -> 20
        dac = 150;
        *sx_power = 0;
        *actual_power_dbm = 20;
    } else {
        dac = 180;
        *sx_power = 0;
        *actual_power_dbm = 10;
    }
    dacWrite(SX_PA_DAC_IO, dac);
}

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};

//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//*******************************************************

//-------------------------------------------------------
// ESP32, Jumper Tx, Internal SX1280 2400, good for T20 V1, T20 V2, T15, T15 Pro, T14, T-Pro S
// Backpack: Generic ESP8266 Module, define LED_IO 16
//-------------------------------------------------------
// https://github.com/ExpressLRS/targets/blob/master/TX/Jumper%20T-20%202400.json
// Added fan on GPIO 33

#define DEVICE_HAS_JRPIN5
#define DEVICE_HAS_NO_LED
#define DEVICE_HAS_FAN_ONOFF
#define DEVICE_HAS_NO_SERIAL
#define DEVICE_HAS_NO_COM
#define DEVICE_HAS_ESP_WIFI_BRIDGE_ESP8266
#define DEVICE_HAS_ESP_WIFI_BRIDGE_CONFIGURE
#define DEVICE_HAS_ESP_WIFI_BRIDGE_W_PASSTHRU_VIA_JRPIN5
#define DEVICE_HAS_NO_DEBUG


//-- UARTS
// UARTB = serial port
// UARTC (or USB) = com (CLI) port
// UARTD = serial2 port or wireless bridge port
// UART  = JR bay pin5, full duplex CRSF serial connection to radio - code still calls it JR bay pin5
// UARTE = in port, SBus or whatever
// UARTF or SWUART = debug port

#define UARTD_USE_SERIAL // serial2 or wireless bridge
#define UARTD_BAUD                TX_SERIAL_BAUDRATE
#define UARTD_USE_TX_IO           IO_P12
#define UARTD_USE_RX_IO           IO_P13
#define UARTD_TXBUFSIZE           TX_SERIAL_TXBUFSIZE // 512
#define UARTD_RXBUFSIZE           TX_SERIAL_RXBUFSIZE // 2048

#define UART_USE_SERIAL1 // full duplex CRSF/MBridge (JR pin5)
#define UART_BAUD                 400000
#define UART_USE_TX_IO            IO_P1
#define UART_USE_RX_IO            IO_P3
#define UART_TXBUFSIZE            0 // TX_SERIAL_TXBUFSIZE  // 512
#define UART_RXBUFSIZE            0 // TX_SERIAL_RXBUFSIZE  // 2048

#define JR_PIN5_FULL_DUPLEX       // internal module


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
#define PWR_EN                    IO_P25


//-- LEDs
//--


//-- Cooling Fan

#define FAN_IO                    IO_P33


//-- ESP32 Wifi Bridge

#define ESP_RESET                 IO_P15 // backpack_en
#define ESP_GPIO0                 IO_P2  // backpack_boot inverted?
#define ESP_BOOTPIN               IO_P0  // will always be IO_P0


//-- POWER

#define POWER_GAIN_DBM            28 // gain of a PA stage if present
#define POWER_SX1280_MAX          SX1280_POWER_3_DBM  // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_17_DBM, .mW = 50 },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
    { .dbm = POWER_30_DBM, .mW = 1000 },
};

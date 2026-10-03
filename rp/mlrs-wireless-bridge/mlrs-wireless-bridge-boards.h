//*******************************************************
// mLRS Wireless Bridge for RP2040/RP2350 (Pico W family)
// Copyright (c) www.olliw.eu, OlliW, OlliW42
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Modules
//*******************************************************

/*
------------------------------
Raspberry Pi Pico W
------------------------------
board: Raspberry Pi Pico W
RP2040 + CYW43439 (2.4 GHz WiFi b/g/n, BT 5.2 classic + LE)
GP0/GP1: TX0/RX0, is Serial1
GP8/GP9: TX1/RX1, is Serial2
GP2: AT mode line from the Tx module (see Connections in the sketch)
LED is on the CYW43 chip (WL_GPIO0), available as LED_BUILTIN

------------------------------
Raspberry Pi Pico 2 W
------------------------------
board: Raspberry Pi Pico 2W
RP2350 + CYW43439, pin compatible with the Pico W, uses the same pins

Notes on all modules:
- UART pins are freely mappable, but only within the pin group of the UART:
  UART0 TX: 0, 12, 16, 28   UART0 RX: 1, 13, 17, 29
  UART1 TX: 4, 8, 20, 24    UART1 RX: 5, 9, 21, 25
- the RP UARTs can invert TX/RX in hardware, so USE_SERIAL_INVERTED works on any pin
- the CYW43 is brought up by the core before setup() runs, so LED_BUILTIN just works
*/


//-------------------------------------------------------
// Module details
//-------------------------------------------------------

//-- Raspberry Pi Pico W
#if defined MODULE_RP_PICO_W
    #ifndef ARDUINO_RASPBERRY_PI_PICO_W
        #error Select board Raspberry Pi Pico W!
    #endif

    #define SERIAL_TXD  0 // = TX0
    #define SERIAL_RXD  1 // = RX0

    #define LED_IO  LED_BUILTIN
    #define USE_LED


//-- Raspberry Pi Pico 2 W
#elif defined MODULE_RP_PICO_2W
    #ifndef ARDUINO_RASPBERRY_PI_PICO_2W
        #error Select board Raspberry Pi Pico 2W!
    #endif

    #define SERIAL_TXD  0 // = TX0
    #define SERIAL_RXD  1 // = RX0

    #define LED_IO  LED_BUILTIN
    #define USE_LED


#else
    #error No module selected !
#endif


//-------------------------------------------------------
// Internals
//-------------------------------------------------------

// the Arduino API defines SERIAL as 0x0 (a print format constant), so undefine it first
#undef SERIAL

// debug is always the USB Serial, the communication port is always a UART, so both are always available
#define DBG  Serial
#define DBG_PRINT(x)  Serial.print(x)
#define DBG_PRINTLN(x)  Serial.println(x)

#if defined USE_SERIAL2
    #undef SERIAL_TXD // Serial2 is UART1, so the UART0 pins of the module don't apply
    #undef SERIAL_RXD
    #define SERIAL_TXD  8 // = TX1
    #define SERIAL_RXD  9 // = RX1

    #define SERIAL  Serial2

#else // default, Serial1 for communication
    #define SERIAL  Serial1
#endif


void dbg_init(void)
{
    DBG.begin(115200);
    // USB CDC drops TX when the host has not raised DTR, so don't wait for it
    DBG.ignoreFlowControl(true);
    DBG_PRINTLN();
    DBG_PRINTLN("Hello");
}


#if defined LED_IO && defined USE_LED
    void led_init(void)
    {
        pinMode(LED_IO, OUTPUT);
        digitalWrite(LED_IO, LOW);
    }

    void led_on(bool is_connected)
    {
        digitalWrite(LED_IO, HIGH);
    }

    void led_off(void)
    {
        digitalWrite(LED_IO, LOW);
    }
#endif

#ifndef USE_LED
    void led_init(void) {}
    void led_on(bool is_connected) {}
    void led_off(void) {}
#endif

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
LED is on the CYW43 chip (WL_GPIO0), available as LED_BUILTIN

------------------------------
Raspberry Pi Pico 2 W
------------------------------
board: Raspberry Pi Pico 2W
RP2350 + CYW43439, pin compatible with the Pico W

------------------------------
Generic
------------------------------
any other board with a CYW43439 (e.g. Pimoroni Pico Plus 2 W)
set SERIAL_RXD, SERIAL_TXD and LED_IO in the sketch as needed

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

    #undef LED_IO
    #define LED_IO  LED_BUILTIN
    #define USE_LED


//-- Raspberry Pi Pico 2 W
#elif defined MODULE_RP_PICO_2W
    #ifndef ARDUINO_RASPBERRY_PI_PICO_2W
        #error Select board Raspberry Pi Pico 2W!
    #endif

    #define SERIAL_TXD  0 // = TX0
    #define SERIAL_RXD  1 // = RX0

    #undef LED_IO
    #define LED_IO  LED_BUILTIN
    #define USE_LED


//-- Generic
#elif defined MODULE_GENERIC
    #ifdef LED_IO
        #define USE_LED
    #endif

#else
    #error No module selected !
#endif


//-------------------------------------------------------
// Internals
//-------------------------------------------------------

// the Arduino API defines SERIAL as 0x0 (a print format constant), so undefine it first
#undef SERIAL

#if defined USE_SERIAL2_DBG
    #define SERIAL  Serial2
    #define DBG  Serial
    #define DBG_PRINT(x)  Serial.print(x)
    #define DBG_PRINTLN(x)  Serial.println(x)

#elif defined USE_SERIAL1_NODBG
    #define SERIAL  Serial1

    #define DBG_PRINT(x)
    #define DBG_PRINTLN(x)

#else // default, Serial1 for communication, USB Serial for debug
    #define SERIAL  Serial1
    #define DBG  Serial
    #define DBG_PRINT(x)  Serial.print(x)
    #define DBG_PRINTLN(x)  Serial.println(x)
#endif


#ifdef DBG
    void dbg_init(void)
    {
        DBG.begin(115200);
        // USB CDC drops TX when the host has not raised DTR, so don't wait for it
        DBG.ignoreFlowControl(true);
        DBG_PRINTLN();
        DBG_PRINTLN("Hello");
    }
#else
    void dbg_init(void) {}
#endif


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

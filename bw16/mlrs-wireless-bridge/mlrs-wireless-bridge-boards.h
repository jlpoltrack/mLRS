//*******************************************************
// mLRS Wireless Bridge for BW16 (RTL8720DN)
// Copyright (c) www.olliw.eu, OlliW, OlliW42
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Board definitions
//*******************************************************
// 13. Mar. 2026

/*
------------------------------
BW16 (RTL8720DN)
------------------------------
board: BW16 (RTL8720DN) in AmebaD board package
https://www.amebaiot.com/en/amebad-bw16-arduino-getting-started/

LOG_UART: PA7 (TX) / PA8 (RX) - used for program upload and debug
UART1:    PB1 (TX) / PB2 (RX) - used for bridge serial data

tri-color LED:
  PA12 = Red
  PA13 = Blue
  PA14 = Green
*/


//-------------------------------------------------------
// Module details
//-------------------------------------------------------

// currently only one module is supported
#ifndef MODULE_BW16
#define MODULE_BW16
#endif

#undef SERIAL  // AmebaD wiring_constants.h defines SERIAL as 0x0 (pin mode), must undef
#define SERIAL Serial1

// UART1 pins (bridge data)
#define SERIAL_RXD  PB2
#define SERIAL_TXD  PB1

// debug via LOG_UART
#define DBG Serial
#define DBG_PRINT(x)    Serial.print(x)
#define DBG_PRINTLN(x)  Serial.println(x)


//-------------------------------------------------------
// LED - tri-color discrete RGB on PA12/PA13/PA14
//-------------------------------------------------------

#define LED_RED_PIN    PA12
#define LED_BLUE_PIN   PA13
#define LED_GREEN_PIN  PA14

// state constants, matching esp-hal-led-rgb.h pattern
#define LED_RGB_OFF    0
#define LED_RGB_RED    1
#define LED_RGB_GREEN  2
#define LED_RGB_BLUE   3

uint8_t ledCurrentColorState = LED_RGB_OFF;

void led_init(void)
{
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_BLUE_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
}

// helper: set color and track state, skip if already in target state
void led_set(bool r, bool g, bool b, uint8_t state)
{
    if (ledCurrentColorState == state) return;
    digitalWrite(LED_RED_PIN, r ? HIGH : LOW);
    digitalWrite(LED_GREEN_PIN, g ? HIGH : LOW);
    digitalWrite(LED_BLUE_PIN, b ? HIGH : LOW);
    ledCurrentColorState = state;
}

void led_on(bool is_connected)
{
    if (is_connected) {
        led_set(0, 1, 0, LED_RGB_GREEN);  // green = connected
    } else {
        led_set(1, 0, 0, LED_RGB_RED);    // red = not connected
    }
}

void led_off(void)
{
    led_set(0, 0, 0, LED_RGB_OFF);
}


//-------------------------------------------------------
// Debug init
//-------------------------------------------------------

void dbg_init(void)
{
    DBG.begin(115200);
    DBG_PRINTLN();
    DBG_PRINTLN("Hello");
}

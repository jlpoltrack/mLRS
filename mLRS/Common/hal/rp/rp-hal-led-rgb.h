//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP2040/RP2350 RGB LED using NeoPixelBus (PIO0)
//********************************************************
#ifndef RPLIB_LED_RGB_H
#define RPLIB_LED_RGB_H

#include <NeoPixelBus.h>

#define LED_RGB_OFF    0
#define LED_RGB_RED    1
#define LED_RGB_GREEN  2
#define LED_RGB_BLUE   3
#define LED_RGB_PURPLE 4

static uint8_t ledCurrentColorState;

// use PIO0 to leave PIO1 free for CAN on receivers
NeoPixelBus<NeoGrbFeature, Rp2040x4Pio0Ws2812xMethod> ledRGB(LED_RGB_PIXEL_NUM, LED_RGB);


void leds_init(void)
{
    ledRGB.Begin();
    ledRGB.ClearTo(RgbColor(0));
    ledRGB.Show();
    ledCurrentColorState = LED_RGB_OFF;
}


void set_led_color_and_state(uint8_t targetState, RgbColor color)
{
    if (ledCurrentColorState == targetState) return;
    ledRGB.ClearTo(color);
    ledRGB.Show();
    ledCurrentColorState = targetState;
}


void led_red_off(void)    { set_led_color_and_state(LED_RGB_OFF, RgbColor(0)); }
void led_red_on(void)     { set_led_color_and_state(LED_RGB_RED, RgbColor(255, 0, 0)); }
void led_red_toggle(void) { (ledCurrentColorState == LED_RGB_RED) ? led_red_off() : led_red_on(); }

void led_green_off(void)    { set_led_color_and_state(LED_RGB_OFF, RgbColor(0)); }
void led_green_on(void)     { set_led_color_and_state(LED_RGB_GREEN, RgbColor(0, 255, 0)); }
void led_green_toggle(void) { (ledCurrentColorState == LED_RGB_GREEN) ? led_green_off() : led_green_on(); }

void led_blue_off(void)    { set_led_color_and_state(LED_RGB_OFF, RgbColor(0)); }
void led_blue_on(void)     { set_led_color_and_state(LED_RGB_BLUE, RgbColor(0, 0, 255)); }
void led_blue_toggle(void) { (ledCurrentColorState == LED_RGB_BLUE) ? led_blue_off() : led_blue_on(); }

void led_purple_off(void)    { set_led_color_and_state(LED_RGB_OFF, RgbColor(0)); }
void led_purple_on(void)     { set_led_color_and_state(LED_RGB_PURPLE, RgbColor(255, 0, 255)); }
void led_purple_toggle(void) { (ledCurrentColorState == LED_RGB_PURPLE) ? led_purple_off() : led_purple_on(); }


#endif // RPLIB_LED_RGB_H

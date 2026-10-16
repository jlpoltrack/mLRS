//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP hal common
//*******************************************************
// Functions which are identical for all ESP boards. Is included after the board hal,
// and the board hal selects what it gets by the pins and features it defines.


//-- SX1
// DIO is polled in the main loop, so no isr is attached

#ifdef SX_BUSY
IRAM_ATTR bool sx_busy_read(void) { return (gpio_read_activehigh(SX_BUSY)) ? true : false; }
#endif

#if defined SX_TX_EN && defined SX_RX_EN
IRAM_ATTR void sx_amp_transmit(void) { gpio_low(SX_RX_EN); gpio_high(SX_TX_EN); }
IRAM_ATTR void sx_amp_receive(void) { gpio_low(SX_TX_EN); gpio_high(SX_RX_EN); }
#elif defined SX_TX_EN
IRAM_ATTR void sx_amp_transmit(void) { gpio_high(SX_TX_EN); }
IRAM_ATTR void sx_amp_receive(void) { gpio_low(SX_TX_EN); }
#else
IRAM_ATTR void sx_amp_transmit(void) {}
IRAM_ATTR void sx_amp_receive(void) {}
#endif

void sx_init_gpio(void)
{
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH);
    gpio_init(SX_DIO, IO_MODE_INPUT_ANALOG);
#ifdef SX_BUSY
    gpio_init(SX_BUSY, IO_MODE_INPUT_ANALOG);
#endif
#ifdef SX_TX_EN
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW);
#endif
#ifdef SX_RX_EN
    gpio_init(SX_RX_EN, IO_MODE_OUTPUT_PP_LOW);
#endif
#ifdef SX_ANT
    gpio_init(SX_ANT, IO_MODE_OUTPUT_PP_HIGH); // force to one antenna
#endif
#ifdef PWR_EN
    gpio_init(PWR_EN, IO_MODE_OUTPUT_PP_HIGH); // front-end enable
#endif
}

IRAM_ATTR bool sx_dio_read(void) { return (gpio_read_activehigh(SX_DIO)) ? true : false; }
void sx_dio_init_exti_isroff(void) {}
void sx_dio_enable_exti_isr(void) {}
void sx_dio_exti_isr_clearflag(void) {}


//-- SX2

#ifdef SX2_DIO

IRAM_ATTR void spib_select(void) { gpio_low(SX2_CS_IO); }
IRAM_ATTR void spib_deselect(void) { gpio_high(SX2_CS_IO); }

#ifdef SX2_BUSY
IRAM_ATTR bool sx2_busy_read(void) { return (gpio_read_activehigh(SX2_BUSY)) ? true : false; }
#endif

#if defined SX2_TX_EN && defined SX2_RX_EN
IRAM_ATTR void sx2_amp_transmit(void) { gpio_low(SX2_RX_EN); gpio_high(SX2_TX_EN); }
IRAM_ATTR void sx2_amp_receive(void) { gpio_low(SX2_TX_EN); gpio_high(SX2_RX_EN); }
#else
IRAM_ATTR void sx2_amp_transmit(void) {}
IRAM_ATTR void sx2_amp_receive(void) {}
#endif

void sx2_init_gpio(void)
{
    gpio_init(SX2_CS_IO, IO_MODE_OUTPUT_PP_HIGH);
    gpio_init(SX2_RESET, IO_MODE_OUTPUT_PP_HIGH);
    gpio_init(SX2_DIO, IO_MODE_INPUT_ANALOG);
#ifdef SX2_BUSY
    gpio_init(SX2_BUSY, IO_MODE_INPUT_ANALOG);
#endif
#if defined SX2_TX_EN && defined SX2_RX_EN
    gpio_init(SX2_TX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX2_RX_EN, IO_MODE_OUTPUT_PP_LOW);
#endif
}

IRAM_ATTR bool sx2_dio_read(void) { return (gpio_read_activehigh(SX2_DIO)) ? true : false; }
void sx2_dio_init_exti_isroff(void) {}
void sx2_dio_enable_exti_isr(void) {}
void sx2_dio_exti_isr_clearflag(void) {}

#endif // SX2_DIO


//-- In port

#ifdef DEVICE_HAS_IN
void in_init_gpio(void) {}
void in_set_normal(void) { gpio_matrix_in((gpio_num_t)UARTE_USE_RX_IO, U1RXD_IN_IDX, false); }
void in_set_inverted(void) { gpio_matrix_in((gpio_num_t)UARTE_USE_RX_IO, U1RXD_IN_IDX, true); }
#endif


//-- Out port
// https://github.com/espressif/esp-idf/blob/release/v4.4/components/esp_rom/include/esp32/rom/gpio.h#L228-L242

#ifdef DEVICE_HAS_OUT
void out_init_gpio(void) {}
void out_set_normal(void) { gpio_matrix_out((gpio_num_t)UART_USE_TX_IO, U1TXD_OUT_IDX, false, false); }
void out_set_inverted(void) { gpio_matrix_out((gpio_num_t)UART_USE_TX_IO, U1TXD_OUT_IDX, true, false); }
#endif


//-- Button

#ifdef BUTTON
void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU);
#ifdef BUTTON2
    gpio_init(BUTTON2, IO_MODE_INPUT_PU);
#endif
}

bool button_pressed(void) { return gpio_read_activelow(BUTTON) ? true : false; }
#else
void button_init(void) {}
bool button_pressed(void) { return false; }
#endif

#ifdef BUTTON2
bool button2_pressed(void) { return gpio_read_activelow(BUTTON2) ? true : false; }
#endif


//-- LEDs

#ifdef LED_RED
void leds_init(void) { gpio_init(LED_RED, IO_MODE_OUTPUT_PP_LOW); }
void led_red_off(void) { gpio_low(LED_RED); }
void led_red_on(void) { gpio_high(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }
#endif


//-- 5 Way Switch
// The key whose level is nearest to the adc reading wins, 4095 is no key pressed. This is
// tolerant to the resistor values varying from board to board.

#ifdef FIVEWAY_ADC_IO

void fiveway_init(void) {} // no init needed to read an analog pin in Arduino

uint16_t fiveway_adc_read(void) { return analogRead(FIVEWAY_ADC_IO); }

uint8_t fiveway_read(void)
{
    const int16_t level[] = { KEY_UP_THRESH, KEY_DOWN_THRESH, KEY_LEFT_THRESH, KEY_RIGHT_THRESH, KEY_CENTER_THRESH, 4095 };
    const uint8_t key[] = { (1 << KEY_UP), (1 << KEY_DOWN), (1 << KEY_LEFT), (1 << KEY_RIGHT), (1 << KEY_CENTER), 0 };

    int16_t adc = analogRead(FIVEWAY_ADC_IO);
    uint8_t nearest = 5;
    for (uint8_t i = 0; i < 5; i++) {
        if (abs(adc - level[i]) < abs(adc - level[nearest])) nearest = i;
    }
    return key[nearest];
}

#endif


//-- Cooling Fan
// PWM fan, half at 250 mW, full above 250 mW, off below.
// 1 dB of margin, the actual power may be slightly below the nominal power of a level

#ifdef FAN_IO

void fan_init(void)
{
    gpio_init(FAN_IO, IO_MODE_OUTPUT_PP_LOW);
    analogWriteFrequency(25000);
}

void fan_set_power(int8_t power_dbm)
{
    if (power_dbm > POWER_24_DBM) {
        analogWrite(FAN_IO, 255);
    } else if (power_dbm >= POWER_23_DBM) {
        analogWrite(FAN_IO, 127);
    } else {
        analogWrite(FAN_IO, 0);
    }
}

#endif


//-- ESP Wifi Bridge

#if defined ESP_RESET && defined ESP_GPIO0

void esp_init(void)
{
    gpio_init(ESP_GPIO0, IO_MODE_OUTPUT_PP_LOW); // high -> esp will start in bootloader mode
    gpio_init(ESP_RESET, IO_MODE_OUTPUT_PP_LOW); // low -> esp is in reset
}

void esp_reset_high(void) { gpio_high(ESP_RESET); }
void esp_reset_low(void) { gpio_low(ESP_RESET); }

// the gpio0 line is inverted
void esp_gpio0_high(void) { gpio_low(ESP_GPIO0); }
void esp_gpio0_low(void) { gpio_high(ESP_GPIO0); }

#endif

#ifdef ESP_BOOTPIN
// no need to configure ESP_BOOTPIN, it is pulled up by default
uint8_t esp_bootpin() { return gpio_read_activelow(ESP_BOOTPIN); }
#endif

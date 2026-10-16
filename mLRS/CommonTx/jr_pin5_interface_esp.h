//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// JR Pin5 Interface Header for ESP32
//********************************************************
// Interrupt driven UART1 via the IDF low-level layer, isr runs on the main loop core.
// Half-duplex switches one pin with the GPIO matrix, full-duplex is never inverted.
//********************************************************
#ifndef JRPIN5_INTERFACE_ESP_H
#define JRPIN5_INTERFACE_ESP_H


#include <driver/uart.h>
#include <hal/uart_ll.h>
#include <hal/gpio_ll.h>
#include <soc/uart_periph.h>
#include <soc/gpio_pins.h>
#include <esp_rom_gpio.h>
#include <esp_intr_alloc.h>


#ifndef UART_USE_SERIAL1
  #error JRPin5 must use Serial1!
#endif


//-------------------------------------------------------
// Low-level UART1 driver
//-------------------------------------------------------
// rx calls UART_RX_CALLBACK_FULL(c) for each byte, bytes come in on rx timeout
// tx writes directly into the 128 bytes hw fifo, tx done calls UART_TC_CALLBACK()

#define UART_UARTx                (&UART1)
#define UART_UART_NUM             UART_NUM_1

#define JRPIN5_UART_RXBUFSIZE     256 // MUST be 2^N, only used in serial mode
#define UART_RXBUFSIZEMASK        (JRPIN5_UART_RXBUFSIZE-1)

#define UART_RX_TOUT_US           30 // gap between end of a received frame and start of our response

#define UART_INTR_RX_MASK         (UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_OVF)

volatile char uart_rxbuf[JRPIN5_UART_RXBUFSIZE];
volatile uint16_t uart_rxwritepos; // pos at which the last byte was stored
volatile uint16_t uart_rxreadpos; // pos at which the next byte is to be fetched

intr_handle_t uart_intr_handle = nullptr;
portMUX_TYPE uart_spinlock = portMUX_INITIALIZER_UNLOCKED; // int_ena read-modify-writes in main loop vs isr


void uart_rx_putc_torxbuf(uint8_t c)
{
    uint16_t next = (uart_rxwritepos + 1) & UART_RXBUFSIZEMASK;
    if (uart_rxreadpos != next) { // fifo not full
        uart_rxbuf[next] = c;
        uart_rxwritepos = next;
    }
}


// enabling also discards what the rx fifo received while disabled
void uart_rx_enableisr(FunctionalState flag)
{
    portENTER_CRITICAL_SAFE(&uart_spinlock);
    if (flag == ENABLE) {
        uart_ll_rxfifo_rst(UART_UARTx);
        uart_ll_clr_intsts_mask(UART_UARTx, UART_INTR_RX_MASK);
        uart_ll_ena_intr_mask(UART_UARTx, UART_INTR_RX_MASK);
    } else {
        uart_ll_disable_intr_mask(UART_UARTx, UART_INTR_RX_MASK);
    }
    portEXIT_CRITICAL_SAFE(&uart_spinlock);
}


void uart_disableisr(void)
{
    portENTER_CRITICAL_SAFE(&uart_spinlock);
    uart_ll_disable_intr_mask(UART_UARTx, UART_LL_INTR_MASK);
    portEXIT_CRITICAL_SAFE(&uart_spinlock);
}


void uart_putbuf(const uint8_t* const buf, uint16_t len)
{
    uint32_t free = uart_ll_get_txfifo_len(UART_UARTx);
    uart_ll_write_txfifo(UART_UARTx, buf, (len < free) ? len : free);
}


// isr context only, TX_DONE is cleared first so it signals only these bytes are out
void uart_tx_start_tc(const uint8_t* const buf, uint16_t len)
{
    uart_ll_clr_intsts_mask(UART_UARTx, UART_INTR_TX_DONE);
    uart_putbuf(buf, len);
    uart_ll_ena_intr_mask(UART_UARTx, UART_INTR_TX_DONE);
}


static void uart_irq_handler(void* arg)
{
    uint32_t status = uart_ll_get_intsts_mask(UART_UARTx);

    if (status & UART_INTR_RX_MASK) {
        uint32_t len = uart_ll_get_rxfifo_len(UART_UARTx);
        // the callback may disable rx, the remaining bytes are then discarded
        while (len-- && (uart_ll_get_intr_ena_status(UART_UARTx) & UART_INTR_RXFIFO_TOUT)) {
            uint8_t c;
            uart_ll_read_rxfifo(UART_UARTx, &c, 1);
            UART_RX_CALLBACK_FULL(c);
        }
        uart_ll_clr_intsts_mask(UART_UARTx, UART_INTR_RX_MASK);
    }

    if (status & UART_INTR_TX_DONE) {
        uart_ll_disable_intr_mask(UART_UARTx, UART_INTR_TX_DONE);
        uart_ll_clr_intsts_mask(UART_UARTx, UART_INTR_TX_DONE);
        UART_TC_CALLBACK();
    }
}


// in bit times, must exceed a symbol so it does not fire within a byte
uint16_t uart_rx_tout_bits(uint32_t baud)
{
    uint32_t bits = (baud * UART_RX_TOUT_US) / 1000000;
    return (bits < 16) ? 16 : bits;
}


void uart_init(uint32_t baud)
{
    uart_disableisr();

    uart_config_t config = {};
    config.baud_rate = baud;
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.source_clk = UART_SCLK_APB;
    uart_param_config(UART_UART_NUM, &config); // enables the peripheral, works without uart driver

    uart_ll_set_tx_idle_num(UART_UARTx, 0);
    uart_ll_set_rxfifo_full_thr(UART_UARTx, UART_LL_FIFO_DEF_LEN - 32); // only overflow guard
    uart_ll_set_rx_tout(UART_UARTx, uart_rx_tout_bits(baud));
    uart_ll_clr_intsts_mask(UART_UARTx, UART_LL_INTR_MASK);

#ifdef JR_PIN5_FULL_DUPLEX
    uart_set_pin(UART_UART_NUM, UART_USE_TX_IO, UART_USE_RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
#endif
    // half-duplex pin handling is done by tPin5BridgeBase

    uart_rxwritepos = uart_rxreadpos = 0;

    if (!uart_intr_handle) {
        esp_intr_alloc(uart_periph_signal[UART_UART_NUM].irq, 0, uart_irq_handler, NULL, &uart_intr_handle);
    }

    uart_rx_enableisr(ENABLE);
}


//-------------------------------------------------------
// Pin5 Serial class
// used for ESP passthrough flashing
// switches the uart from the pin5 bridge to a plain serial, can only be undone by restart

class tJrPin5SerialPort : public tSerialBase
{
  public:
    void SetBaudRate(uint32_t baud) override
    {
        uart_rx_callback_ptr = &uart_rx_putc_torxbuf;
        uart_tc_callback_ptr = &uart_tc_callback_dummy;
        uart_init(baud);
    }
    bool full(void) override { return (uart_ll_get_txfifo_len(UART_UARTx) == 0); }
    void putbuf(uint8_t* const buf, uint16_t len) override { uart_putbuf(buf, len); }
    bool available(void) override { return (uart_rxwritepos != uart_rxreadpos); }
    char getc(void) override
    {
        if (uart_rxwritepos == uart_rxreadpos) return '\0';
        uart_rxreadpos = (uart_rxreadpos + 1) & UART_RXBUFSIZEMASK;
        return uart_rxbuf[uart_rxreadpos];
    }
    void flush(void) override { uart_rxreadpos = uart_rxwritepos; }
};

tJrPin5SerialPort jrpin5serial;


//-------------------------------------------------------
// Pin5BridgeBase class

class tPin5BridgeBase
{
  public:
    void Init(void);

    // telemetry handling
    bool telemetry_start_next_tick;
    uint16_t telemetry_state;

    void TelemetryStart(void);

    // interface to the uart hardware peripheral used for the bridge, may be called in isr context
    // transmit_start() hands the frame to pin5_putbuf(), pin5_tx_start() writes it after pin5_tx_enable()
    void pin5_init(void);
    void pin5_tx_start(void) { uart_tx_start_tc(pin5_tx_buf, pin5_tx_len); }
    void pin5_putbuf(uint8_t* const buf, uint16_t len) { pin5_tx_buf = buf; pin5_tx_len = len; }
    bool pin5_set_protocol(uint32_t baudrate, bool inverted);
    bool pin5_inverted; // CRSF is normally inverted, but some radios use normal polarity

    // for in-isr processing
    void pin5_tx_enable(void);
    void pin5_rx_enable(void);
    virtual void parse_nextchar(uint8_t c) = 0;
    virtual bool transmit_start(void) = 0; // returns true if transmission should be started

    // actual isr functions
    void pin5_rx_callback(uint8_t c);
    void pin5_tc_callback(void);
    void pin5_cc1_callback(void) {} // is needed in derived classes

    // parser
    typedef enum {
        STATE_IDLE = 0,

        // CRSF receive states
        STATE_RECEIVE_CRSF_LEN,
        STATE_RECEIVE_CRSF_PAYLOAD,
        STATE_RECEIVE_CRSF_CRC,

        // transmit states, used by all
        STATE_TRANSMIT_START = 100,
        STATE_TRANSMITING,
    } STATE_ENUM;

    // not used in this class, but required by the children, so just add them here
    // no need for volatile since used only in isr context
    uint8_t state;

    // check and rescue, safety net as on STM32
    uint32_t nottransmiting_tlast_ms;
    void CheckAndRescue(void);

  private:
    uint8_t* pin5_tx_buf;
    uint16_t pin5_tx_len;
};


void tPin5BridgeBase::Init(void)
{
    state = STATE_IDLE;

    telemetry_start_next_tick = false;
    telemetry_state = 0;

    nottransmiting_tlast_ms = 0;

    pin5_init();
}


void tPin5BridgeBase::TelemetryStart(void)
{
    telemetry_start_next_tick = true;
}


//-------------------------------------------------------
// Interface to the uart hardware peripheral used for the bridge
// all except pin5_init(), pin5_set_protocol() may be called in isr context

void tPin5BridgeBase::pin5_init(void)
{
    uart_init(UART_BAUD);

#ifndef JR_PIN5_FULL_DUPLEX
    esp_rom_gpio_pad_select_gpio(UART_USE_TX_IO);
    gpio_set_direction((gpio_num_t)UART_USE_TX_IO, GPIO_MODE_INPUT);
#endif

    pin5_set_protocol(UART_BAUD, true);
}


bool tPin5BridgeBase::pin5_set_protocol(uint32_t baudrate, bool inverted)
{
#ifdef JR_PIN5_FULL_DUPLEX
    if (!inverted) return false; // non-inverted not supported for full duplex
#endif

    uart_disableisr(); // isr is now quiet, so no races with it
    uart_set_baudrate(UART_UART_NUM, baudrate);
    uart_ll_set_rx_tout(UART_UARTx, uart_rx_tout_bits(baudrate));

    pin5_inverted = inverted;
#ifndef JR_PIN5_FULL_DUPLEX
    gpio_set_pull_mode((gpio_num_t)UART_USE_TX_IO, (pin5_inverted) ? GPIO_PULLDOWN_ONLY : GPIO_PULLUP_ONLY); // pull to idle level
#endif
    pin5_rx_enable(); // apply polarity to rx pin, enables isr
    return true;
}


void tPin5BridgeBase::pin5_tx_enable(void)
{
#ifndef JR_PIN5_FULL_DUPLEX
    uart_rx_enableisr(DISABLE);

    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, U1RXD_IN_IDX, false); // disconnect rx from pad, rx sees idle
    if (pin5_inverted) { gpio_low(UART_USE_TX_IO); } else { gpio_high(UART_USE_TX_IO); } // set idle level
    gpio_ll_output_enable(&GPIO, (gpio_num_t)UART_USE_TX_IO);
    esp_rom_gpio_connect_out_signal(UART_USE_TX_IO, U1TXD_OUT_IDX, pin5_inverted, false);
#endif
}


void tPin5BridgeBase::pin5_rx_enable(void)
{
#ifndef JR_PIN5_FULL_DUPLEX
    gpio_ll_output_disable(&GPIO, (gpio_num_t)UART_USE_TX_IO); // also disconnects tx from pad
    esp_rom_gpio_connect_in_signal(UART_USE_TX_IO, U1RXD_IN_IDX, pin5_inverted);
#endif

    uart_rx_enableisr(ENABLE); // also discards bytes from the switching
}


// bytes come in batches on rx timeout, a frame split by a pause continues with the next batch
// we do not add a delay here before we transmit, the rx timeout provides the gap

void tPin5BridgeBase::pin5_rx_callback(uint8_t c)
{
    parse_nextchar(c);

    if (state < STATE_TRANSMIT_START) return; // we are in receiving

    if (state != STATE_TRANSMIT_START) { // we are in transmitting, should not happen!
        state = STATE_IDLE;
        return;
    }

    if (transmit_start()) { // check if a transmission waits, put it into buf and return true to start
        pin5_tx_enable();
        state = STATE_TRANSMITING;
        pin5_tx_start();
    } else {
        state = STATE_IDLE;
    }
}


void tPin5BridgeBase::pin5_tc_callback(void)
{
#ifndef JR_PIN5_FULL_DUPLEX
    pin5_rx_enable(); // switches on rx, full-duplex keeps rx on during tx
#endif
    state = STATE_IDLE;
}


//-------------------------------------------------------
// Check and rescue
// called in main loop, safety net in case the TC callback was missed

void tPin5BridgeBase::CheckAndRescue(void)
{
    uint32_t tnow_ms = millis32();

    if (state < STATE_TRANSMITING) {
        nottransmiting_tlast_ms = tnow_ms;
    } else {
        if (tnow_ms - nottransmiting_tlast_ms > 20) { // we are stuck, so rescue
            uart_disableisr();
            state = STATE_IDLE;
            pin5_rx_enable();
        }
    }
}


#endif // JRPIN5_INTERFACE_ESP_H

//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// JR Pin5 Interface Header for RP2040/RP2350
//*******************************************************
// Half-duplex UART on a single pin using two PIO SMs:
// RX parses in the PIO IRQ, TX is fed by DMA and the PIO program does the turnaround gap,
// pin direction switching and end-of-transmit signalling itself.
//********************************************************
#ifndef JRPIN5_INTERFACE_RP_H
#define JRPIN5_INTERFACE_RP_H

#include "../Common/protocols/crsf_protocol.h"
#include <hardware/pio.h>
#include <hardware/dma.h>
#include <hardware/irq.h>
#include <hardware/sync.h>
#include <hardware/clocks.h>

#ifndef UART_USE_PIO_HALF_DUPLEX
  #error "JRPin5 requires UART_USE_PIO_HALF_DUPLEX"
#endif


// rx program (pico-examples uart_rx), bytes with a bad stop bit are dropped
static const uint16_t pin5_rx_program[] = {
    0x2020, //  0: wait   0 pin, 0          ; start bit
    0xea27, //  1: set    x, 7        [10]  ; to middle of first data bit
    0x4001, //  2: in     pins, 1
    0x0642, //  3: jmp    x--, 2      [6]
    0x00c7, //  4: jmp    pin, 7            ; stop bit ok?
    0x20a0, //  5: wait   1 pin, 0          ; framing error, wait for idle
    0x0000, //  6: jmp    0                 ; and drop
    0x8020, //  7: push   block
};

// the radio is still finishing its stop bit when we see the frame end and needs time to switch to rx,
// so wait this many bit times (like the ESP's 2 symbol rx timeout) before driving the line
#define PIN5_TURNAROUND_BITS      20
static_assert(PIN5_TURNAROUND_BITS >= 1 && PIN5_TURNAROUND_BITS <= 32, "PIN5_TURNAROUND_BITS out of range");

// tx program, first FIFO word is the byte count - 1
static const uint16_t pin5_tx_program[] = {
    0x80a0, //  0: pull   block
    0x6048, //  1: out    y, 8              ; byte count - 1
    0xe033, //  2: set    x, 19             ; turnaround gap, pin still an input, patched in Init()
    0x0743, //  3: jmp    x--, 3      [7]
    0xf881, //  4: set    pindirs, 1  side 1
    0x9fa0, //  5: pull   block       side 1 [7] ; idle/stop bit
    0xf727, //  6: set    x, 7        side 0 [7] ; start bit
    0x6001, //  7: out    pins, 1
    0x0647, //  8: jmp    x--, 7      [6]
    0x0085, //  9: jmp    y--, 5
    0xbf42, // 10: nop                side 1 [7] ; last stop bit
    0xe080, // 11: set    pindirs, 0             ; release the line
    0xc010, // 12: irq    nowait 0 rel           ; tx done
};


static PIO pin5_pio = pio0;
static uint pin5_sm_rx;
static uint pin5_sm_tx;
static uint pin5_tx_offset;
static int pin5_dma_ch;

// tx_buf[0] holds the byte count - 1 for the PIO program, copied so the caller can refill its buffer
#define PIN5_TXBUFSIZE            (CRSF_FRAME_LEN_MAX + 16) // same as CRSF_BUF_SIZE
static uint8_t pin5_tx_buf[1 + PIN5_TXBUFSIZE];
static uint8_t pin5_tx_len;
static volatile bool pin5_sync_complete; // don't reply before a first frame was seen

class tPin5BridgeBase;
static tPin5BridgeBase* pin5_bridge;
static void pin5_irq_handler(void);


//-------------------------------------------------------
// Pin5BridgeBase class
//-------------------------------------------------------

class tPin5BridgeBase
{
  public:
    void Init(void);

    // telemetry handling
    bool telemetry_start_next_tick;
    uint16_t telemetry_state;

    void TelemetryStart(void) { telemetry_start_next_tick = true; }

    // interface to the uart hardware peripheral used for the bridge, called in isr context
    void pin5_putbuf(uint8_t* const buf, uint16_t len);
    bool pin5_set_protocol(uint32_t baudrate, bool inverted);

    // callbacks used by the other platforms, not needed here
    void pin5_rx_callback(uint8_t c) {}
    void pin5_tc_callback(void) {}
    void pin5_cc1_callback(void) {}

    // parsing and transmit, implemented by the child
    virtual void parse_nextchar(uint8_t c) = 0;
    virtual bool transmit_start(void) { return false; }

    // parser state
    typedef enum {
        STATE_IDLE = 0,
        STATE_RECEIVE_CRSF_LEN,
        STATE_RECEIVE_CRSF_PAYLOAD,
        STATE_RECEIVE_CRSF_CRC,
        STATE_TRANSMIT_START = 100,
        STATE_TRANSMITING,
    } STATE_ENUM;

    volatile uint8_t state;

    // the PIO always completes a transmit, so nothing to rescue
    void CheckAndRescue(void) {}

    void pin5_irq(void);
    void pin5_tx_start(void);
    void pin5_rx_irq_enable(bool enable);
};


void tPin5BridgeBase::Init(void)
{
    state = STATE_IDLE;
    telemetry_start_next_tick = false;
    telemetry_state = 0;
    pin5_sync_complete = false;
    pin5_bridge = this;

    // must be called only once, a controller restart on RP is a full reboot
    // IRQ is enabled on the calling core, which must be the core running the main loop
    uint pin = UART_TX_PIN;
    float div = (float)clock_get_hz(clk_sys) / (8.0f * UART_BAUD);

    pio_gpio_init(pin5_pio, pin);
    gpio_set_inover(pin, GPIO_OVERRIDE_INVERT);
    gpio_set_outover(pin, GPIO_OVERRIDE_INVERT);
    gpio_set_pulls(pin, false, true);

    // rx
    pin5_sm_rx = pio_claim_unused_sm(pin5_pio, true);
    pio_program_t rx_prog = { .instructions = pin5_rx_program, .length = count_of(pin5_rx_program), .origin = -1 };
    uint rx_offset = pio_add_program(pin5_pio, &rx_prog);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, rx_offset, rx_offset + count_of(pin5_rx_program) - 1);
    sm_config_set_in_pins(&c, pin);
    sm_config_set_jmp_pin(&c, pin);
    sm_config_set_in_shift(&c, true, false, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&c, div);
    pio_sm_init(pin5_pio, pin5_sm_rx, rx_offset, &c);

    // tx
    pin5_sm_tx = pio_claim_unused_sm(pin5_pio, true);
    uint16_t tx_instr[count_of(pin5_tx_program)];
    memcpy(tx_instr, pin5_tx_program, sizeof(tx_instr));
    tx_instr[2] = pio_encode_set(pio_x, PIN5_TURNAROUND_BITS - 1);
    pio_program_t tx_prog = { .instructions = tx_instr, .length = count_of(tx_instr), .origin = -1 };
    pin5_tx_offset = pio_add_program(pin5_pio, &tx_prog);

    c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, pin5_tx_offset, pin5_tx_offset + count_of(pin5_tx_program) - 1);
    sm_config_set_out_pins(&c, pin, 1);
    sm_config_set_set_pins(&c, pin, 1);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_sideset(&c, 2, true, false); // 1 bit, optional
    sm_config_set_out_shift(&c, true, false, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&c, div);
    pio_sm_init(pin5_pio, pin5_sm_tx, pin5_tx_offset, &c);
    pio_sm_set_consecutive_pindirs(pin5_pio, pin5_sm_tx, pin, 1, false);

    // dma feeds the tx FIFO, 8 bit writes are replicated to all byte lanes, the program uses the low byte
    pin5_dma_ch = dma_claim_unused_channel(true);
    dma_channel_config dc = dma_channel_get_default_config(pin5_dma_ch);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_8);
    channel_config_set_read_increment(&dc, true);
    channel_config_set_write_increment(&dc, false);
    channel_config_set_dreq(&dc, pio_get_dreq(pin5_pio, pin5_sm_tx, true));
    dma_channel_configure(pin5_dma_ch, &dc, &pin5_pio->txf[pin5_sm_tx], pin5_tx_buf, 0, false);

    // rx data and tx done share PIO0_IRQ_0
    pin5_rx_irq_enable(true);
    pio_set_irq0_source_enabled(pin5_pio, (pio_interrupt_source_t)(pis_interrupt0 + pin5_sm_tx), true);
    irq_set_exclusive_handler(PIO0_IRQ_0, pin5_irq_handler);
    irq_set_enabled(PIO0_IRQ_0, true);

    pio_sm_set_enabled(pin5_pio, pin5_sm_rx, true);
    pio_sm_set_enabled(pin5_pio, pin5_sm_tx, true);
}


void tPin5BridgeBase::pin5_irq(void)
{
    // tx done, the PIO has released the line; drop our echo and listen again
    if (pio_interrupt_get(pin5_pio, pin5_sm_tx)) {
        pio_interrupt_clear(pin5_pio, pin5_sm_tx);
        while (!pio_sm_is_rx_fifo_empty(pin5_pio, pin5_sm_rx)) pio_sm_get(pin5_pio, pin5_sm_rx);
        state = STATE_IDLE;
        pin5_rx_irq_enable(true);
    }

    while (state != STATE_TRANSMITING && !pio_sm_is_rx_fifo_empty(pin5_pio, pin5_sm_rx)) {
        uint8_t c = pio_sm_get(pin5_pio, pin5_sm_rx) >> 24;
        parse_nextchar(c);
        if (state != STATE_TRANSMIT_START) continue;

        if (!pin5_sync_complete) { // first frame after init or baud change, only sync
            pin5_sync_complete = true;
            state = STATE_IDLE;
        } else
        if (transmit_start() && pin5_tx_len) { // calls pin5_putbuf()
            state = STATE_TRANSMITING;
            pin5_rx_irq_enable(false); // don't take an IRQ for each echoed byte
            pin5_tx_start();
        } else {
            state = STATE_IDLE;
        }
    }
}


static void pin5_irq_handler(void) { pin5_bridge->pin5_irq(); }


void tPin5BridgeBase::pin5_rx_irq_enable(bool enable)
{
    pio_set_irq0_source_enabled(pin5_pio, (pio_interrupt_source_t)(pis_sm0_rx_fifo_not_empty + pin5_sm_rx), enable);
}


void tPin5BridgeBase::pin5_putbuf(uint8_t* const buf, uint16_t len)
{
    if (len > PIN5_TXBUFSIZE) len = PIN5_TXBUFSIZE;
    pin5_tx_len = len;
    if (!len) return;
    pin5_tx_buf[0] = len - 1;
    memcpy(pin5_tx_buf + 1, buf, len);
}


void tPin5BridgeBase::pin5_tx_start(void)
{
    dma_channel_transfer_from_buffer_now(pin5_dma_ch, pin5_tx_buf, pin5_tx_len + 1);
}


// called from the main loop, on the same core as the IRQ, so masking interrupts keeps it out
bool tPin5BridgeBase::pin5_set_protocol(uint32_t baudrate, bool inverted)
{
    uint32_t irq_status = save_and_disable_interrupts();

    // abort any transmit, back to a clean rx state
    dma_channel_abort(pin5_dma_ch);
    pio_sm_set_enabled(pin5_pio, pin5_sm_rx, false);
    pio_sm_set_enabled(pin5_pio, pin5_sm_tx, false);
    pio_sm_set_consecutive_pindirs(pin5_pio, pin5_sm_tx, UART_TX_PIN, 1, false);
    pio_sm_clear_fifos(pin5_pio, pin5_sm_rx);
    pio_sm_clear_fifos(pin5_pio, pin5_sm_tx);
    pio_sm_restart(pin5_pio, pin5_sm_rx);
    pio_sm_restart(pin5_pio, pin5_sm_tx);
    pio_sm_exec(pin5_pio, pin5_sm_tx, pio_encode_jmp(pin5_tx_offset));
    pio_interrupt_clear(pin5_pio, pin5_sm_tx);

    // CRSF is normally inverted, but some radios use normal polarity
    gpio_set_inover(UART_TX_PIN, (inverted) ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
    gpio_set_outover(UART_TX_PIN, (inverted) ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
    gpio_set_pulls(UART_TX_PIN, !inverted, inverted); // pull to idle level

    float div = (float)clock_get_hz(clk_sys) / (8.0f * baudrate);
    pio_sm_set_clkdiv(pin5_pio, pin5_sm_rx, div);
    pio_sm_set_clkdiv(pin5_pio, pin5_sm_tx, div);
    pio_sm_clkdiv_restart(pin5_pio, pin5_sm_rx);
    pio_sm_clkdiv_restart(pin5_pio, pin5_sm_tx);

    state = STATE_IDLE;
    pin5_sync_complete = false;
    pin5_rx_irq_enable(true);
    pio_sm_set_enabled(pin5_pio, pin5_sm_rx, true);
    pio_sm_set_enabled(pin5_pio, pin5_sm_tx, true);

    restore_interrupts(irq_status);
    return true;
}


// needed by init_hw(), not used on RP
tSerialBase jrpin5serial;


#endif // JRPIN5_INTERFACE_RP_H

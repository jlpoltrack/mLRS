//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// JR Pin5 Interface Header for RP2040/RP2350
// Custom PIO UART with IRQ-driven byte processing
// Full half-duplex: RX + TX on same pin
//********************************************************
#ifndef JRPIN5_INTERFACE_RP_H
#define JRPIN5_INTERFACE_RP_H

#include "../Common/protocols/crsf_protocol.h"
#include "../Common/libs/fifo.h"
#include <hardware/pio.h>
#include <hardware/irq.h>
#include <hardware/gpio.h>
#include <hardware/clocks.h>
#include <pico/time.h>

// Ensure the custom define is present
#ifndef UART_USE_PIO_HALF_DUPLEX
  #error "JRPin5 requires UART_USE_PIO_HALF_DUPLEX"
#endif

// pio resources - use PIO0 to leave PIO1 free for CYW43 WiFi on Pico W variants
// state machines are dynamically claimed to avoid init order dependencies
static PIO pio_uart = pio0;
static uint pio_sm_rx = 0;
static uint pio_sm_tx = 0;
static uint pio_rx_offset = 0;
static uint pio_tx_offset = 0;

#define TX_BUF_SIZE UART_TXBUFSIZE
static uint8_t tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_buf_len = 0;
static volatile uint16_t tx_index = 0;  // current position for IRQ-driven TX
static volatile bool tx_in_progress = false;
static volatile alarm_id_t pending_turnaround_alarm = 0;
static volatile bool sync_complete = false;  // don't TX until first frame received

// rx ring buffer for additional buffering (PIO FIFO is only 8 bytes)
static tFifo<uint8_t, UART_RXBUFSIZE> rx_fifo;

// bridge instance pointer - set during Init()
class tPin5BridgeBase;
static tPin5BridgeBase* g_pin5_bridge;

// pio uart rx program (from pico-examples uart_rx_mini)
static const uint16_t pio_uart_rx_mini[] = {
    0x2020,  // wait 0 pin 0 - wait for start bit
    0xea27,  // set x, 7 [10] - preload counter, delay to first data bit
    0x4001,  // in pins, 1 - sample data
    0x0642,  // jmp x-- 2 [6] - 8 iterations, 8 cycles each
};
#define PIO_UART_RX_MINI_LEN 4

// pio uart tx program - fires IRQ after each byte for TX complete detection
static const uint16_t pio_uart_tx[] = {
    0x9fa0,  // 0: pull block side 1 [7] - stall idle high (stop bit timing)
    0xf727,  // 1: set x, 7 side 0 [7] - start bit (low)
    0x6001,  // 2: out pins, 1 - output data bit
    0x0642,  // 3: jmp x-- 2 [6] - loop for 8 bits
    0xc000,  // 4: irq nowait 0 - signal byte complete, wrap to 0
};
#define PIO_UART_TX_LEN 5

//-------------------------------------------------------
// function declarations
//-------------------------------------------------------
void pio_uart_irq_handler(void);

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

    // interface methods
    void pin5_init_rx(void);
    void pin5_init_tx(void);
    
    void pin5_putbuf(uint8_t* const buf, uint16_t len);
    uint16_t pin5_bytes_available(void) { return 0; }
    void pin5_tx_enable(bool enable);
    void pin5_tx_start(void);
    
    // callback stubs
    void pin5_rx_callback(uint8_t c) {}
    void pin5_tc_callback(void);
    void pin5_cc1_callback(void) {}
    
    // callback processing (virtuals)
    virtual void parse_nextchar(uint8_t c) = 0;
    virtual bool transmit_start(void) { return false; }

    // parser state
    typedef enum {
        STATE_IDLE = 0,
        STATE_RECEIVE_MBRIDGE_STX2,
        STATE_RECEIVE_MBRIDGE_LEN,
        STATE_RECEIVE_MBRIDGE_SERIALPACKET,
        STATE_RECEIVE_MBRIDGE_CHANNELPACKET,
        STATE_RECEIVE_MBRIDGE_COMMANDPACKET,
        STATE_RECEIVE_CRSF_LEN,
        STATE_RECEIVE_CRSF_PAYLOAD,
        STATE_RECEIVE_CRSF_CRC,
        STATE_TRANSMIT_START,
        STATE_TRANSMIT_PENDING,
        STATE_TRANSMITING,
    } STATE_ENUM;

    volatile uint8_t state;
    volatile uint8_t len;
    volatile uint8_t cnt;
    volatile uint16_t tlast_us;

    void CheckAndRescue(void) {}
    void DebugReset(void) {}
};



//-------------------------------------------------------
// tx complete delay callback - called after last byte fully transmitted
// 2 byte times after PIO signals last byte pulled ensures stop bit is complete
//-------------------------------------------------------
static int64_t __not_in_flash_func(tx_complete_alarm_callback)(alarm_id_t id, void* user_data) {
    (void)id;
    (void)user_data;
    if (g_pin5_bridge && tx_in_progress) {
        g_pin5_bridge->pin5_tc_callback();
    }
    return 0;
}

// 2 byte times in microseconds (10 bits per byte at baud rate, times 2)
#define TX_STOP_BIT_DELAY_US  (20000000 / UART_BAUD)

//-------------------------------------------------------
// tx turnaround delay callback - starts tx after delay
//-------------------------------------------------------
static int64_t __not_in_flash_func(tx_turnaround_alarm_callback)(alarm_id_t id, void* user_data) {
    (void)id;
    (void)user_data;
    if (g_pin5_bridge && g_pin5_bridge->state == tPin5BridgeBase::STATE_TRANSMIT_PENDING) {
        g_pin5_bridge->pin5_tx_enable(true);
        g_pin5_bridge->state = tPin5BridgeBase::STATE_TRANSMITING;
        g_pin5_bridge->pin5_tx_start();
    }
    return 0;
}

// shared pio irq handler - handles both RX and TX
// RX: drains PIO to ring buffer, then parses
// TX: feeds bytes from tx_buf as FIFO drains
//-------------------------------------------------------
void __not_in_flash_func(pio_uart_irq_handler)(void) {
    // handle TX complete: PIO fires irq 0 after each byte
    if (pio_interrupt_get(pio_uart, 0)) {
        pio_interrupt_clear(pio_uart, 0);
        // check if this was the last byte (all bytes queued and FIFO drained)
        if (tx_in_progress && tx_index >= tx_buf_len && 
            pio_sm_is_tx_fifo_empty(pio_uart, pio_sm_tx)) {
            // wait 2 byte times for stop bit to complete before switching to RX
            add_alarm_in_us(TX_STOP_BIT_DELAY_US, tx_complete_alarm_callback, NULL, true);
        }
    }
    
    // handle RX: drain PIO FIFO to ring buffer
    while (!pio_sm_is_rx_fifo_empty(pio_uart, pio_sm_rx)) {
        uint8_t c = (uint8_t)(pio_sm_get(pio_uart, pio_sm_rx) >> 24);
        rx_fifo.Put(c);
    }
    
    // handle TX feed: push bytes while FIFO has space
    if (tx_in_progress && tx_index < tx_buf_len) {
        while (tx_index < tx_buf_len && !pio_sm_is_tx_fifo_full(pio_uart, pio_sm_tx)) {
            pio_sm_put(pio_uart, pio_sm_tx, (uint32_t)tx_buf[tx_index++]);
        }
        // disable TX FIFO IRQ when all bytes queued (PIO irq 0 will signal completion)
        if (tx_index >= tx_buf_len) {
            pio_interrupt_source_t tx_irq = (pio_interrupt_source_t)(pis_sm0_tx_fifo_not_full + pio_sm_tx);
            pio_set_irq1_source_enabled(pio_uart, tx_irq, false);
        }
    }
    
    if (!g_pin5_bridge) return;
    
    // parse bytes from ring buffer
    while (rx_fifo.Available()) {
        uint8_t c = rx_fifo.Get();
        g_pin5_bridge->parse_nextchar(c);
        
        if (g_pin5_bridge->state == tPin5BridgeBase::STATE_TRANSMIT_START) {
            if (!sync_complete) {
                // first frame after reinit - just sync, don't TX yet
                sync_complete = true;
                g_pin5_bridge->state = tPin5BridgeBase::STATE_IDLE;
            } else if (g_pin5_bridge->transmit_start()) {
                g_pin5_bridge->state = tPin5BridgeBase::STATE_TRANSMIT_PENDING;
                pending_turnaround_alarm = add_alarm_in_us(100, tx_turnaround_alarm_callback, NULL, true);
            } else {
                g_pin5_bridge->state = tPin5BridgeBase::STATE_IDLE;
            }
        }
    }
}

//-------------------------------------------------------
// init implementations
//-------------------------------------------------------

void tPin5BridgeBase::Init(void)
{
    state = STATE_IDLE;
    len = 0;
    cnt = 0;
    tlast_us = 0;
    telemetry_start_next_tick = false;
    telemetry_state = 0;
    
    // don't TX until first frame received (parser needs to sync first)
    sync_complete = false;
    rx_fifo.Init();
    
    pin5_init_rx();
    pin5_init_tx();
    
    g_pin5_bridge = this;
}

void tPin5BridgeBase::pin5_init_rx(void)
{
    pio_sm_rx = pio_claim_unused_sm(pio_uart, true);
    
    static const pio_program_t uart_rx_prog = {
        .instructions = pio_uart_rx_mini,
        .length = PIO_UART_RX_MINI_LEN,
        .origin = -1
    };
    pio_rx_offset = pio_add_program(pio_uart, &uart_rx_prog);
    
    pio_sm_set_consecutive_pindirs(pio_uart, pio_sm_rx, UART_TX_PIN, 1, false);
    pio_gpio_init(pio_uart, UART_TX_PIN);
    
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, pio_rx_offset, pio_rx_offset + PIO_UART_RX_MINI_LEN - 1);
    sm_config_set_in_pins(&c, UART_TX_PIN);
    sm_config_set_in_shift(&c, true, true, 8);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    
    float div = (float)clock_get_hz(clk_sys) / (8.0f * (float)UART_BAUD);
    sm_config_set_clkdiv(&c, div);
    
    // fully reset SM before init to ensure clean state
    pio_sm_restart(pio_uart, pio_sm_rx);
    pio_sm_clear_fifos(pio_uart, pio_sm_rx);
    pio_sm_init(pio_uart, pio_sm_rx, pio_rx_offset, &c);
    
    gpio_set_inover(UART_TX_PIN, GPIO_OVERRIDE_INVERT);
    gpio_set_pulls(UART_TX_PIN, false, true);
    
    uint irq_num = PIO0_IRQ_1;  // use IRQ_1 to leave IRQ_0 for other PIO users
    pio_interrupt_source_t irq_source = (pio_interrupt_source_t)(pis_sm0_rx_fifo_not_empty + pio_sm_rx);
    pio_set_irq1_source_enabled(pio_uart, irq_source, true);
    irq_set_exclusive_handler(irq_num, pio_uart_irq_handler);
    irq_set_enabled(irq_num, true);
    
    pio_sm_set_enabled(pio_uart, pio_sm_rx, true);
}

void tPin5BridgeBase::pin5_init_tx(void)
{
    pio_sm_tx = pio_claim_unused_sm(pio_uart, true);
    
    static const pio_program_t uart_tx_prog = {
        .instructions = pio_uart_tx,
        .length = PIO_UART_TX_LEN,
        .origin = -1
    };
    pio_tx_offset = pio_add_program(pio_uart, &uart_tx_prog);
    
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, pio_tx_offset, pio_tx_offset + PIO_UART_TX_LEN - 1);
    sm_config_set_out_shift(&c, true, false, 32);
    sm_config_set_out_pins(&c, UART_TX_PIN, 1);
    sm_config_set_sideset_pins(&c, UART_TX_PIN);
    sm_config_set_sideset(&c, 2, true, false);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    
    float div = (float)clock_get_hz(clk_sys) / (8.0f * (float)UART_BAUD);
    sm_config_set_clkdiv(&c, div);
    
    pio_sm_init(pio_uart, pio_sm_tx, pio_tx_offset, &c);
    
    // enable PIO IRQ flag 0 for TX complete detection
    pio_set_irq1_source_enabled(pio_uart, pis_interrupt0, true);
    gpio_set_outover(UART_TX_PIN, GPIO_OVERRIDE_INVERT);
}

//-------------------------------------------------------
// tx functions
//-------------------------------------------------------

void tPin5BridgeBase::pin5_tx_enable(bool enable)
{
    if (enable) {
        // disable RX IRQ to avoid reading our own TX
        pio_interrupt_source_t irq_source = (pio_interrupt_source_t)(pis_sm0_rx_fifo_not_empty + pio_sm_rx);
        pio_set_irq1_source_enabled(pio_uart, irq_source, false);
        
        // disable RX state machine
        pio_sm_set_enabled(pio_uart, pio_sm_rx, false);
        
        // set pin as output
        pio_sm_set_consecutive_pindirs(pio_uart, pio_sm_tx, UART_TX_PIN, 1, true);
        
        // enable TX state machine
        pio_sm_set_enabled(pio_uart, pio_sm_tx, true);
    } else {
        // drain TX FIFO (should be empty, but just in case)
        while (!pio_sm_is_tx_fifo_empty(pio_uart, pio_sm_tx)) {
            // wait for TX to finish
        }
        
        // disable and reset TX state machine
        pio_sm_set_enabled(pio_uart, pio_sm_tx, false);
        pio_sm_restart(pio_uart, pio_sm_tx);
        pio_sm_clear_fifos(pio_uart, pio_sm_tx);
        
        // set pin as input
        pio_sm_set_consecutive_pindirs(pio_uart, pio_sm_rx, UART_TX_PIN, 1, false);
        
        // clear any junk from RX FIFO
        while (!pio_sm_is_rx_fifo_empty(pio_uart, pio_sm_rx)) {
            pio_sm_get(pio_uart, pio_sm_rx);
        }
        
        // restart RX state machine
        pio_sm_restart(pio_uart, pio_sm_rx);
        pio_sm_exec(pio_uart, pio_sm_rx, pio_encode_jmp(pio_rx_offset));
        pio_sm_set_enabled(pio_uart, pio_sm_rx, true);
        
        // re-enable RX IRQ
        pio_interrupt_source_t irq_source = (pio_interrupt_source_t)(pis_sm0_rx_fifo_not_empty + pio_sm_rx);
        pio_set_irq1_source_enabled(pio_uart, irq_source, true);
    }
}

void tPin5BridgeBase::pin5_putbuf(uint8_t* const buf, uint16_t buflen)
{
    if (buflen > TX_BUF_SIZE) buflen = TX_BUF_SIZE;
    memcpy(tx_buf, buf, buflen);
    tx_buf_len = buflen;
}

void tPin5BridgeBase::pin5_tx_start(void)
{
    if (tx_buf_len == 0) return;

    tx_in_progress = true;
    tx_index = 0;

    // prime the FIFO with initial bytes (up to 8)
    while (tx_index < tx_buf_len && !pio_sm_is_tx_fifo_full(pio_uart, pio_sm_tx)) {
        pio_sm_put(pio_uart, pio_sm_tx, (uint32_t)tx_buf[tx_index++]);
    }
    
    // if more bytes remain, enable TX FIFO IRQ to continue feeding
    if (tx_index < tx_buf_len) {
        pio_interrupt_source_t tx_irq = (pio_interrupt_source_t)(pis_sm0_tx_fifo_not_full + pio_sm_tx);
        pio_set_irq1_source_enabled(pio_uart, tx_irq, true);
    }
    // TX complete will be detected by PIO irq 0 after last byte
}

void tPin5BridgeBase::pin5_tc_callback(void)
{
    tx_in_progress = false;
    tx_buf_len = 0;
    state = STATE_IDLE; // set state BEFORE re-enabling RX to avoid race condition
    pin5_tx_enable(false);
}


//-------------------------------------------------------
// jrpin5 serial class stub (needed for compilation)
//-------------------------------------------------------

class tJrPin5SerialPort : public tSerialBase
{
  public:
    void SetBaudRate(uint32_t baud) override {}
    bool full(void) { return false; }
    void putbuf(uint8_t* const buf, uint16_t len) override {}
    bool available(void) override { return false; }
    char getc(void) override { return 0; }
    void flush(void) override {}
};

tJrPin5SerialPort jrpin5serial;


#endif // JRPIN5_INTERFACE_RP_H

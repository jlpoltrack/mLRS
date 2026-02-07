//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP UART$
// IRQ-driven RX/TX buffering for hardware UARTs (Serial1/Serial2)
// and PIO-based UARTs with software ring buffers
//********************************************************
// 2026-02-03: refactored PIO serial to use SDK directly with IRQ-driven
// software ring buffers, bypassing blocking SerialPIO
//********************************************************
#ifndef RPLIB_UART$_H
#define RPLIB_UART$_H

// helper to detect PIO usage
#if defined(UART$_USE_SERIALPIO1) || defined(UART$_USE_SERIALPIO2)
  #define UART$_IS_PIO_SERIAL
  #include <hardware/pio.h>
  #include <hardware/irq.h>
  #include <hardware/gpio.h>
  #include <hardware/clocks.h>
#endif

// helper to detect hardware UART usage - use pure SDK with IRQ
#if defined(UART$_USE_SERIAL1) || defined(UART$_USE_SERIAL2)
  #define UART$_IS_HW_SERIAL
  #include <hardware/uart.h>
  #include <hardware/irq.h>
  #include <hardware/gpio.h>
#endif

// helper to detect USB serial
#ifdef UART$_USE_SERIAL
  #define UART$_IS_USB_SERIAL
#endif


//-------------------------------------------------------
// Enums
//-------------------------------------------------------
#ifndef RPLIB_UART_ENUMS
#define RPLIB_UART_ENUMS

typedef enum {
    XUART_PARITY_NO = 0,
    XUART_PARITY_EVEN,
    XUART_PARITY_ODD,
} UARTPARITYENUM;

typedef enum {
//    UART_STOPBIT_0_5 = 0, // not supported
    UART_STOPBIT_1 = 0,
    UART_STOPBIT_2,
} UARTSTOPBITENUM;

#endif


//-------------------------------------------------------
// Defines
//-------------------------------------------------------

#ifdef UART$_USE_SERIAL
  #define UART$_SERIAL_NO       Serial
#elif defined UART$_USE_SERIAL1
  #define UART$_UART_INST       uart0
  #define UART$_UART_IRQ        UART0_IRQ
#elif defined UART$_USE_SERIAL2
  #define UART$_UART_INST       uart1
  #define UART$_UART_IRQ        UART1_IRQ
#elif defined UART$_USE_SERIALPIO1
  // PIO serial uses PIO0 (PIO1 is reserved for CYW43 WiFi on Pico W)
  #define UART$_PIO_INST        pio0
  #define UART$_PIO_IRQ         PIO0_IRQ_0
  #define UART$_PIO_SET_IRQ_SOURCE_ENABLED  pio_set_irq0_source_enabled
#elif defined UART$_USE_SERIALPIO2
  // PIO serial uses PIO0 (PIO1 reserved, IRQ_1 to avoid conflict with SERIALPIO1)
  #define UART$_PIO_INST        pio0
  #define UART$_PIO_IRQ         PIO0_IRQ_1
  #define UART$_PIO_SET_IRQ_SOURCE_ENABLED  pio_set_irq1_source_enabled
#else
  #error UART$ serial type must be defined!
#endif


#ifndef UART$_TX_PIN
  #define UART$_TX_PIN          -1
#endif
#ifndef UART$_RX_PIN
  #define UART$_RX_PIN          -1
#endif
#ifndef UART$_TXBUFSIZE
  #define UART$_TXBUFSIZE       256 // MUST be 2^N
#endif
#ifndef UART$_RXBUFSIZE
  #define UART$_RXBUFSIZE       256 // MUST be 2^N
#endif


//-------------------------------------------------------
// Software buffers (for HW UART and PIO serial)
//-------------------------------------------------------

#if defined(UART$_IS_HW_SERIAL) || defined(UART$_IS_PIO_SERIAL)

#define UART$_TXBUFSIZEMASK  (UART$_TXBUFSIZE - 1)
#define UART$_RXBUFSIZEMASK  (UART$_RXBUFSIZE - 1)

// TX buffer - writepos/readpos point to LAST written/read position (STM32 pattern)
static volatile uint8_t uart$_txbuf[UART$_TXBUFSIZE];
static volatile uint16_t uart$_txwritepos;
static volatile uint16_t uart$_txreadpos;

// RX buffer
static volatile uint8_t uart$_rxbuf[UART$_RXBUFSIZE];
static volatile uint16_t uart$_rxwritepos;
static volatile uint16_t uart$_rxreadpos;

#endif


//-------------------------------------------------------
// PIO serial resources and programs
//-------------------------------------------------------
// LIMITATIONS:
// - 8N1 only (8 data bits, no parity, 1 stop bit)
// - parity and stopbits parameters are ignored
// - TX/RX inversion supported via UART$_INVERT_TX/UART$_INVERT_RX
// - uses PIO0 (PIO1 reserved for CYW43 WiFi on Pico W)
//-------------------------------------------------------

#ifdef UART$_IS_PIO_SERIAL

// PIO state machine assignments
static uint uart$_pio_sm_tx;
static uint uart$_pio_sm_rx;
static uint uart$_pio_tx_offset;
static uint uart$_pio_rx_offset;

// PIO UART TX program (8N1)
// based on pico-examples uart_tx
static const uint16_t uart$_pio_uart_tx_program[] = {
    0x9fa0,  // 0: pull block side 1 [7] - stall idle high
    0xf727,  // 1: set x, 7 side 0 [7] - start bit (low)
    0x6001,  // 2: out pins, 1 - output data bit
    0x0642,  // 3: jmp x-- 2 [6] - loop for 8 bits
    0xf767,  // 4: nop side 1 [7] - stop bit, wrap to 0
};
#define UART$_PIO_TX_PROG_LEN 5

// PIO UART RX program (8N1, mini version from pico-examples)
static const uint16_t uart$_pio_uart_rx_program[] = {
    0x2020,  // 0: wait 0 pin 0 - wait for start bit
    0xea27,  // 1: set x, 7 [10] - preload counter, delay to first data bit
    0x4001,  // 2: in pins, 1 - sample data
    0x0642,  // 3: jmp x-- 2 [6] - 8 iterations
};
#define UART$_PIO_RX_PROG_LEN 4


//-------------------------------------------------------
// PIO IRQ handler (in RAM)
//-------------------------------------------------------

void __not_in_flash_func(uart$_pio_irq_handler)(void)
{
    // RX: drain PIO FIFO to software buffer
    while (!pio_sm_is_rx_fifo_empty(UART$_PIO_INST, uart$_pio_sm_rx)) {
        uint32_t raw = pio_sm_get(UART$_PIO_INST, uart$_pio_sm_rx);
        uint8_t c = (uint8_t)(raw >> 24);  // data is in upper 8 bits with autopush
        uint16_t next = (uart$_rxwritepos + 1) & UART$_RXBUFSIZEMASK;
        if (next != uart$_rxreadpos) {  // not full
            uart$_rxbuf[next] = c;
            uart$_rxwritepos = next;
        }
    }

    // TX: drain software buffer to PIO FIFO
    while (!pio_sm_is_tx_fifo_full(UART$_PIO_INST, uart$_pio_sm_tx) &&
           (uart$_txwritepos != uart$_txreadpos)) {
        uart$_txreadpos = (uart$_txreadpos + 1) & UART$_TXBUFSIZEMASK;
        pio_sm_put(UART$_PIO_INST, uart$_pio_sm_tx, (uint32_t)uart$_txbuf[uart$_txreadpos]);
    }

    // disable TX IRQ when buffer empty (prevents endless IRQ)
    if (uart$_txwritepos == uart$_txreadpos) {
        UART$_PIO_SET_IRQ_SOURCE_ENABLED(UART$_PIO_INST,
            (pio_interrupt_source)(pis_sm0_tx_fifo_not_full + uart$_pio_sm_tx), false);
    }
}

#endif // UART$_IS_PIO_SERIAL


//-------------------------------------------------------
// HW UART IRQ handler (in RAM)
//-------------------------------------------------------

#ifdef UART$_IS_HW_SERIAL

// track if RX IRQ is enabled
static volatile bool uart$_rx_irq_enabled = true;

void __not_in_flash_func(uart$_irq_handler)(void)
{
    uart_hw_t* hw = uart_get_hw(UART$_UART_INST);

    // clear RX interrupt flags (write-to-clear)
    hw->icr = UART_UARTICR_RTIC_BITS | UART_UARTICR_RXIC_BITS;

    // RX: drain HW FIFO to software buffer
    while (uart_is_readable(UART$_UART_INST)) {
        uint32_t dr = hw->dr;
        if (dr & 0x700) continue;  // discard framing/parity/break errors
        uint16_t next = (uart$_rxwritepos + 1) & UART$_RXBUFSIZEMASK;
        if (next != uart$_rxreadpos) {  // not full
            uart$_rxbuf[next] = dr & 0xFF;
            uart$_rxwritepos = next;
        }
    }

    // TX: drain software buffer to HW FIFO
    while (uart_is_writable(UART$_UART_INST) && (uart$_txwritepos != uart$_txreadpos)) {
        uart$_txreadpos = (uart$_txreadpos + 1) & UART$_TXBUFSIZEMASK;
        hw->dr = uart$_txbuf[uart$_txreadpos];
    }

    // disable TX IRQ when buffer empty (prevents endless IRQ)
    if (uart$_txwritepos == uart$_txreadpos) {
        hw_clear_bits(&uart_get_hw(UART$_UART_INST)->imsc, UART_UARTIMSC_TXIM_BITS);
    }
}

#endif // UART$_IS_HW_SERIAL


//-------------------------------------------------------
// TX routines
//-------------------------------------------------------

#ifdef UART$_IS_HW_SERIAL

// non-blocking single char - returns 0 if buffer full
inline uint16_t uart$_putc(char c)
{
    uint16_t next = (uart$_txwritepos + 1) & UART$_TXBUFSIZEMASK;
    if (uart$_txreadpos != next) {  // not full
        uart$_txbuf[next] = c;
        uart$_txwritepos = next;
        // enable TX IRQ using direct register access (faster than uart_set_irq_enables)
        hw_set_bits(&uart_get_hw(UART$_UART_INST)->imsc, UART_UARTIMSC_TXIM_BITS);
        return 1;
    }
    return 0;
}

// blocking buffer write
// hybrid: fill HW FIFO directly first (fast path), then use SW buffer for overflow
inline void uart$_putbuf(uint8_t* buf, uint16_t len)
{
    uart_hw_t* hw = uart_get_hw(UART$_UART_INST);
    uint16_t i = 0;

    // fast path: if SW buffer is empty, write directly to HW FIFO
    if (uart$_txwritepos == uart$_txreadpos) {
        while (i < len && uart_is_writable(UART$_UART_INST)) {
            hw->dr = buf[i++];
        }
    }

    // remaining bytes go to software buffer
    for (; i < len; i++) {
        uint16_t next = (uart$_txwritepos + 1) & UART$_TXBUFSIZEMASK;
        // spin wait if buffer full
        while (uart$_txreadpos == next) {
            // enable TX IRQ to drain
            hw_set_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
        }
        uart$_txbuf[next] = buf[i];
        uart$_txwritepos = next;
    }

    // enable TX IRQ if we have data in SW buffer
    if (uart$_txwritepos != uart$_txreadpos) {
        hw_set_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
    }
}

inline uint16_t uart$_tx_notfull(void)
{
    uint16_t next = (uart$_txwritepos + 1) & UART$_TXBUFSIZEMASK;
    return (uart$_txreadpos != next) ? 1 : 0;
}

inline void uart$_tx_flush(void)
{
    // wait for software buffer to drain
    while (uart$_txwritepos != uart$_txreadpos) {}
    // wait for HW FIFO to empty
    while (!(uart_get_hw(UART$_UART_INST)->fr & UART_UARTFR_TXFE_BITS)) {}
}

#elif defined(UART$_IS_PIO_SERIAL)

// non-blocking single char - returns 0 if buffer full
inline uint16_t uart$_putc(char c)
{
    uint16_t next = (uart$_txwritepos + 1) & UART$_TXBUFSIZEMASK;
    if (uart$_txreadpos != next) {  // not full
        uart$_txbuf[next] = c;
        uart$_txwritepos = next;
        // enable TX IRQ
        UART$_PIO_SET_IRQ_SOURCE_ENABLED(UART$_PIO_INST,
            (pio_interrupt_source)(pis_sm0_tx_fifo_not_full + uart$_pio_sm_tx), true);
        return 1;
    }
    return 0;
}

// buffer write - uses SW buffer with IRQ drain
inline void uart$_putbuf(uint8_t* buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (uart$_txwritepos + 1) & UART$_TXBUFSIZEMASK;
        // spin wait if buffer full
        while (uart$_txreadpos == next) {
            // enable TX IRQ to drain
            UART$_PIO_SET_IRQ_SOURCE_ENABLED(UART$_PIO_INST,
                (pio_interrupt_source)(pis_sm0_tx_fifo_not_full + uart$_pio_sm_tx), true);
        }
        uart$_txbuf[next] = buf[i];
        uart$_txwritepos = next;
    }

    // enable TX IRQ to start draining
    if (uart$_txwritepos != uart$_txreadpos) {
        UART$_PIO_SET_IRQ_SOURCE_ENABLED(UART$_PIO_INST,
            (pio_interrupt_source)(pis_sm0_tx_fifo_not_full + uart$_pio_sm_tx), true);
    }
}

inline uint16_t uart$_tx_notfull(void)
{
    uint16_t next = (uart$_txwritepos + 1) & UART$_TXBUFSIZEMASK;
    return (uart$_txreadpos != next) ? 1 : 0;
}

inline void uart$_tx_flush(void)
{
    // wait for software buffer to drain
    while (uart$_txwritepos != uart$_txreadpos) {}
    // wait for PIO FIFO to empty
    while (!pio_sm_is_tx_fifo_empty(UART$_PIO_INST, uart$_pio_sm_tx)) {}
}

#else  // USB serial

inline void uart$_putbuf(uint8_t* buf, uint16_t len)
{
    UART$_SERIAL_NO.write((uint8_t*)buf, len);
}

inline uint16_t uart$_tx_notfull(void)
{
    return (UART$_SERIAL_NO.availableForWrite() > 0) ? 1 : 0;
}

inline void uart$_tx_flush(void)
{
    UART$_SERIAL_NO.flush();
}

#endif


//-------------------------------------------------------
// RX routines
//-------------------------------------------------------

#if defined(UART$_IS_HW_SERIAL) || defined(UART$_IS_PIO_SERIAL)

inline char uart$_getc(void)
{
    if (uart$_rxwritepos == uart$_rxreadpos) return 0;  // empty
    uart$_rxreadpos = (uart$_rxreadpos + 1) & UART$_RXBUFSIZEMASK;
    return uart$_rxbuf[uart$_rxreadpos];
}

inline void uart$_rx_flush(void)
{
    uart$_rxreadpos = uart$_rxwritepos;
}

inline uint16_t uart$_rx_bytesavailable(void)
{
    int16_t count = uart$_rxwritepos - uart$_rxreadpos;
    if (count < 0) count += UART$_RXBUFSIZE;
    return count;
}

inline uint16_t uart$_rx_available(void)
{
    return (uart$_rxwritepos != uart$_rxreadpos) ? 1 : 0;
}

#else  // USB serial

inline char uart$_getc(void)
{
    return (char)UART$_SERIAL_NO.read();
}

inline void uart$_rx_flush(void)
{
    while (UART$_SERIAL_NO.available() > 0) UART$_SERIAL_NO.read();
}

inline uint16_t uart$_rx_bytesavailable(void)
{
    return (UART$_SERIAL_NO.available() > 0) ? UART$_SERIAL_NO.available() : 0;
}

inline uint16_t uart$_rx_available(void)
{
    return (UART$_SERIAL_NO.available() > 0) ? 1 : 0;
}

#endif


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

#ifdef UART$_IS_HW_SERIAL

void _uart$_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    // initialize software buffers
    uart$_txwritepos = 0;
    uart$_txreadpos = 0;
    uart$_rxwritepos = 0;
    uart$_rxreadpos = 0;

    // pure SDK initialization
    uart_init(UART$_UART_INST, baud);

    // enable FIFOs - critical for batched transfers
    uart_set_fifo_enabled(UART$_UART_INST, true);

    // set pins
    if (UART$_TX_PIN >= 0) {
        gpio_set_function(UART$_TX_PIN, GPIO_FUNC_UART);
    }
    if (UART$_RX_PIN >= 0) {
        gpio_set_function(UART$_RX_PIN, GPIO_FUNC_UART);
    }

    // set format
    uart_parity_t sdk_parity = UART_PARITY_NONE;
    if (parity == XUART_PARITY_EVEN) sdk_parity = UART_PARITY_EVEN;
    else if (parity == XUART_PARITY_ODD) sdk_parity = UART_PARITY_ODD;

    uint stop = (stopbits == UART_STOPBIT_2) ? 2 : 1;
    uart_set_format(UART$_UART_INST, 8, stop, sdk_parity);

    // inversion
    #ifdef UART$_INVERT_TX
        gpio_set_outover(UART$_TX_PIN, GPIO_OVERRIDE_INVERT);
    #endif
    #ifdef UART$_INVERT_RX
        gpio_set_inover(UART$_RX_PIN, GPIO_OVERRIDE_INVERT);
    #endif

    // set up IRQ handler
    irq_set_exclusive_handler(UART$_UART_IRQ, uart$_irq_handler);
    irq_set_enabled(UART$_UART_IRQ, true);

    // enable RX IRQ only (TX enabled on demand when data to send)
    uart$_rx_irq_enabled = true;
    uart_set_irq_enables(UART$_UART_INST, true, false);
}

void uart$_setbaudrate(uint32_t baud)
{
    uart_set_baudrate(UART$_UART_INST, baud);
}

void uart$_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    uart_set_baudrate(UART$_UART_INST, baud);
    uart_parity_t sdk_parity = UART_PARITY_NONE;
    if (parity == XUART_PARITY_EVEN) sdk_parity = UART_PARITY_EVEN;
    else if (parity == XUART_PARITY_ODD) sdk_parity = UART_PARITY_ODD;
    uint stop = (stopbits == UART_STOPBIT_2) ? 2 : 1;
    uart_set_format(UART$_UART_INST, 8, stop, sdk_parity);
}

#elif defined(UART$_IS_PIO_SERIAL)

void _uart$_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    (void)parity;    // PIO UART only supports 8N1 (see limitations above)
    (void)stopbits;

    // initialize software buffers
    uart$_txwritepos = 0;
    uart$_txreadpos = 0;
    uart$_rxwritepos = 0;
    uart$_rxreadpos = 0;

    // calculate clock divider for baud rate
    // PIO runs at 8 cycles per bit for these programs
    float div = (float)clock_get_hz(clk_sys) / (8.0f * (float)baud);

    // --- TX setup ---
    // only claim SM and configure if TX pin is valid
    if (UART$_TX_PIN >= 0) {
        uart$_pio_sm_tx = pio_claim_unused_sm(UART$_PIO_INST, true);
        static const pio_program_t tx_prog = {
            .instructions = uart$_pio_uart_tx_program,
            .length = UART$_PIO_TX_PROG_LEN,
            .origin = -1
        };
        uart$_pio_tx_offset = pio_add_program(UART$_PIO_INST, &tx_prog);

        pio_sm_config c = pio_get_default_sm_config();
        sm_config_set_wrap(&c, uart$_pio_tx_offset, uart$_pio_tx_offset + UART$_PIO_TX_PROG_LEN - 1);
        sm_config_set_out_shift(&c, true, false, 32);  // shift right, no autopull
        sm_config_set_out_pins(&c, UART$_TX_PIN, 1);
        sm_config_set_sideset_pins(&c, UART$_TX_PIN);
        sm_config_set_sideset(&c, 2, true, false);  // 1 bit sideset, optional, no pindirs
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
        sm_config_set_clkdiv(&c, div);

        pio_gpio_init(UART$_PIO_INST, UART$_TX_PIN);
        pio_sm_set_consecutive_pindirs(UART$_PIO_INST, uart$_pio_sm_tx, UART$_TX_PIN, 1, true);
        #ifdef UART$_INVERT_TX
            gpio_set_outover(UART$_TX_PIN, GPIO_OVERRIDE_INVERT);
        #endif
        pio_sm_init(UART$_PIO_INST, uart$_pio_sm_tx, uart$_pio_tx_offset, &c);
        pio_sm_set_enabled(UART$_PIO_INST, uart$_pio_sm_tx, true);
    }

    // --- RX setup ---
    // only claim SM and configure if RX pin is valid
    if (UART$_RX_PIN >= 0) {
        uart$_pio_sm_rx = pio_claim_unused_sm(UART$_PIO_INST, true);

        static const pio_program_t rx_prog = {
            .instructions = uart$_pio_uart_rx_program,
            .length = UART$_PIO_RX_PROG_LEN,
            .origin = -1
        };
        uart$_pio_rx_offset = pio_add_program(UART$_PIO_INST, &rx_prog);

        pio_sm_config c = pio_get_default_sm_config();
        sm_config_set_wrap(&c, uart$_pio_rx_offset, uart$_pio_rx_offset + UART$_PIO_RX_PROG_LEN - 1);
        sm_config_set_in_pins(&c, UART$_RX_PIN);
        sm_config_set_in_shift(&c, true, true, 8);  // shift right, autopush at 8 bits
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
        sm_config_set_clkdiv(&c, div);

        pio_gpio_init(UART$_PIO_INST, UART$_RX_PIN);
        pio_sm_set_consecutive_pindirs(UART$_PIO_INST, uart$_pio_sm_rx, UART$_RX_PIN, 1, false);
        gpio_set_pulls(UART$_RX_PIN, true, false);  // pull-up for idle high
        #ifdef UART$_INVERT_RX
            gpio_set_inover(UART$_RX_PIN, GPIO_OVERRIDE_INVERT);
        #endif
        pio_sm_init(UART$_PIO_INST, uart$_pio_sm_rx, uart$_pio_rx_offset, &c);
        pio_sm_set_enabled(UART$_PIO_INST, uart$_pio_sm_rx, true);

        // enable RX FIFO not-empty IRQ
        UART$_PIO_SET_IRQ_SOURCE_ENABLED(UART$_PIO_INST,
            (pio_interrupt_source)(pis_sm0_rx_fifo_not_empty + uart$_pio_sm_rx), true);
    }

    // set up shared IRQ handler
    irq_set_exclusive_handler(UART$_PIO_IRQ, uart$_pio_irq_handler);
    irq_set_enabled(UART$_PIO_IRQ, true);
}

void uart$_setbaudrate(uint32_t baud)
{
    float div = (float)clock_get_hz(clk_sys) / (8.0f * (float)baud);
    if (UART$_TX_PIN >= 0) {
        pio_sm_set_clkdiv(UART$_PIO_INST, uart$_pio_sm_tx, div);
    }
    if (UART$_RX_PIN >= 0) {
        pio_sm_set_clkdiv(UART$_PIO_INST, uart$_pio_sm_rx, div);
    }
}

void uart$_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    (void)parity;    // PIO UART only supports 8N1 currently
    (void)stopbits;
    uart$_setbaudrate(baud);
}

#else  // USB serial

void _uart$_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    uint32_t config = SERIAL_8N1;
    switch (parity) {
        case XUART_PARITY_NO:
            switch (stopbits) {
                case UART_STOPBIT_1: config = SERIAL_8N1; break;
                case UART_STOPBIT_2: config = SERIAL_8N2; break;
            }
            break;
        case XUART_PARITY_EVEN:
            switch (stopbits) {
                case UART_STOPBIT_1: config = SERIAL_8E1; break;
                case UART_STOPBIT_2: config = SERIAL_8E2; break;
            }
            break;
        case XUART_PARITY_ODD:
            switch (stopbits) {
                case UART_STOPBIT_1: config = SERIAL_8O1; break;
                case UART_STOPBIT_2: config = SERIAL_8O2; break;
            }
            break;
    }

    UART$_SERIAL_NO.begin(baud, config);
}

void uart$_setbaudrate(uint32_t baud)
{
    UART$_SERIAL_NO.end();
    _uart$_initit(baud, XUART_PARITY_NO, UART_STOPBIT_1);
}

void uart$_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    UART$_SERIAL_NO.end();
    _uart$_initit(baud, parity, stopbits);
}

#endif


void uart$_init_isroff(void)
{
#if defined(UART$_IS_HW_SERIAL) || defined(UART$_IS_PIO_SERIAL)
    _uart$_initit(UART$_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
#else
    // usb serial ignores baud rate, use 0 as default if not defined
    #ifndef UART$_BAUD
      #define UART$_BAUD 0
    #endif
    UART$_SERIAL_NO.end();
    _uart$_initit(UART$_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
#endif
}


void uart$_init(void)
{
    uart$_init_isroff();
}

void uart$_rx_enableisr(FunctionalState flag) {}


//-------------------------------------------------------
// System bootloader
//-------------------------------------------------------

inline uint8_t uart$_has_systemboot(void)
{
    return 0;
}


#endif // RPLIB_UART$_H
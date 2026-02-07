//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP UART
// IRQ-driven RX/TX buffering for hardware UARTs (Serial1/Serial2)
// and PIO-based UARTs with software ring buffers
//********************************************************
// 2026-02-03: refactored PIO serial to use SDK directly with IRQ-driven
// software ring buffers, bypassing blocking SerialPIO
//********************************************************
#ifndef RPLIB_UART_H
#define RPLIB_UART_H

// helper to detect PIO usage
#if defined(UART_USE_SERIALPIO1) || defined(UART_USE_SERIALPIO2)
  #define UART_IS_PIO_SERIAL
  #include <hardware/pio.h>
  #include <hardware/irq.h>
  #include <hardware/gpio.h>
  #include <hardware/clocks.h>
#endif

// helper to detect hardware UART usage - use pure SDK with IRQ
#if defined(UART_USE_SERIAL1) || defined(UART_USE_SERIAL2)
  #define UART_IS_HW_SERIAL
  #include <hardware/uart.h>
  #include <hardware/irq.h>
  #include <hardware/gpio.h>
#endif

// helper to detect USB serial
#ifdef UART_USE_SERIAL
  #define UART_IS_USB_SERIAL
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

#ifdef UART_USE_SERIAL
  #define UART_SERIAL_NO       Serial
#elif defined UART_USE_SERIAL1
  #define UART_UART_INST       uart0
  #define UART_UART_IRQ        UART0_IRQ
#elif defined UART_USE_SERIAL2
  #define UART_UART_INST       uart1
  #define UART_UART_IRQ        UART1_IRQ
#elif defined UART_USE_SERIALPIO1
  // PIO serial uses PIO0 (PIO1 is reserved for CYW43 WiFi on Pico W)
  #define UART_PIO_INST        pio0
  #define UART_PIO_IRQ         PIO0_IRQ_0
  #define UART_PIO_SET_IRQ_SOURCE_ENABLED  pio_set_irq0_source_enabled
#elif defined UART_USE_SERIALPIO2
  // PIO serial uses PIO0 (PIO1 reserved, IRQ_1 to avoid conflict with SERIALPIO1)
  #define UART_PIO_INST        pio0
  #define UART_PIO_IRQ         PIO0_IRQ_1
  #define UART_PIO_SET_IRQ_SOURCE_ENABLED  pio_set_irq1_source_enabled
#else
  #error UART serial type must be defined!
#endif


#ifndef UART_TX_PIN
  #define UART_TX_PIN          -1
#endif
#ifndef UART_RX_PIN
  #define UART_RX_PIN          -1
#endif
#ifndef UART_TXBUFSIZE
  #define UART_TXBUFSIZE       256 // MUST be 2^N
#endif
#ifndef UART_RXBUFSIZE
  #define UART_RXBUFSIZE       256 // MUST be 2^N
#endif


//-------------------------------------------------------
// Software buffers (for HW UART and PIO serial)
//-------------------------------------------------------

#if defined(UART_IS_HW_SERIAL) || defined(UART_IS_PIO_SERIAL)

#define UART_TXBUFSIZEMASK  (UART_TXBUFSIZE - 1)
#define UART_RXBUFSIZEMASK  (UART_RXBUFSIZE - 1)

// TX buffer - writepos/readpos point to LAST written/read position (STM32 pattern)
static volatile uint8_t uart_txbuf[UART_TXBUFSIZE];
static volatile uint16_t uart_txwritepos;
static volatile uint16_t uart_txreadpos;

// RX buffer
static volatile uint8_t uart_rxbuf[UART_RXBUFSIZE];
static volatile uint16_t uart_rxwritepos;
static volatile uint16_t uart_rxreadpos;

#endif


//-------------------------------------------------------
// PIO serial resources and programs
//-------------------------------------------------------
// LIMITATIONS:
// - 8N1 only (8 data bits, no parity, 1 stop bit)
// - parity and stopbits parameters are ignored
// - TX/RX inversion supported via UART_INVERT_TX/UART_INVERT_RX
// - uses PIO0 (PIO1 reserved for CYW43 WiFi on Pico W)
//-------------------------------------------------------

#ifdef UART_IS_PIO_SERIAL

// PIO state machine assignments
static uint uart_pio_sm_tx;
static uint uart_pio_sm_rx;
static uint uart_pio_tx_offset;
static uint uart_pio_rx_offset;

// PIO UART TX program (8N1)
// based on pico-examples uart_tx
static const uint16_t uart_pio_uart_tx_program[] = {
    0x9fa0,  // 0: pull block side 1 [7] - stall idle high
    0xf727,  // 1: set x, 7 side 0 [7] - start bit (low)
    0x6001,  // 2: out pins, 1 - output data bit
    0x0642,  // 3: jmp x-- 2 [6] - loop for 8 bits
    0xf767,  // 4: nop side 1 [7] - stop bit, wrap to 0
};
#define UART_PIO_TX_PROG_LEN 5

// PIO UART RX program (8N1, mini version from pico-examples)
static const uint16_t uart_pio_uart_rx_program[] = {
    0x2020,  // 0: wait 0 pin 0 - wait for start bit
    0xea27,  // 1: set x, 7 [10] - preload counter, delay to first data bit
    0x4001,  // 2: in pins, 1 - sample data
    0x0642,  // 3: jmp x-- 2 [6] - 8 iterations
};
#define UART_PIO_RX_PROG_LEN 4


//-------------------------------------------------------
// PIO IRQ handler (in RAM)
//-------------------------------------------------------

void __not_in_flash_func(uart_pio_irq_handler)(void)
{
    // RX: drain PIO FIFO to software buffer
    while (!pio_sm_is_rx_fifo_empty(UART_PIO_INST, uart_pio_sm_rx)) {
        uint32_t raw = pio_sm_get(UART_PIO_INST, uart_pio_sm_rx);
        uint8_t c = (uint8_t)(raw >> 24);  // data is in upper 8 bits with autopush
        uint16_t next = (uart_rxwritepos + 1) & UART_RXBUFSIZEMASK;
        if (next != uart_rxreadpos) {  // not full
            uart_rxbuf[next] = c;
            uart_rxwritepos = next;
        }
    }

    // TX: drain software buffer to PIO FIFO
    while (!pio_sm_is_tx_fifo_full(UART_PIO_INST, uart_pio_sm_tx) &&
           (uart_txwritepos != uart_txreadpos)) {
        uart_txreadpos = (uart_txreadpos + 1) & UART_TXBUFSIZEMASK;
        pio_sm_put(UART_PIO_INST, uart_pio_sm_tx, (uint32_t)uart_txbuf[uart_txreadpos]);
    }

    // disable TX IRQ when buffer empty (prevents endless IRQ)
    if (uart_txwritepos == uart_txreadpos) {
        UART_PIO_SET_IRQ_SOURCE_ENABLED(UART_PIO_INST,
            (pio_interrupt_source)(pis_sm0_tx_fifo_not_full + uart_pio_sm_tx), false);
    }
}

#endif // UART_IS_PIO_SERIAL


//-------------------------------------------------------
// HW UART IRQ handler (in RAM)
//-------------------------------------------------------

#ifdef UART_IS_HW_SERIAL

// track if RX IRQ is enabled
static volatile bool uart_rx_irq_enabled = true;

void __not_in_flash_func(uart_irq_handler)(void)
{
    uart_hw_t* hw = uart_get_hw(UART_UART_INST);

    // clear RX interrupt flags (write-to-clear)
    hw->icr = UART_UARTICR_RTIC_BITS | UART_UARTICR_RXIC_BITS;

    // RX: drain HW FIFO to software buffer
    while (uart_is_readable(UART_UART_INST)) {
        uint32_t dr = hw->dr;
        if (dr & 0x700) continue;  // discard framing/parity/break errors
        uint16_t next = (uart_rxwritepos + 1) & UART_RXBUFSIZEMASK;
        if (next != uart_rxreadpos) {  // not full
            uart_rxbuf[next] = dr & 0xFF;
            uart_rxwritepos = next;
        }
    }

    // TX: drain software buffer to HW FIFO
    while (uart_is_writable(UART_UART_INST) && (uart_txwritepos != uart_txreadpos)) {
        uart_txreadpos = (uart_txreadpos + 1) & UART_TXBUFSIZEMASK;
        hw->dr = uart_txbuf[uart_txreadpos];
    }

    // disable TX IRQ when buffer empty (prevents endless IRQ)
    if (uart_txwritepos == uart_txreadpos) {
        hw_clear_bits(&uart_get_hw(UART_UART_INST)->imsc, UART_UARTIMSC_TXIM_BITS);
    }
}

#endif // UART_IS_HW_SERIAL


//-------------------------------------------------------
// TX routines
//-------------------------------------------------------

#ifdef UART_IS_HW_SERIAL

// non-blocking single char - returns 0 if buffer full
inline uint16_t uart_putc(char c)
{
    uint16_t next = (uart_txwritepos + 1) & UART_TXBUFSIZEMASK;
    if (uart_txreadpos != next) {  // not full
        uart_txbuf[next] = c;
        uart_txwritepos = next;
        // enable TX IRQ using direct register access (faster than uart_set_irq_enables)
        hw_set_bits(&uart_get_hw(UART_UART_INST)->imsc, UART_UARTIMSC_TXIM_BITS);
        return 1;
    }
    return 0;
}

// blocking buffer write
// hybrid: fill HW FIFO directly first (fast path), then use SW buffer for overflow
inline void uart_putbuf(uint8_t* buf, uint16_t len)
{
    uart_hw_t* hw = uart_get_hw(UART_UART_INST);
    uint16_t i = 0;

    // fast path: if SW buffer is empty, write directly to HW FIFO
    if (uart_txwritepos == uart_txreadpos) {
        while (i < len && uart_is_writable(UART_UART_INST)) {
            hw->dr = buf[i++];
        }
    }

    // remaining bytes go to software buffer
    for (; i < len; i++) {
        uint16_t next = (uart_txwritepos + 1) & UART_TXBUFSIZEMASK;
        // spin wait if buffer full
        while (uart_txreadpos == next) {
            // enable TX IRQ to drain
            hw_set_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
        }
        uart_txbuf[next] = buf[i];
        uart_txwritepos = next;
    }

    // enable TX IRQ if we have data in SW buffer
    if (uart_txwritepos != uart_txreadpos) {
        hw_set_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
    }
}

inline uint16_t uart_tx_notfull(void)
{
    uint16_t next = (uart_txwritepos + 1) & UART_TXBUFSIZEMASK;
    return (uart_txreadpos != next) ? 1 : 0;
}

inline void uart_tx_flush(void)
{
    // wait for software buffer to drain
    while (uart_txwritepos != uart_txreadpos) {}
    // wait for HW FIFO to empty
    while (!(uart_get_hw(UART_UART_INST)->fr & UART_UARTFR_TXFE_BITS)) {}
}

#elif defined(UART_IS_PIO_SERIAL)

// non-blocking single char - returns 0 if buffer full
inline uint16_t uart_putc(char c)
{
    uint16_t next = (uart_txwritepos + 1) & UART_TXBUFSIZEMASK;
    if (uart_txreadpos != next) {  // not full
        uart_txbuf[next] = c;
        uart_txwritepos = next;
        // enable TX IRQ
        UART_PIO_SET_IRQ_SOURCE_ENABLED(UART_PIO_INST,
            (pio_interrupt_source)(pis_sm0_tx_fifo_not_full + uart_pio_sm_tx), true);
        return 1;
    }
    return 0;
}

// buffer write - uses SW buffer with IRQ drain
inline void uart_putbuf(uint8_t* buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (uart_txwritepos + 1) & UART_TXBUFSIZEMASK;
        // spin wait if buffer full
        while (uart_txreadpos == next) {
            // enable TX IRQ to drain
            UART_PIO_SET_IRQ_SOURCE_ENABLED(UART_PIO_INST,
                (pio_interrupt_source)(pis_sm0_tx_fifo_not_full + uart_pio_sm_tx), true);
        }
        uart_txbuf[next] = buf[i];
        uart_txwritepos = next;
    }

    // enable TX IRQ to start draining
    if (uart_txwritepos != uart_txreadpos) {
        UART_PIO_SET_IRQ_SOURCE_ENABLED(UART_PIO_INST,
            (pio_interrupt_source)(pis_sm0_tx_fifo_not_full + uart_pio_sm_tx), true);
    }
}

inline uint16_t uart_tx_notfull(void)
{
    uint16_t next = (uart_txwritepos + 1) & UART_TXBUFSIZEMASK;
    return (uart_txreadpos != next) ? 1 : 0;
}

inline void uart_tx_flush(void)
{
    // wait for software buffer to drain
    while (uart_txwritepos != uart_txreadpos) {}
    // wait for PIO FIFO to empty
    while (!pio_sm_is_tx_fifo_empty(UART_PIO_INST, uart_pio_sm_tx)) {}
}

#else  // USB serial

inline void uart_putbuf(uint8_t* buf, uint16_t len)
{
    UART_SERIAL_NO.write((uint8_t*)buf, len);
}

inline uint16_t uart_tx_notfull(void)
{
    return (UART_SERIAL_NO.availableForWrite() > 0) ? 1 : 0;
}

inline void uart_tx_flush(void)
{
    UART_SERIAL_NO.flush();
}

#endif


//-------------------------------------------------------
// RX routines
//-------------------------------------------------------

#if defined(UART_IS_HW_SERIAL) || defined(UART_IS_PIO_SERIAL)

inline char uart_getc(void)
{
    if (uart_rxwritepos == uart_rxreadpos) return 0;  // empty
    uart_rxreadpos = (uart_rxreadpos + 1) & UART_RXBUFSIZEMASK;
    return uart_rxbuf[uart_rxreadpos];
}

inline void uart_rx_flush(void)
{
    uart_rxreadpos = uart_rxwritepos;
}

inline uint16_t uart_rx_bytesavailable(void)
{
    int16_t count = uart_rxwritepos - uart_rxreadpos;
    if (count < 0) count += UART_RXBUFSIZE;
    return count;
}

inline uint16_t uart_rx_available(void)
{
    return (uart_rxwritepos != uart_rxreadpos) ? 1 : 0;
}

#else  // USB serial

inline char uart_getc(void)
{
    return (char)UART_SERIAL_NO.read();
}

inline void uart_rx_flush(void)
{
    while (UART_SERIAL_NO.available() > 0) UART_SERIAL_NO.read();
}

inline uint16_t uart_rx_bytesavailable(void)
{
    return (UART_SERIAL_NO.available() > 0) ? UART_SERIAL_NO.available() : 0;
}

inline uint16_t uart_rx_available(void)
{
    return (UART_SERIAL_NO.available() > 0) ? 1 : 0;
}

#endif


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

#ifdef UART_IS_HW_SERIAL

void _uart_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    // initialize software buffers
    uart_txwritepos = 0;
    uart_txreadpos = 0;
    uart_rxwritepos = 0;
    uart_rxreadpos = 0;

    // pure SDK initialization
    uart_init(UART_UART_INST, baud);

    // enable FIFOs - critical for batched transfers
    uart_set_fifo_enabled(UART_UART_INST, true);

    // set pins
    if (UART_TX_PIN >= 0) {
        gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    }
    if (UART_RX_PIN >= 0) {
        gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    }

    // set format
    uart_parity_t sdk_parity = UART_PARITY_NONE;
    if (parity == XUART_PARITY_EVEN) sdk_parity = UART_PARITY_EVEN;
    else if (parity == XUART_PARITY_ODD) sdk_parity = UART_PARITY_ODD;

    uint stop = (stopbits == UART_STOPBIT_2) ? 2 : 1;
    uart_set_format(UART_UART_INST, 8, stop, sdk_parity);

    // inversion
    #ifdef UART_INVERT_TX
        gpio_set_outover(UART_TX_PIN, GPIO_OVERRIDE_INVERT);
    #endif
    #ifdef UART_INVERT_RX
        gpio_set_inover(UART_RX_PIN, GPIO_OVERRIDE_INVERT);
    #endif

    // set up IRQ handler
    irq_set_exclusive_handler(UART_UART_IRQ, uart_irq_handler);
    irq_set_enabled(UART_UART_IRQ, true);

    // enable RX IRQ only (TX enabled on demand when data to send)
    uart_rx_irq_enabled = true;
    uart_set_irq_enables(UART_UART_INST, true, false);
}

void uart_setbaudrate(uint32_t baud)
{
    uart_set_baudrate(UART_UART_INST, baud);
}

void uart_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    uart_set_baudrate(UART_UART_INST, baud);
    uart_parity_t sdk_parity = UART_PARITY_NONE;
    if (parity == XUART_PARITY_EVEN) sdk_parity = UART_PARITY_EVEN;
    else if (parity == XUART_PARITY_ODD) sdk_parity = UART_PARITY_ODD;
    uint stop = (stopbits == UART_STOPBIT_2) ? 2 : 1;
    uart_set_format(UART_UART_INST, 8, stop, sdk_parity);
}

#elif defined(UART_IS_PIO_SERIAL)

void _uart_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    (void)parity;    // PIO UART only supports 8N1 (see limitations above)
    (void)stopbits;

    // initialize software buffers
    uart_txwritepos = 0;
    uart_txreadpos = 0;
    uart_rxwritepos = 0;
    uart_rxreadpos = 0;

    // calculate clock divider for baud rate
    // PIO runs at 8 cycles per bit for these programs
    float div = (float)clock_get_hz(clk_sys) / (8.0f * (float)baud);

    // --- TX setup ---
    // only claim SM and configure if TX pin is valid
    if (UART_TX_PIN >= 0) {
        uart_pio_sm_tx = pio_claim_unused_sm(UART_PIO_INST, true);
        static const pio_program_t tx_prog = {
            .instructions = uart_pio_uart_tx_program,
            .length = UART_PIO_TX_PROG_LEN,
            .origin = -1
        };
        uart_pio_tx_offset = pio_add_program(UART_PIO_INST, &tx_prog);

        pio_sm_config c = pio_get_default_sm_config();
        sm_config_set_wrap(&c, uart_pio_tx_offset, uart_pio_tx_offset + UART_PIO_TX_PROG_LEN - 1);
        sm_config_set_out_shift(&c, true, false, 32);  // shift right, no autopull
        sm_config_set_out_pins(&c, UART_TX_PIN, 1);
        sm_config_set_sideset_pins(&c, UART_TX_PIN);
        sm_config_set_sideset(&c, 2, true, false);  // 1 bit sideset, optional, no pindirs
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
        sm_config_set_clkdiv(&c, div);

        pio_gpio_init(UART_PIO_INST, UART_TX_PIN);
        pio_sm_set_consecutive_pindirs(UART_PIO_INST, uart_pio_sm_tx, UART_TX_PIN, 1, true);
        #ifdef UART_INVERT_TX
            gpio_set_outover(UART_TX_PIN, GPIO_OVERRIDE_INVERT);
        #endif
        pio_sm_init(UART_PIO_INST, uart_pio_sm_tx, uart_pio_tx_offset, &c);
        pio_sm_set_enabled(UART_PIO_INST, uart_pio_sm_tx, true);
    }

    // --- RX setup ---
    // only claim SM and configure if RX pin is valid
    if (UART_RX_PIN >= 0) {
        uart_pio_sm_rx = pio_claim_unused_sm(UART_PIO_INST, true);

        static const pio_program_t rx_prog = {
            .instructions = uart_pio_uart_rx_program,
            .length = UART_PIO_RX_PROG_LEN,
            .origin = -1
        };
        uart_pio_rx_offset = pio_add_program(UART_PIO_INST, &rx_prog);

        pio_sm_config c = pio_get_default_sm_config();
        sm_config_set_wrap(&c, uart_pio_rx_offset, uart_pio_rx_offset + UART_PIO_RX_PROG_LEN - 1);
        sm_config_set_in_pins(&c, UART_RX_PIN);
        sm_config_set_in_shift(&c, true, true, 8);  // shift right, autopush at 8 bits
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
        sm_config_set_clkdiv(&c, div);

        pio_gpio_init(UART_PIO_INST, UART_RX_PIN);
        pio_sm_set_consecutive_pindirs(UART_PIO_INST, uart_pio_sm_rx, UART_RX_PIN, 1, false);
        gpio_set_pulls(UART_RX_PIN, true, false);  // pull-up for idle high
        #ifdef UART_INVERT_RX
            gpio_set_inover(UART_RX_PIN, GPIO_OVERRIDE_INVERT);
        #endif
        pio_sm_init(UART_PIO_INST, uart_pio_sm_rx, uart_pio_rx_offset, &c);
        pio_sm_set_enabled(UART_PIO_INST, uart_pio_sm_rx, true);

        // enable RX FIFO not-empty IRQ
        UART_PIO_SET_IRQ_SOURCE_ENABLED(UART_PIO_INST,
            (pio_interrupt_source)(pis_sm0_rx_fifo_not_empty + uart_pio_sm_rx), true);
    }

    // set up shared IRQ handler
    irq_set_exclusive_handler(UART_PIO_IRQ, uart_pio_irq_handler);
    irq_set_enabled(UART_PIO_IRQ, true);
}

void uart_setbaudrate(uint32_t baud)
{
    float div = (float)clock_get_hz(clk_sys) / (8.0f * (float)baud);
    if (UART_TX_PIN >= 0) {
        pio_sm_set_clkdiv(UART_PIO_INST, uart_pio_sm_tx, div);
    }
    if (UART_RX_PIN >= 0) {
        pio_sm_set_clkdiv(UART_PIO_INST, uart_pio_sm_rx, div);
    }
}

void uart_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    (void)parity;    // PIO UART only supports 8N1 currently
    (void)stopbits;
    uart_setbaudrate(baud);
}

#else  // USB serial

void _uart_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
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

    UART_SERIAL_NO.begin(baud, config);
}

void uart_setbaudrate(uint32_t baud)
{
    UART_SERIAL_NO.end();
    _uart_initit(baud, XUART_PARITY_NO, UART_STOPBIT_1);
}

void uart_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    UART_SERIAL_NO.end();
    _uart_initit(baud, parity, stopbits);
}

#endif


void uart_init_isroff(void)
{
#if defined(UART_IS_HW_SERIAL) || defined(UART_IS_PIO_SERIAL)
    _uart_initit(UART_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
#else
    // usb serial ignores baud rate, use 0 as default if not defined
    #ifndef UART_BAUD
      #define UART_BAUD 0
    #endif
    UART_SERIAL_NO.end();
    _uart_initit(UART_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
#endif
}


void uart_init(void)
{
    uart_init_isroff();
}

void uart_rx_enableisr(FunctionalState flag) {}


//-------------------------------------------------------
// System bootloader
//-------------------------------------------------------

inline uint8_t uart_has_systemboot(void)
{
    return 0;
}


#endif // RPLIB_UART_H
//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP UARTD
// IRQ-driven RX/TX buffering for hardware UARTs (Serial1/Serial2)
// and PIO-based UARTs with software ring buffers
//********************************************************
// 2026-02-03: refactored PIO serial to use SDK directly with IRQ-driven
// software ring buffers, bypassing blocking SerialPIO
//********************************************************
#ifndef RPLIB_UARTD_H
#define RPLIB_UARTD_H

// helper to detect PIO usage
#if defined(UARTD_USE_SERIALPIO1) || defined(UARTD_USE_SERIALPIO2)
  #define UARTD_IS_PIO_SERIAL
  #include <hardware/pio.h>
  #include <hardware/irq.h>
  #include <hardware/gpio.h>
  #include <hardware/clocks.h>
#endif

// helper to detect hardware UART usage - use pure SDK with IRQ
#if defined(UARTD_USE_SERIAL1) || defined(UARTD_USE_SERIAL2)
  #define UARTD_IS_HW_SERIAL
  #include <hardware/uart.h>
  #include <hardware/irq.h>
  #include <hardware/gpio.h>
#endif

// helper to detect USB serial
#ifdef UARTD_USE_SERIAL
  #define UARTD_IS_USB_SERIAL
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

#ifdef UARTD_USE_SERIAL
  #define UARTD_SERIAL_NO       Serial
#elif defined UARTD_USE_SERIAL1
  #define UARTD_UART_INST       uart0
  #define UARTD_UART_IRQ        UART0_IRQ
#elif defined UARTD_USE_SERIAL2
  #define UARTD_UART_INST       uart1
  #define UARTD_UART_IRQ        UART1_IRQ
#elif defined UARTD_USE_SERIALPIO1
  // PIO serial uses PIO0 (PIO1 is reserved for CYW43 WiFi on Pico W)
  #define UARTD_PIO_INST        pio0
  #define UARTD_PIO_IRQ         PIO0_IRQ_0
  #define UARTD_PIO_SET_IRQ_SOURCE_ENABLED  pio_set_irq0_source_enabled
#elif defined UARTD_USE_SERIALPIO2
  // PIO serial uses PIO0 (PIO1 reserved, IRQ_1 to avoid conflict with SERIALPIO1)
  #define UARTD_PIO_INST        pio0
  #define UARTD_PIO_IRQ         PIO0_IRQ_1
  #define UARTD_PIO_SET_IRQ_SOURCE_ENABLED  pio_set_irq1_source_enabled
#else
  #error UARTD serial type must be defined!
#endif


#ifndef UARTD_TX_PIN
  #define UARTD_TX_PIN          -1
#endif
#ifndef UARTD_RX_PIN
  #define UARTD_RX_PIN          -1
#endif
#ifndef UARTD_TXBUFSIZE
  #define UARTD_TXBUFSIZE       256 // MUST be 2^N
#endif
#ifndef UARTD_RXBUFSIZE
  #define UARTD_RXBUFSIZE       256 // MUST be 2^N
#endif


//-------------------------------------------------------
// Software buffers (for HW UART and PIO serial)
//-------------------------------------------------------

#if defined(UARTD_IS_HW_SERIAL) || defined(UARTD_IS_PIO_SERIAL)

#define UARTD_TXBUFSIZEMASK  (UARTD_TXBUFSIZE - 1)
#define UARTD_RXBUFSIZEMASK  (UARTD_RXBUFSIZE - 1)

// TX buffer - writepos/readpos point to LAST written/read position (STM32 pattern)
static volatile uint8_t uartd_txbuf[UARTD_TXBUFSIZE];
static volatile uint16_t uartd_txwritepos;
static volatile uint16_t uartd_txreadpos;

// RX buffer
static volatile uint8_t uartd_rxbuf[UARTD_RXBUFSIZE];
static volatile uint16_t uartd_rxwritepos;
static volatile uint16_t uartd_rxreadpos;

#endif


//-------------------------------------------------------
// PIO serial resources and programs
//-------------------------------------------------------
// LIMITATIONS:
// - 8N1 only (8 data bits, no parity, 1 stop bit)
// - parity and stopbits parameters are ignored
// - TX/RX inversion supported via UARTD_INVERT_TX/UARTD_INVERT_RX
// - uses PIO0 (PIO1 reserved for CYW43 WiFi on Pico W)
//-------------------------------------------------------

#ifdef UARTD_IS_PIO_SERIAL

// PIO state machine assignments
static uint uartd_pio_sm_tx;
static uint uartd_pio_sm_rx;
static uint uartd_pio_tx_offset;
static uint uartd_pio_rx_offset;

// PIO UART TX program (8N1)
// based on pico-examples uart_tx
static const uint16_t uartd_pio_uart_tx_program[] = {
    0x9fa0,  // 0: pull block side 1 [7] - stall idle high
    0xf727,  // 1: set x, 7 side 0 [7] - start bit (low)
    0x6001,  // 2: out pins, 1 - output data bit
    0x0642,  // 3: jmp x-- 2 [6] - loop for 8 bits
    0xf767,  // 4: nop side 1 [7] - stop bit, wrap to 0
};
#define UARTD_PIO_TX_PROG_LEN 5

// PIO UART RX program (8N1, mini version from pico-examples)
static const uint16_t uartd_pio_uart_rx_program[] = {
    0x2020,  // 0: wait 0 pin 0 - wait for start bit
    0xea27,  // 1: set x, 7 [10] - preload counter, delay to first data bit
    0x4001,  // 2: in pins, 1 - sample data
    0x0642,  // 3: jmp x-- 2 [6] - 8 iterations
};
#define UARTD_PIO_RX_PROG_LEN 4


//-------------------------------------------------------
// PIO IRQ handler (in RAM)
//-------------------------------------------------------

void __not_in_flash_func(uartd_pio_irq_handler)(void)
{
    // RX: drain PIO FIFO to software buffer
    while (!pio_sm_is_rx_fifo_empty(UARTD_PIO_INST, uartd_pio_sm_rx)) {
        uint32_t raw = pio_sm_get(UARTD_PIO_INST, uartd_pio_sm_rx);
        uint8_t c = (uint8_t)(raw >> 24);  // data is in upper 8 bits with autopush
        uint16_t next = (uartd_rxwritepos + 1) & UARTD_RXBUFSIZEMASK;
        if (next != uartd_rxreadpos) {  // not full
            uartd_rxbuf[next] = c;
            uartd_rxwritepos = next;
        }
    }

    // TX: drain software buffer to PIO FIFO
    while (!pio_sm_is_tx_fifo_full(UARTD_PIO_INST, uartd_pio_sm_tx) &&
           (uartd_txwritepos != uartd_txreadpos)) {
        uartd_txreadpos = (uartd_txreadpos + 1) & UARTD_TXBUFSIZEMASK;
        pio_sm_put(UARTD_PIO_INST, uartd_pio_sm_tx, (uint32_t)uartd_txbuf[uartd_txreadpos]);
    }

    // disable TX IRQ when buffer empty (prevents endless IRQ)
    if (uartd_txwritepos == uartd_txreadpos) {
        UARTD_PIO_SET_IRQ_SOURCE_ENABLED(UARTD_PIO_INST,
            (pio_interrupt_source)(pis_sm0_tx_fifo_not_full + uartd_pio_sm_tx), false);
    }
}

#endif // UARTD_IS_PIO_SERIAL


//-------------------------------------------------------
// HW UART IRQ handler (in RAM)
//-------------------------------------------------------

#ifdef UARTD_IS_HW_SERIAL

// track if RX IRQ is enabled
static volatile bool uartd_rx_irq_enabled = true;

void __not_in_flash_func(uartd_irq_handler)(void)
{
    uart_hw_t* hw = uart_get_hw(UARTD_UART_INST);

    // clear RX interrupt flags (write-to-clear)
    hw->icr = UART_UARTICR_RTIC_BITS | UART_UARTICR_RXIC_BITS;

    // RX: drain HW FIFO to software buffer
    while (uart_is_readable(UARTD_UART_INST)) {
        uint32_t dr = hw->dr;
        if (dr & 0x700) continue;  // discard framing/parity/break errors
        uint16_t next = (uartd_rxwritepos + 1) & UARTD_RXBUFSIZEMASK;
        if (next != uartd_rxreadpos) {  // not full
            uartd_rxbuf[next] = dr & 0xFF;
            uartd_rxwritepos = next;
        }
    }

    // TX: drain software buffer to HW FIFO
    while (uart_is_writable(UARTD_UART_INST) && (uartd_txwritepos != uartd_txreadpos)) {
        uartd_txreadpos = (uartd_txreadpos + 1) & UARTD_TXBUFSIZEMASK;
        hw->dr = uartd_txbuf[uartd_txreadpos];
    }

    // disable TX IRQ when buffer empty (prevents endless IRQ)
    if (uartd_txwritepos == uartd_txreadpos) {
        hw_clear_bits(&uart_get_hw(UARTD_UART_INST)->imsc, UART_UARTIMSC_TXIM_BITS);
    }
}

#endif // UARTD_IS_HW_SERIAL


//-------------------------------------------------------
// TX routines
//-------------------------------------------------------

#ifdef UARTD_IS_HW_SERIAL

// non-blocking single char - returns 0 if buffer full
inline uint16_t uartd_putc(char c)
{
    uint16_t next = (uartd_txwritepos + 1) & UARTD_TXBUFSIZEMASK;
    if (uartd_txreadpos != next) {  // not full
        uartd_txbuf[next] = c;
        uartd_txwritepos = next;
        // enable TX IRQ using direct register access (faster than uart_set_irq_enables)
        hw_set_bits(&uart_get_hw(UARTD_UART_INST)->imsc, UART_UARTIMSC_TXIM_BITS);
        return 1;
    }
    return 0;
}

// blocking buffer write
// hybrid: fill HW FIFO directly first (fast path), then use SW buffer for overflow
inline void uartd_putbuf(uint8_t* buf, uint16_t len)
{
    uart_hw_t* hw = uart_get_hw(UARTD_UART_INST);
    uint16_t i = 0;

    // fast path: if SW buffer is empty, write directly to HW FIFO
    if (uartd_txwritepos == uartd_txreadpos) {
        while (i < len && uart_is_writable(UARTD_UART_INST)) {
            hw->dr = buf[i++];
        }
    }

    // remaining bytes go to software buffer
    for (; i < len; i++) {
        uint16_t next = (uartd_txwritepos + 1) & UARTD_TXBUFSIZEMASK;
        // spin wait if buffer full
        while (uartd_txreadpos == next) {
            // enable TX IRQ to drain
            hw_set_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
        }
        uartd_txbuf[next] = buf[i];
        uartd_txwritepos = next;
    }

    // enable TX IRQ if we have data in SW buffer
    if (uartd_txwritepos != uartd_txreadpos) {
        hw_set_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
    }
}

inline uint16_t uartd_tx_notfull(void)
{
    uint16_t next = (uartd_txwritepos + 1) & UARTD_TXBUFSIZEMASK;
    return (uartd_txreadpos != next) ? 1 : 0;
}

inline void uartd_tx_flush(void)
{
    // wait for software buffer to drain
    while (uartd_txwritepos != uartd_txreadpos) {}
    // wait for HW FIFO to empty
    while (!(uart_get_hw(UARTD_UART_INST)->fr & UART_UARTFR_TXFE_BITS)) {}
}

#elif defined(UARTD_IS_PIO_SERIAL)

// non-blocking single char - returns 0 if buffer full
inline uint16_t uartd_putc(char c)
{
    uint16_t next = (uartd_txwritepos + 1) & UARTD_TXBUFSIZEMASK;
    if (uartd_txreadpos != next) {  // not full
        uartd_txbuf[next] = c;
        uartd_txwritepos = next;
        // enable TX IRQ
        UARTD_PIO_SET_IRQ_SOURCE_ENABLED(UARTD_PIO_INST,
            (pio_interrupt_source)(pis_sm0_tx_fifo_not_full + uartd_pio_sm_tx), true);
        return 1;
    }
    return 0;
}

// buffer write - uses SW buffer with IRQ drain
inline void uartd_putbuf(uint8_t* buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (uartd_txwritepos + 1) & UARTD_TXBUFSIZEMASK;
        // spin wait if buffer full
        while (uartd_txreadpos == next) {
            // enable TX IRQ to drain
            UARTD_PIO_SET_IRQ_SOURCE_ENABLED(UARTD_PIO_INST,
                (pio_interrupt_source)(pis_sm0_tx_fifo_not_full + uartd_pio_sm_tx), true);
        }
        uartd_txbuf[next] = buf[i];
        uartd_txwritepos = next;
    }

    // enable TX IRQ to start draining
    if (uartd_txwritepos != uartd_txreadpos) {
        UARTD_PIO_SET_IRQ_SOURCE_ENABLED(UARTD_PIO_INST,
            (pio_interrupt_source)(pis_sm0_tx_fifo_not_full + uartd_pio_sm_tx), true);
    }
}

inline uint16_t uartd_tx_notfull(void)
{
    uint16_t next = (uartd_txwritepos + 1) & UARTD_TXBUFSIZEMASK;
    return (uartd_txreadpos != next) ? 1 : 0;
}

inline void uartd_tx_flush(void)
{
    // wait for software buffer to drain
    while (uartd_txwritepos != uartd_txreadpos) {}
    // wait for PIO FIFO to empty
    while (!pio_sm_is_tx_fifo_empty(UARTD_PIO_INST, uartd_pio_sm_tx)) {}
}

#else  // USB serial

inline void uartd_putbuf(uint8_t* buf, uint16_t len)
{
    UARTD_SERIAL_NO.write((uint8_t*)buf, len);
}

inline uint16_t uartd_tx_notfull(void)
{
    return (UARTD_SERIAL_NO.availableForWrite() > 0) ? 1 : 0;
}

inline void uartd_tx_flush(void)
{
    UARTD_SERIAL_NO.flush();
}

#endif


//-------------------------------------------------------
// RX routines
//-------------------------------------------------------

#if defined(UARTD_IS_HW_SERIAL) || defined(UARTD_IS_PIO_SERIAL)

inline char uartd_getc(void)
{
    if (uartd_rxwritepos == uartd_rxreadpos) return 0;  // empty
    uartd_rxreadpos = (uartd_rxreadpos + 1) & UARTD_RXBUFSIZEMASK;
    return uartd_rxbuf[uartd_rxreadpos];
}

inline void uartd_rx_flush(void)
{
    uartd_rxreadpos = uartd_rxwritepos;
}

inline uint16_t uartd_rx_bytesavailable(void)
{
    int16_t count = uartd_rxwritepos - uartd_rxreadpos;
    if (count < 0) count += UARTD_RXBUFSIZE;
    return count;
}

inline uint16_t uartd_rx_available(void)
{
    return (uartd_rxwritepos != uartd_rxreadpos) ? 1 : 0;
}

#else  // USB serial

inline char uartd_getc(void)
{
    return (char)UARTD_SERIAL_NO.read();
}

inline void uartd_rx_flush(void)
{
    while (UARTD_SERIAL_NO.available() > 0) UARTD_SERIAL_NO.read();
}

inline uint16_t uartd_rx_bytesavailable(void)
{
    return (UARTD_SERIAL_NO.available() > 0) ? UARTD_SERIAL_NO.available() : 0;
}

inline uint16_t uartd_rx_available(void)
{
    return (UARTD_SERIAL_NO.available() > 0) ? 1 : 0;
}

#endif


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

#ifdef UARTD_IS_HW_SERIAL

void _uartd_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    // initialize software buffers
    uartd_txwritepos = 0;
    uartd_txreadpos = 0;
    uartd_rxwritepos = 0;
    uartd_rxreadpos = 0;

    // pure SDK initialization
    uart_init(UARTD_UART_INST, baud);

    // enable FIFOs - critical for batched transfers
    uart_set_fifo_enabled(UARTD_UART_INST, true);

    // set pins
    if (UARTD_TX_PIN >= 0) {
        gpio_set_function(UARTD_TX_PIN, GPIO_FUNC_UART);
    }
    if (UARTD_RX_PIN >= 0) {
        gpio_set_function(UARTD_RX_PIN, GPIO_FUNC_UART);
    }

    // set format
    uart_parity_t sdk_parity = UART_PARITY_NONE;
    if (parity == XUART_PARITY_EVEN) sdk_parity = UART_PARITY_EVEN;
    else if (parity == XUART_PARITY_ODD) sdk_parity = UART_PARITY_ODD;

    uint stop = (stopbits == UART_STOPBIT_2) ? 2 : 1;
    uart_set_format(UARTD_UART_INST, 8, stop, sdk_parity);

    // inversion
    #ifdef UARTD_INVERT_TX
        gpio_set_outover(UARTD_TX_PIN, GPIO_OVERRIDE_INVERT);
    #endif
    #ifdef UARTD_INVERT_RX
        gpio_set_inover(UARTD_RX_PIN, GPIO_OVERRIDE_INVERT);
    #endif

    // set up IRQ handler
    irq_set_exclusive_handler(UARTD_UART_IRQ, uartd_irq_handler);
    irq_set_enabled(UARTD_UART_IRQ, true);

    // enable RX IRQ only (TX enabled on demand when data to send)
    uartd_rx_irq_enabled = true;
    uart_set_irq_enables(UARTD_UART_INST, true, false);
}

void uartd_setbaudrate(uint32_t baud)
{
    uart_set_baudrate(UARTD_UART_INST, baud);
}

void uartd_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    uart_set_baudrate(UARTD_UART_INST, baud);
    uart_parity_t sdk_parity = UART_PARITY_NONE;
    if (parity == XUART_PARITY_EVEN) sdk_parity = UART_PARITY_EVEN;
    else if (parity == XUART_PARITY_ODD) sdk_parity = UART_PARITY_ODD;
    uint stop = (stopbits == UART_STOPBIT_2) ? 2 : 1;
    uart_set_format(UARTD_UART_INST, 8, stop, sdk_parity);
}

#elif defined(UARTD_IS_PIO_SERIAL)

void _uartd_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    (void)parity;    // PIO UART only supports 8N1 (see limitations above)
    (void)stopbits;

    // initialize software buffers
    uartd_txwritepos = 0;
    uartd_txreadpos = 0;
    uartd_rxwritepos = 0;
    uartd_rxreadpos = 0;

    // calculate clock divider for baud rate
    // PIO runs at 8 cycles per bit for these programs
    float div = (float)clock_get_hz(clk_sys) / (8.0f * (float)baud);

    // --- TX setup ---
    // only claim SM and configure if TX pin is valid
    if (UARTD_TX_PIN >= 0) {
        uartd_pio_sm_tx = pio_claim_unused_sm(UARTD_PIO_INST, true);
        static const pio_program_t tx_prog = {
            .instructions = uartd_pio_uart_tx_program,
            .length = UARTD_PIO_TX_PROG_LEN,
            .origin = -1
        };
        uartd_pio_tx_offset = pio_add_program(UARTD_PIO_INST, &tx_prog);

        pio_sm_config c = pio_get_default_sm_config();
        sm_config_set_wrap(&c, uartd_pio_tx_offset, uartd_pio_tx_offset + UARTD_PIO_TX_PROG_LEN - 1);
        sm_config_set_out_shift(&c, true, false, 32);  // shift right, no autopull
        sm_config_set_out_pins(&c, UARTD_TX_PIN, 1);
        sm_config_set_sideset_pins(&c, UARTD_TX_PIN);
        sm_config_set_sideset(&c, 2, true, false);  // 1 bit sideset, optional, no pindirs
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
        sm_config_set_clkdiv(&c, div);

        pio_gpio_init(UARTD_PIO_INST, UARTD_TX_PIN);
        pio_sm_set_consecutive_pindirs(UARTD_PIO_INST, uartd_pio_sm_tx, UARTD_TX_PIN, 1, true);
        #ifdef UARTD_INVERT_TX
            gpio_set_outover(UARTD_TX_PIN, GPIO_OVERRIDE_INVERT);
        #endif
        pio_sm_init(UARTD_PIO_INST, uartd_pio_sm_tx, uartd_pio_tx_offset, &c);
        pio_sm_set_enabled(UARTD_PIO_INST, uartd_pio_sm_tx, true);
    }

    // --- RX setup ---
    // only claim SM and configure if RX pin is valid
    if (UARTD_RX_PIN >= 0) {
        uartd_pio_sm_rx = pio_claim_unused_sm(UARTD_PIO_INST, true);

        static const pio_program_t rx_prog = {
            .instructions = uartd_pio_uart_rx_program,
            .length = UARTD_PIO_RX_PROG_LEN,
            .origin = -1
        };
        uartd_pio_rx_offset = pio_add_program(UARTD_PIO_INST, &rx_prog);

        pio_sm_config c = pio_get_default_sm_config();
        sm_config_set_wrap(&c, uartd_pio_rx_offset, uartd_pio_rx_offset + UARTD_PIO_RX_PROG_LEN - 1);
        sm_config_set_in_pins(&c, UARTD_RX_PIN);
        sm_config_set_in_shift(&c, true, true, 8);  // shift right, autopush at 8 bits
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
        sm_config_set_clkdiv(&c, div);

        pio_gpio_init(UARTD_PIO_INST, UARTD_RX_PIN);
        pio_sm_set_consecutive_pindirs(UARTD_PIO_INST, uartd_pio_sm_rx, UARTD_RX_PIN, 1, false);
        gpio_set_pulls(UARTD_RX_PIN, true, false);  // pull-up for idle high
        #ifdef UARTD_INVERT_RX
            gpio_set_inover(UARTD_RX_PIN, GPIO_OVERRIDE_INVERT);
        #endif
        pio_sm_init(UARTD_PIO_INST, uartd_pio_sm_rx, uartd_pio_rx_offset, &c);
        pio_sm_set_enabled(UARTD_PIO_INST, uartd_pio_sm_rx, true);

        // enable RX FIFO not-empty IRQ
        UARTD_PIO_SET_IRQ_SOURCE_ENABLED(UARTD_PIO_INST,
            (pio_interrupt_source)(pis_sm0_rx_fifo_not_empty + uartd_pio_sm_rx), true);
    }

    // set up shared IRQ handler
    irq_set_exclusive_handler(UARTD_PIO_IRQ, uartd_pio_irq_handler);
    irq_set_enabled(UARTD_PIO_IRQ, true);
}

void uartd_setbaudrate(uint32_t baud)
{
    float div = (float)clock_get_hz(clk_sys) / (8.0f * (float)baud);
    if (UARTD_TX_PIN >= 0) {
        pio_sm_set_clkdiv(UARTD_PIO_INST, uartd_pio_sm_tx, div);
    }
    if (UARTD_RX_PIN >= 0) {
        pio_sm_set_clkdiv(UARTD_PIO_INST, uartd_pio_sm_rx, div);
    }
}

void uartd_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    (void)parity;    // PIO UART only supports 8N1 currently
    (void)stopbits;
    uartd_setbaudrate(baud);
}

#else  // USB serial

void _uartd_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
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

    UARTD_SERIAL_NO.begin(baud, config);
}

void uartd_setbaudrate(uint32_t baud)
{
    UARTD_SERIAL_NO.end();
    _uartd_initit(baud, XUART_PARITY_NO, UART_STOPBIT_1);
}

void uartd_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    UARTD_SERIAL_NO.end();
    _uartd_initit(baud, parity, stopbits);
}

#endif


void uartd_init_isroff(void)
{
#if defined(UARTD_IS_HW_SERIAL) || defined(UARTD_IS_PIO_SERIAL)
    _uartd_initit(UARTD_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
#else
    // usb serial ignores baud rate, use 0 as default if not defined
    #ifndef UARTD_BAUD
      #define UARTD_BAUD 0
    #endif
    UARTD_SERIAL_NO.end();
    _uartd_initit(UARTD_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
#endif
}


void uartd_init(void)
{
    uartd_init_isroff();
}

void uartd_rx_enableisr(FunctionalState flag) {}


//-------------------------------------------------------
// System bootloader
//-------------------------------------------------------

inline uint8_t uartd_has_systemboot(void)
{
    return 0;
}


#endif // RPLIB_UARTD_H
//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP UARTC
// IRQ-driven RX/TX buffering for hardware UARTs (Serial1/Serial2)
// and PIO-based UARTs with software ring buffers
//********************************************************
// 2026-02-03: refactored PIO serial to use SDK directly with IRQ-driven
// software ring buffers, bypassing blocking SerialPIO
//********************************************************
#ifndef RPLIB_UARTC_H
#define RPLIB_UARTC_H

// helper to detect PIO usage
#if defined(UARTC_USE_SERIALPIO1) || defined(UARTC_USE_SERIALPIO2)
  #define UARTC_IS_PIO_SERIAL
  #include <hardware/pio.h>
  #include <hardware/irq.h>
  #include <hardware/gpio.h>
  #include <hardware/clocks.h>
#endif

// helper to detect hardware UART usage - use pure SDK with IRQ
#if defined(UARTC_USE_SERIAL1) || defined(UARTC_USE_SERIAL2)
  #define UARTC_IS_HW_SERIAL
  #include <hardware/uart.h>
  #include <hardware/irq.h>
  #include <hardware/gpio.h>
#endif

// helper to detect USB serial
#ifdef UARTC_USE_SERIAL
  #define UARTC_IS_USB_SERIAL
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

#ifdef UARTC_USE_SERIAL
  #define UARTC_SERIAL_NO       Serial
#elif defined UARTC_USE_SERIAL1
  #define UARTC_UART_INST       uart0
  #define UARTC_UART_IRQ        UART0_IRQ
#elif defined UARTC_USE_SERIAL2
  #define UARTC_UART_INST       uart1
  #define UARTC_UART_IRQ        UART1_IRQ
#elif defined UARTC_USE_SERIALPIO1
  // PIO serial uses PIO0 (PIO1 is reserved for CYW43 WiFi on Pico W)
  #define UARTC_PIO_INST        pio0
  #define UARTC_PIO_IRQ         PIO0_IRQ_0
  #define UARTC_PIO_SET_IRQ_SOURCE_ENABLED  pio_set_irq0_source_enabled
#elif defined UARTC_USE_SERIALPIO2
  // PIO serial uses PIO0 (PIO1 reserved, IRQ_1 to avoid conflict with SERIALPIO1)
  #define UARTC_PIO_INST        pio0
  #define UARTC_PIO_IRQ         PIO0_IRQ_1
  #define UARTC_PIO_SET_IRQ_SOURCE_ENABLED  pio_set_irq1_source_enabled
#else
  #error UARTC serial type must be defined!
#endif


#ifndef UARTC_TX_PIN
  #define UARTC_TX_PIN          -1
#endif
#ifndef UARTC_RX_PIN
  #define UARTC_RX_PIN          -1
#endif
#ifndef UARTC_TXBUFSIZE
  #define UARTC_TXBUFSIZE       256 // MUST be 2^N
#endif
#ifndef UARTC_RXBUFSIZE
  #define UARTC_RXBUFSIZE       256 // MUST be 2^N
#endif


//-------------------------------------------------------
// Software buffers (for HW UART and PIO serial)
//-------------------------------------------------------

#if defined(UARTC_IS_HW_SERIAL) || defined(UARTC_IS_PIO_SERIAL)

#define UARTC_TXBUFSIZEMASK  (UARTC_TXBUFSIZE - 1)
#define UARTC_RXBUFSIZEMASK  (UARTC_RXBUFSIZE - 1)

// TX buffer - writepos/readpos point to LAST written/read position (STM32 pattern)
static volatile uint8_t uartc_txbuf[UARTC_TXBUFSIZE];
static volatile uint16_t uartc_txwritepos;
static volatile uint16_t uartc_txreadpos;

// RX buffer
static volatile uint8_t uartc_rxbuf[UARTC_RXBUFSIZE];
static volatile uint16_t uartc_rxwritepos;
static volatile uint16_t uartc_rxreadpos;

#endif


//-------------------------------------------------------
// PIO serial resources and programs
//-------------------------------------------------------
// LIMITATIONS:
// - 8N1 only (8 data bits, no parity, 1 stop bit)
// - parity and stopbits parameters are ignored
// - TX/RX inversion supported via UARTC_INVERT_TX/UARTC_INVERT_RX
// - uses PIO0 (PIO1 reserved for CYW43 WiFi on Pico W)
//-------------------------------------------------------

#ifdef UARTC_IS_PIO_SERIAL

// PIO state machine assignments
static uint uartc_pio_sm_tx;
static uint uartc_pio_sm_rx;
static uint uartc_pio_tx_offset;
static uint uartc_pio_rx_offset;

// PIO UART TX program (8N1)
// based on pico-examples uart_tx
static const uint16_t uartc_pio_uart_tx_program[] = {
    0x9fa0,  // 0: pull block side 1 [7] - stall idle high
    0xf727,  // 1: set x, 7 side 0 [7] - start bit (low)
    0x6001,  // 2: out pins, 1 - output data bit
    0x0642,  // 3: jmp x-- 2 [6] - loop for 8 bits
    0xf767,  // 4: nop side 1 [7] - stop bit, wrap to 0
};
#define UARTC_PIO_TX_PROG_LEN 5

// PIO UART RX program (8N1, mini version from pico-examples)
static const uint16_t uartc_pio_uart_rx_program[] = {
    0x2020,  // 0: wait 0 pin 0 - wait for start bit
    0xea27,  // 1: set x, 7 [10] - preload counter, delay to first data bit
    0x4001,  // 2: in pins, 1 - sample data
    0x0642,  // 3: jmp x-- 2 [6] - 8 iterations
};
#define UARTC_PIO_RX_PROG_LEN 4


//-------------------------------------------------------
// PIO IRQ handler (in RAM)
//-------------------------------------------------------

void __not_in_flash_func(uartc_pio_irq_handler)(void)
{
    // RX: drain PIO FIFO to software buffer
    while (!pio_sm_is_rx_fifo_empty(UARTC_PIO_INST, uartc_pio_sm_rx)) {
        uint32_t raw = pio_sm_get(UARTC_PIO_INST, uartc_pio_sm_rx);
        uint8_t c = (uint8_t)(raw >> 24);  // data is in upper 8 bits with autopush
        uint16_t next = (uartc_rxwritepos + 1) & UARTC_RXBUFSIZEMASK;
        if (next != uartc_rxreadpos) {  // not full
            uartc_rxbuf[next] = c;
            uartc_rxwritepos = next;
        }
    }

    // TX: drain software buffer to PIO FIFO
    while (!pio_sm_is_tx_fifo_full(UARTC_PIO_INST, uartc_pio_sm_tx) &&
           (uartc_txwritepos != uartc_txreadpos)) {
        uartc_txreadpos = (uartc_txreadpos + 1) & UARTC_TXBUFSIZEMASK;
        pio_sm_put(UARTC_PIO_INST, uartc_pio_sm_tx, (uint32_t)uartc_txbuf[uartc_txreadpos]);
    }

    // disable TX IRQ when buffer empty (prevents endless IRQ)
    if (uartc_txwritepos == uartc_txreadpos) {
        UARTC_PIO_SET_IRQ_SOURCE_ENABLED(UARTC_PIO_INST,
            (pio_interrupt_source)(pis_sm0_tx_fifo_not_full + uartc_pio_sm_tx), false);
    }
}

#endif // UARTC_IS_PIO_SERIAL


//-------------------------------------------------------
// HW UART IRQ handler (in RAM)
//-------------------------------------------------------

#ifdef UARTC_IS_HW_SERIAL

// track if RX IRQ is enabled
static volatile bool uartc_rx_irq_enabled = true;

void __not_in_flash_func(uartc_irq_handler)(void)
{
    uart_hw_t* hw = uart_get_hw(UARTC_UART_INST);

    // clear RX interrupt flags (write-to-clear)
    hw->icr = UART_UARTICR_RTIC_BITS | UART_UARTICR_RXIC_BITS;

    // RX: drain HW FIFO to software buffer
    while (uart_is_readable(UARTC_UART_INST)) {
        uint32_t dr = hw->dr;
        if (dr & 0x700) continue;  // discard framing/parity/break errors
        uint16_t next = (uartc_rxwritepos + 1) & UARTC_RXBUFSIZEMASK;
        if (next != uartc_rxreadpos) {  // not full
            uartc_rxbuf[next] = dr & 0xFF;
            uartc_rxwritepos = next;
        }
    }

    // TX: drain software buffer to HW FIFO
    while (uart_is_writable(UARTC_UART_INST) && (uartc_txwritepos != uartc_txreadpos)) {
        uartc_txreadpos = (uartc_txreadpos + 1) & UARTC_TXBUFSIZEMASK;
        hw->dr = uartc_txbuf[uartc_txreadpos];
    }

    // disable TX IRQ when buffer empty (prevents endless IRQ)
    if (uartc_txwritepos == uartc_txreadpos) {
        hw_clear_bits(&uart_get_hw(UARTC_UART_INST)->imsc, UART_UARTIMSC_TXIM_BITS);
    }
}

#endif // UARTC_IS_HW_SERIAL


//-------------------------------------------------------
// TX routines
//-------------------------------------------------------

#ifdef UARTC_IS_HW_SERIAL

// non-blocking single char - returns 0 if buffer full
inline uint16_t uartc_putc(char c)
{
    uint16_t next = (uartc_txwritepos + 1) & UARTC_TXBUFSIZEMASK;
    if (uartc_txreadpos != next) {  // not full
        uartc_txbuf[next] = c;
        uartc_txwritepos = next;
        // enable TX IRQ using direct register access (faster than uart_set_irq_enables)
        hw_set_bits(&uart_get_hw(UARTC_UART_INST)->imsc, UART_UARTIMSC_TXIM_BITS);
        return 1;
    }
    return 0;
}

// blocking buffer write
// hybrid: fill HW FIFO directly first (fast path), then use SW buffer for overflow
inline void uartc_putbuf(uint8_t* buf, uint16_t len)
{
    uart_hw_t* hw = uart_get_hw(UARTC_UART_INST);
    uint16_t i = 0;

    // fast path: if SW buffer is empty, write directly to HW FIFO
    if (uartc_txwritepos == uartc_txreadpos) {
        while (i < len && uart_is_writable(UARTC_UART_INST)) {
            hw->dr = buf[i++];
        }
    }

    // remaining bytes go to software buffer
    for (; i < len; i++) {
        uint16_t next = (uartc_txwritepos + 1) & UARTC_TXBUFSIZEMASK;
        // spin wait if buffer full
        while (uartc_txreadpos == next) {
            // enable TX IRQ to drain
            hw_set_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
        }
        uartc_txbuf[next] = buf[i];
        uartc_txwritepos = next;
    }

    // enable TX IRQ if we have data in SW buffer
    if (uartc_txwritepos != uartc_txreadpos) {
        hw_set_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
    }
}

inline uint16_t uartc_tx_notfull(void)
{
    uint16_t next = (uartc_txwritepos + 1) & UARTC_TXBUFSIZEMASK;
    return (uartc_txreadpos != next) ? 1 : 0;
}

inline void uartc_tx_flush(void)
{
    // wait for software buffer to drain
    while (uartc_txwritepos != uartc_txreadpos) {}
    // wait for HW FIFO to empty
    while (!(uart_get_hw(UARTC_UART_INST)->fr & UART_UARTFR_TXFE_BITS)) {}
}

#elif defined(UARTC_IS_PIO_SERIAL)

// non-blocking single char - returns 0 if buffer full
inline uint16_t uartc_putc(char c)
{
    uint16_t next = (uartc_txwritepos + 1) & UARTC_TXBUFSIZEMASK;
    if (uartc_txreadpos != next) {  // not full
        uartc_txbuf[next] = c;
        uartc_txwritepos = next;
        // enable TX IRQ
        UARTC_PIO_SET_IRQ_SOURCE_ENABLED(UARTC_PIO_INST,
            (pio_interrupt_source)(pis_sm0_tx_fifo_not_full + uartc_pio_sm_tx), true);
        return 1;
    }
    return 0;
}

// buffer write - uses SW buffer with IRQ drain
inline void uartc_putbuf(uint8_t* buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (uartc_txwritepos + 1) & UARTC_TXBUFSIZEMASK;
        // spin wait if buffer full
        while (uartc_txreadpos == next) {
            // enable TX IRQ to drain
            UARTC_PIO_SET_IRQ_SOURCE_ENABLED(UARTC_PIO_INST,
                (pio_interrupt_source)(pis_sm0_tx_fifo_not_full + uartc_pio_sm_tx), true);
        }
        uartc_txbuf[next] = buf[i];
        uartc_txwritepos = next;
    }

    // enable TX IRQ to start draining
    if (uartc_txwritepos != uartc_txreadpos) {
        UARTC_PIO_SET_IRQ_SOURCE_ENABLED(UARTC_PIO_INST,
            (pio_interrupt_source)(pis_sm0_tx_fifo_not_full + uartc_pio_sm_tx), true);
    }
}

inline uint16_t uartc_tx_notfull(void)
{
    uint16_t next = (uartc_txwritepos + 1) & UARTC_TXBUFSIZEMASK;
    return (uartc_txreadpos != next) ? 1 : 0;
}

inline void uartc_tx_flush(void)
{
    // wait for software buffer to drain
    while (uartc_txwritepos != uartc_txreadpos) {}
    // wait for PIO FIFO to empty
    while (!pio_sm_is_tx_fifo_empty(UARTC_PIO_INST, uartc_pio_sm_tx)) {}
}

#else  // USB serial

inline void uartc_putbuf(uint8_t* buf, uint16_t len)
{
    UARTC_SERIAL_NO.write((uint8_t*)buf, len);
}

inline uint16_t uartc_tx_notfull(void)
{
    return (UARTC_SERIAL_NO.availableForWrite() > 0) ? 1 : 0;
}

inline void uartc_tx_flush(void)
{
    UARTC_SERIAL_NO.flush();
}

#endif


//-------------------------------------------------------
// RX routines
//-------------------------------------------------------

#if defined(UARTC_IS_HW_SERIAL) || defined(UARTC_IS_PIO_SERIAL)

inline char uartc_getc(void)
{
    if (uartc_rxwritepos == uartc_rxreadpos) return 0;  // empty
    uartc_rxreadpos = (uartc_rxreadpos + 1) & UARTC_RXBUFSIZEMASK;
    return uartc_rxbuf[uartc_rxreadpos];
}

inline void uartc_rx_flush(void)
{
    uartc_rxreadpos = uartc_rxwritepos;
}

inline uint16_t uartc_rx_bytesavailable(void)
{
    int16_t count = uartc_rxwritepos - uartc_rxreadpos;
    if (count < 0) count += UARTC_RXBUFSIZE;
    return count;
}

inline uint16_t uartc_rx_available(void)
{
    return (uartc_rxwritepos != uartc_rxreadpos) ? 1 : 0;
}

#else  // USB serial

inline char uartc_getc(void)
{
    return (char)UARTC_SERIAL_NO.read();
}

inline void uartc_rx_flush(void)
{
    while (UARTC_SERIAL_NO.available() > 0) UARTC_SERIAL_NO.read();
}

inline uint16_t uartc_rx_bytesavailable(void)
{
    return (UARTC_SERIAL_NO.available() > 0) ? UARTC_SERIAL_NO.available() : 0;
}

inline uint16_t uartc_rx_available(void)
{
    return (UARTC_SERIAL_NO.available() > 0) ? 1 : 0;
}

#endif


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

#ifdef UARTC_IS_HW_SERIAL

void _uartc_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    // initialize software buffers
    uartc_txwritepos = 0;
    uartc_txreadpos = 0;
    uartc_rxwritepos = 0;
    uartc_rxreadpos = 0;

    // pure SDK initialization
    uart_init(UARTC_UART_INST, baud);

    // enable FIFOs - critical for batched transfers
    uart_set_fifo_enabled(UARTC_UART_INST, true);

    // set pins
    if (UARTC_TX_PIN >= 0) {
        gpio_set_function(UARTC_TX_PIN, GPIO_FUNC_UART);
    }
    if (UARTC_RX_PIN >= 0) {
        gpio_set_function(UARTC_RX_PIN, GPIO_FUNC_UART);
    }

    // set format
    uart_parity_t sdk_parity = UART_PARITY_NONE;
    if (parity == XUART_PARITY_EVEN) sdk_parity = UART_PARITY_EVEN;
    else if (parity == XUART_PARITY_ODD) sdk_parity = UART_PARITY_ODD;

    uint stop = (stopbits == UART_STOPBIT_2) ? 2 : 1;
    uart_set_format(UARTC_UART_INST, 8, stop, sdk_parity);

    // inversion
    #ifdef UARTC_INVERT_TX
        gpio_set_outover(UARTC_TX_PIN, GPIO_OVERRIDE_INVERT);
    #endif
    #ifdef UARTC_INVERT_RX
        gpio_set_inover(UARTC_RX_PIN, GPIO_OVERRIDE_INVERT);
    #endif

    // set up IRQ handler
    irq_set_exclusive_handler(UARTC_UART_IRQ, uartc_irq_handler);
    irq_set_enabled(UARTC_UART_IRQ, true);

    // enable RX IRQ only (TX enabled on demand when data to send)
    uartc_rx_irq_enabled = true;
    uart_set_irq_enables(UARTC_UART_INST, true, false);
}

void uartc_setbaudrate(uint32_t baud)
{
    uart_set_baudrate(UARTC_UART_INST, baud);
}

void uartc_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    uart_set_baudrate(UARTC_UART_INST, baud);
    uart_parity_t sdk_parity = UART_PARITY_NONE;
    if (parity == XUART_PARITY_EVEN) sdk_parity = UART_PARITY_EVEN;
    else if (parity == XUART_PARITY_ODD) sdk_parity = UART_PARITY_ODD;
    uint stop = (stopbits == UART_STOPBIT_2) ? 2 : 1;
    uart_set_format(UARTC_UART_INST, 8, stop, sdk_parity);
}

#elif defined(UARTC_IS_PIO_SERIAL)

void _uartc_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    (void)parity;    // PIO UART only supports 8N1 (see limitations above)
    (void)stopbits;

    // initialize software buffers
    uartc_txwritepos = 0;
    uartc_txreadpos = 0;
    uartc_rxwritepos = 0;
    uartc_rxreadpos = 0;

    // calculate clock divider for baud rate
    // PIO runs at 8 cycles per bit for these programs
    float div = (float)clock_get_hz(clk_sys) / (8.0f * (float)baud);

    // --- TX setup ---
    // only claim SM and configure if TX pin is valid
    if (UARTC_TX_PIN >= 0) {
        uartc_pio_sm_tx = pio_claim_unused_sm(UARTC_PIO_INST, true);
        static const pio_program_t tx_prog = {
            .instructions = uartc_pio_uart_tx_program,
            .length = UARTC_PIO_TX_PROG_LEN,
            .origin = -1
        };
        uartc_pio_tx_offset = pio_add_program(UARTC_PIO_INST, &tx_prog);

        pio_sm_config c = pio_get_default_sm_config();
        sm_config_set_wrap(&c, uartc_pio_tx_offset, uartc_pio_tx_offset + UARTC_PIO_TX_PROG_LEN - 1);
        sm_config_set_out_shift(&c, true, false, 32);  // shift right, no autopull
        sm_config_set_out_pins(&c, UARTC_TX_PIN, 1);
        sm_config_set_sideset_pins(&c, UARTC_TX_PIN);
        sm_config_set_sideset(&c, 2, true, false);  // 1 bit sideset, optional, no pindirs
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
        sm_config_set_clkdiv(&c, div);

        pio_gpio_init(UARTC_PIO_INST, UARTC_TX_PIN);
        pio_sm_set_consecutive_pindirs(UARTC_PIO_INST, uartc_pio_sm_tx, UARTC_TX_PIN, 1, true);
        #ifdef UARTC_INVERT_TX
            gpio_set_outover(UARTC_TX_PIN, GPIO_OVERRIDE_INVERT);
        #endif
        pio_sm_init(UARTC_PIO_INST, uartc_pio_sm_tx, uartc_pio_tx_offset, &c);
        pio_sm_set_enabled(UARTC_PIO_INST, uartc_pio_sm_tx, true);
    }

    // --- RX setup ---
    // only claim SM and configure if RX pin is valid
    if (UARTC_RX_PIN >= 0) {
        uartc_pio_sm_rx = pio_claim_unused_sm(UARTC_PIO_INST, true);

        static const pio_program_t rx_prog = {
            .instructions = uartc_pio_uart_rx_program,
            .length = UARTC_PIO_RX_PROG_LEN,
            .origin = -1
        };
        uartc_pio_rx_offset = pio_add_program(UARTC_PIO_INST, &rx_prog);

        pio_sm_config c = pio_get_default_sm_config();
        sm_config_set_wrap(&c, uartc_pio_rx_offset, uartc_pio_rx_offset + UARTC_PIO_RX_PROG_LEN - 1);
        sm_config_set_in_pins(&c, UARTC_RX_PIN);
        sm_config_set_in_shift(&c, true, true, 8);  // shift right, autopush at 8 bits
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
        sm_config_set_clkdiv(&c, div);

        pio_gpio_init(UARTC_PIO_INST, UARTC_RX_PIN);
        pio_sm_set_consecutive_pindirs(UARTC_PIO_INST, uartc_pio_sm_rx, UARTC_RX_PIN, 1, false);
        gpio_set_pulls(UARTC_RX_PIN, true, false);  // pull-up for idle high
        #ifdef UARTC_INVERT_RX
            gpio_set_inover(UARTC_RX_PIN, GPIO_OVERRIDE_INVERT);
        #endif
        pio_sm_init(UARTC_PIO_INST, uartc_pio_sm_rx, uartc_pio_rx_offset, &c);
        pio_sm_set_enabled(UARTC_PIO_INST, uartc_pio_sm_rx, true);

        // enable RX FIFO not-empty IRQ
        UARTC_PIO_SET_IRQ_SOURCE_ENABLED(UARTC_PIO_INST,
            (pio_interrupt_source)(pis_sm0_rx_fifo_not_empty + uartc_pio_sm_rx), true);
    }

    // set up shared IRQ handler
    irq_set_exclusive_handler(UARTC_PIO_IRQ, uartc_pio_irq_handler);
    irq_set_enabled(UARTC_PIO_IRQ, true);
}

void uartc_setbaudrate(uint32_t baud)
{
    float div = (float)clock_get_hz(clk_sys) / (8.0f * (float)baud);
    if (UARTC_TX_PIN >= 0) {
        pio_sm_set_clkdiv(UARTC_PIO_INST, uartc_pio_sm_tx, div);
    }
    if (UARTC_RX_PIN >= 0) {
        pio_sm_set_clkdiv(UARTC_PIO_INST, uartc_pio_sm_rx, div);
    }
}

void uartc_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    (void)parity;    // PIO UART only supports 8N1 currently
    (void)stopbits;
    uartc_setbaudrate(baud);
}

#else  // USB serial

void _uartc_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
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

    UARTC_SERIAL_NO.begin(baud, config);
}

void uartc_setbaudrate(uint32_t baud)
{
    UARTC_SERIAL_NO.end();
    _uartc_initit(baud, XUART_PARITY_NO, UART_STOPBIT_1);
}

void uartc_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    UARTC_SERIAL_NO.end();
    _uartc_initit(baud, parity, stopbits);
}

#endif


void uartc_init_isroff(void)
{
#if defined(UARTC_IS_HW_SERIAL) || defined(UARTC_IS_PIO_SERIAL)
    _uartc_initit(UARTC_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
#else
    // usb serial ignores baud rate, use 0 as default if not defined
    #ifndef UARTC_BAUD
      #define UARTC_BAUD 0
    #endif
    UARTC_SERIAL_NO.end();
    _uartc_initit(UARTC_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
#endif
}


void uartc_init(void)
{
    uartc_init_isroff();
}

void uartc_rx_enableisr(FunctionalState flag) {}


//-------------------------------------------------------
// System bootloader
//-------------------------------------------------------

inline uint8_t uartc_has_systemboot(void)
{
    return 0;
}


#endif // RPLIB_UARTC_H
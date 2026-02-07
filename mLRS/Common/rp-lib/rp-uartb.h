//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP UARTB
// IRQ-driven RX/TX buffering for hardware UARTs (Serial1/Serial2)
// and PIO-based UARTs with software ring buffers
//********************************************************
// 2026-02-03: refactored PIO serial to use SDK directly with IRQ-driven
// software ring buffers, bypassing blocking SerialPIO
//********************************************************
#ifndef RPLIB_UARTB_H
#define RPLIB_UARTB_H

// helper to detect PIO usage
#if defined(UARTB_USE_SERIALPIO1) || defined(UARTB_USE_SERIALPIO2)
  #define UARTB_IS_PIO_SERIAL
  #include <hardware/pio.h>
  #include <hardware/irq.h>
  #include <hardware/gpio.h>
  #include <hardware/clocks.h>
#endif

// helper to detect hardware UART usage - use pure SDK with IRQ
#if defined(UARTB_USE_SERIAL1) || defined(UARTB_USE_SERIAL2)
  #define UARTB_IS_HW_SERIAL
  #include <hardware/uart.h>
  #include <hardware/irq.h>
  #include <hardware/gpio.h>
#endif

// helper to detect USB serial
#ifdef UARTB_USE_SERIAL
  #define UARTB_IS_USB_SERIAL
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

#ifdef UARTB_USE_SERIAL
  #define UARTB_SERIAL_NO       Serial
#elif defined UARTB_USE_SERIAL1
  #define UARTB_UART_INST       uart0
  #define UARTB_UART_IRQ        UART0_IRQ
#elif defined UARTB_USE_SERIAL2
  #define UARTB_UART_INST       uart1
  #define UARTB_UART_IRQ        UART1_IRQ
#elif defined UARTB_USE_SERIALPIO1
  // PIO serial uses PIO0 (PIO1 is reserved for CYW43 WiFi on Pico W)
  #define UARTB_PIO_INST        pio0
  #define UARTB_PIO_IRQ         PIO0_IRQ_0
  #define UARTB_PIO_SET_IRQ_SOURCE_ENABLED  pio_set_irq0_source_enabled
#elif defined UARTB_USE_SERIALPIO2
  // PIO serial uses PIO0 (PIO1 reserved, IRQ_1 to avoid conflict with SERIALPIO1)
  #define UARTB_PIO_INST        pio0
  #define UARTB_PIO_IRQ         PIO0_IRQ_1
  #define UARTB_PIO_SET_IRQ_SOURCE_ENABLED  pio_set_irq1_source_enabled
#else
  #error UARTB serial type must be defined!
#endif


#ifndef UARTB_TX_PIN
  #define UARTB_TX_PIN          -1
#endif
#ifndef UARTB_RX_PIN
  #define UARTB_RX_PIN          -1
#endif
#ifndef UARTB_TXBUFSIZE
  #define UARTB_TXBUFSIZE       256 // MUST be 2^N
#endif
#ifndef UARTB_RXBUFSIZE
  #define UARTB_RXBUFSIZE       256 // MUST be 2^N
#endif


//-------------------------------------------------------
// Software buffers (for HW UART and PIO serial)
//-------------------------------------------------------

#if defined(UARTB_IS_HW_SERIAL) || defined(UARTB_IS_PIO_SERIAL)

#define UARTB_TXBUFSIZEMASK  (UARTB_TXBUFSIZE - 1)
#define UARTB_RXBUFSIZEMASK  (UARTB_RXBUFSIZE - 1)

// TX buffer - writepos/readpos point to LAST written/read position (STM32 pattern)
static volatile uint8_t uartb_txbuf[UARTB_TXBUFSIZE];
static volatile uint16_t uartb_txwritepos;
static volatile uint16_t uartb_txreadpos;

// RX buffer
static volatile uint8_t uartb_rxbuf[UARTB_RXBUFSIZE];
static volatile uint16_t uartb_rxwritepos;
static volatile uint16_t uartb_rxreadpos;

#endif


//-------------------------------------------------------
// PIO serial resources and programs
//-------------------------------------------------------
// LIMITATIONS:
// - 8N1 only (8 data bits, no parity, 1 stop bit)
// - parity and stopbits parameters are ignored
// - TX/RX inversion supported via UARTB_INVERT_TX/UARTB_INVERT_RX
// - uses PIO0 (PIO1 reserved for CYW43 WiFi on Pico W)
//-------------------------------------------------------

#ifdef UARTB_IS_PIO_SERIAL

// PIO state machine assignments
static uint uartb_pio_sm_tx;
static uint uartb_pio_sm_rx;
static uint uartb_pio_tx_offset;
static uint uartb_pio_rx_offset;

// PIO UART TX program (8N1)
// based on pico-examples uart_tx
static const uint16_t uartb_pio_uart_tx_program[] = {
    0x9fa0,  // 0: pull block side 1 [7] - stall idle high
    0xf727,  // 1: set x, 7 side 0 [7] - start bit (low)
    0x6001,  // 2: out pins, 1 - output data bit
    0x0642,  // 3: jmp x-- 2 [6] - loop for 8 bits
    0xf767,  // 4: nop side 1 [7] - stop bit, wrap to 0
};
#define UARTB_PIO_TX_PROG_LEN 5

// PIO UART RX program (8N1, mini version from pico-examples)
static const uint16_t uartb_pio_uart_rx_program[] = {
    0x2020,  // 0: wait 0 pin 0 - wait for start bit
    0xea27,  // 1: set x, 7 [10] - preload counter, delay to first data bit
    0x4001,  // 2: in pins, 1 - sample data
    0x0642,  // 3: jmp x-- 2 [6] - 8 iterations
};
#define UARTB_PIO_RX_PROG_LEN 4


//-------------------------------------------------------
// PIO IRQ handler (in RAM)
//-------------------------------------------------------

void __not_in_flash_func(uartb_pio_irq_handler)(void)
{
    // RX: drain PIO FIFO to software buffer
    while (!pio_sm_is_rx_fifo_empty(UARTB_PIO_INST, uartb_pio_sm_rx)) {
        uint32_t raw = pio_sm_get(UARTB_PIO_INST, uartb_pio_sm_rx);
        uint8_t c = (uint8_t)(raw >> 24);  // data is in upper 8 bits with autopush
        uint16_t next = (uartb_rxwritepos + 1) & UARTB_RXBUFSIZEMASK;
        if (next != uartb_rxreadpos) {  // not full
            uartb_rxbuf[next] = c;
            uartb_rxwritepos = next;
        }
    }

    // TX: drain software buffer to PIO FIFO
    while (!pio_sm_is_tx_fifo_full(UARTB_PIO_INST, uartb_pio_sm_tx) &&
           (uartb_txwritepos != uartb_txreadpos)) {
        uartb_txreadpos = (uartb_txreadpos + 1) & UARTB_TXBUFSIZEMASK;
        pio_sm_put(UARTB_PIO_INST, uartb_pio_sm_tx, (uint32_t)uartb_txbuf[uartb_txreadpos]);
    }

    // disable TX IRQ when buffer empty (prevents endless IRQ)
    if (uartb_txwritepos == uartb_txreadpos) {
        UARTB_PIO_SET_IRQ_SOURCE_ENABLED(UARTB_PIO_INST,
            (pio_interrupt_source)(pis_sm0_tx_fifo_not_full + uartb_pio_sm_tx), false);
    }
}

#endif // UARTB_IS_PIO_SERIAL


//-------------------------------------------------------
// HW UART IRQ handler (in RAM)
//-------------------------------------------------------

#ifdef UARTB_IS_HW_SERIAL

// track if RX IRQ is enabled
static volatile bool uartb_rx_irq_enabled = true;

void __not_in_flash_func(uartb_irq_handler)(void)
{
    uart_hw_t* hw = uart_get_hw(UARTB_UART_INST);

    // clear RX interrupt flags (write-to-clear)
    hw->icr = UART_UARTICR_RTIC_BITS | UART_UARTICR_RXIC_BITS;

    // RX: drain HW FIFO to software buffer
    while (uart_is_readable(UARTB_UART_INST)) {
        uint32_t dr = hw->dr;
        if (dr & 0x700) continue;  // discard framing/parity/break errors
        uint16_t next = (uartb_rxwritepos + 1) & UARTB_RXBUFSIZEMASK;
        if (next != uartb_rxreadpos) {  // not full
            uartb_rxbuf[next] = dr & 0xFF;
            uartb_rxwritepos = next;
        }
    }

    // TX: drain software buffer to HW FIFO
    while (uart_is_writable(UARTB_UART_INST) && (uartb_txwritepos != uartb_txreadpos)) {
        uartb_txreadpos = (uartb_txreadpos + 1) & UARTB_TXBUFSIZEMASK;
        hw->dr = uartb_txbuf[uartb_txreadpos];
    }

    // disable TX IRQ when buffer empty (prevents endless IRQ)
    if (uartb_txwritepos == uartb_txreadpos) {
        hw_clear_bits(&uart_get_hw(UARTB_UART_INST)->imsc, UART_UARTIMSC_TXIM_BITS);
    }
}

#endif // UARTB_IS_HW_SERIAL


//-------------------------------------------------------
// TX routines
//-------------------------------------------------------

#ifdef UARTB_IS_HW_SERIAL

// non-blocking single char - returns 0 if buffer full
inline uint16_t uartb_putc(char c)
{
    uint16_t next = (uartb_txwritepos + 1) & UARTB_TXBUFSIZEMASK;
    if (uartb_txreadpos != next) {  // not full
        uartb_txbuf[next] = c;
        uartb_txwritepos = next;
        // enable TX IRQ using direct register access (faster than uart_set_irq_enables)
        hw_set_bits(&uart_get_hw(UARTB_UART_INST)->imsc, UART_UARTIMSC_TXIM_BITS);
        return 1;
    }
    return 0;
}

// blocking buffer write
// hybrid: fill HW FIFO directly first (fast path), then use SW buffer for overflow
inline void uartb_putbuf(uint8_t* buf, uint16_t len)
{
    uart_hw_t* hw = uart_get_hw(UARTB_UART_INST);
    uint16_t i = 0;

    // fast path: if SW buffer is empty, write directly to HW FIFO
    if (uartb_txwritepos == uartb_txreadpos) {
        while (i < len && uart_is_writable(UARTB_UART_INST)) {
            hw->dr = buf[i++];
        }
    }

    // remaining bytes go to software buffer
    for (; i < len; i++) {
        uint16_t next = (uartb_txwritepos + 1) & UARTB_TXBUFSIZEMASK;
        // spin wait if buffer full
        while (uartb_txreadpos == next) {
            // enable TX IRQ to drain
            hw_set_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
        }
        uartb_txbuf[next] = buf[i];
        uartb_txwritepos = next;
    }

    // enable TX IRQ if we have data in SW buffer
    if (uartb_txwritepos != uartb_txreadpos) {
        hw_set_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
    }
}

inline uint16_t uartb_tx_notfull(void)
{
    uint16_t next = (uartb_txwritepos + 1) & UARTB_TXBUFSIZEMASK;
    return (uartb_txreadpos != next) ? 1 : 0;
}

inline void uartb_tx_flush(void)
{
    // wait for software buffer to drain
    while (uartb_txwritepos != uartb_txreadpos) {}
    // wait for HW FIFO to empty
    while (!(uart_get_hw(UARTB_UART_INST)->fr & UART_UARTFR_TXFE_BITS)) {}
}

#elif defined(UARTB_IS_PIO_SERIAL)

// non-blocking single char - returns 0 if buffer full
inline uint16_t uartb_putc(char c)
{
    uint16_t next = (uartb_txwritepos + 1) & UARTB_TXBUFSIZEMASK;
    if (uartb_txreadpos != next) {  // not full
        uartb_txbuf[next] = c;
        uartb_txwritepos = next;
        // enable TX IRQ
        UARTB_PIO_SET_IRQ_SOURCE_ENABLED(UARTB_PIO_INST,
            (pio_interrupt_source)(pis_sm0_tx_fifo_not_full + uartb_pio_sm_tx), true);
        return 1;
    }
    return 0;
}

// buffer write - uses SW buffer with IRQ drain
inline void uartb_putbuf(uint8_t* buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (uartb_txwritepos + 1) & UARTB_TXBUFSIZEMASK;
        // spin wait if buffer full
        while (uartb_txreadpos == next) {
            // enable TX IRQ to drain
            UARTB_PIO_SET_IRQ_SOURCE_ENABLED(UARTB_PIO_INST,
                (pio_interrupt_source)(pis_sm0_tx_fifo_not_full + uartb_pio_sm_tx), true);
        }
        uartb_txbuf[next] = buf[i];
        uartb_txwritepos = next;
    }

    // enable TX IRQ to start draining
    if (uartb_txwritepos != uartb_txreadpos) {
        UARTB_PIO_SET_IRQ_SOURCE_ENABLED(UARTB_PIO_INST,
            (pio_interrupt_source)(pis_sm0_tx_fifo_not_full + uartb_pio_sm_tx), true);
    }
}

inline uint16_t uartb_tx_notfull(void)
{
    uint16_t next = (uartb_txwritepos + 1) & UARTB_TXBUFSIZEMASK;
    return (uartb_txreadpos != next) ? 1 : 0;
}

inline void uartb_tx_flush(void)
{
    // wait for software buffer to drain
    while (uartb_txwritepos != uartb_txreadpos) {}
    // wait for PIO FIFO to empty
    while (!pio_sm_is_tx_fifo_empty(UARTB_PIO_INST, uartb_pio_sm_tx)) {}
}

#else  // USB serial

inline void uartb_putbuf(uint8_t* buf, uint16_t len)
{
    UARTB_SERIAL_NO.write((uint8_t*)buf, len);
}

inline uint16_t uartb_tx_notfull(void)
{
    return (UARTB_SERIAL_NO.availableForWrite() > 0) ? 1 : 0;
}

inline void uartb_tx_flush(void)
{
    UARTB_SERIAL_NO.flush();
}

#endif


//-------------------------------------------------------
// RX routines
//-------------------------------------------------------

#if defined(UARTB_IS_HW_SERIAL) || defined(UARTB_IS_PIO_SERIAL)

inline char uartb_getc(void)
{
    if (uartb_rxwritepos == uartb_rxreadpos) return 0;  // empty
    uartb_rxreadpos = (uartb_rxreadpos + 1) & UARTB_RXBUFSIZEMASK;
    return uartb_rxbuf[uartb_rxreadpos];
}

inline void uartb_rx_flush(void)
{
    uartb_rxreadpos = uartb_rxwritepos;
}

inline uint16_t uartb_rx_bytesavailable(void)
{
    int16_t count = uartb_rxwritepos - uartb_rxreadpos;
    if (count < 0) count += UARTB_RXBUFSIZE;
    return count;
}

inline uint16_t uartb_rx_available(void)
{
    return (uartb_rxwritepos != uartb_rxreadpos) ? 1 : 0;
}

#else  // USB serial

inline char uartb_getc(void)
{
    return (char)UARTB_SERIAL_NO.read();
}

inline void uartb_rx_flush(void)
{
    while (UARTB_SERIAL_NO.available() > 0) UARTB_SERIAL_NO.read();
}

inline uint16_t uartb_rx_bytesavailable(void)
{
    return (UARTB_SERIAL_NO.available() > 0) ? UARTB_SERIAL_NO.available() : 0;
}

inline uint16_t uartb_rx_available(void)
{
    return (UARTB_SERIAL_NO.available() > 0) ? 1 : 0;
}

#endif


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

#ifdef UARTB_IS_HW_SERIAL

void _uartb_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    // initialize software buffers
    uartb_txwritepos = 0;
    uartb_txreadpos = 0;
    uartb_rxwritepos = 0;
    uartb_rxreadpos = 0;

    // pure SDK initialization
    uart_init(UARTB_UART_INST, baud);

    // enable FIFOs - critical for batched transfers
    uart_set_fifo_enabled(UARTB_UART_INST, true);

    // set pins
    if (UARTB_TX_PIN >= 0) {
        gpio_set_function(UARTB_TX_PIN, GPIO_FUNC_UART);
    }
    if (UARTB_RX_PIN >= 0) {
        gpio_set_function(UARTB_RX_PIN, GPIO_FUNC_UART);
    }

    // set format
    uart_parity_t sdk_parity = UART_PARITY_NONE;
    if (parity == XUART_PARITY_EVEN) sdk_parity = UART_PARITY_EVEN;
    else if (parity == XUART_PARITY_ODD) sdk_parity = UART_PARITY_ODD;

    uint stop = (stopbits == UART_STOPBIT_2) ? 2 : 1;
    uart_set_format(UARTB_UART_INST, 8, stop, sdk_parity);

    // inversion
    #ifdef UARTB_INVERT_TX
        gpio_set_outover(UARTB_TX_PIN, GPIO_OVERRIDE_INVERT);
    #endif
    #ifdef UARTB_INVERT_RX
        gpio_set_inover(UARTB_RX_PIN, GPIO_OVERRIDE_INVERT);
    #endif

    // set up IRQ handler
    irq_set_exclusive_handler(UARTB_UART_IRQ, uartb_irq_handler);
    irq_set_enabled(UARTB_UART_IRQ, true);

    // enable RX IRQ only (TX enabled on demand when data to send)
    uartb_rx_irq_enabled = true;
    uart_set_irq_enables(UARTB_UART_INST, true, false);
}

void uartb_setbaudrate(uint32_t baud)
{
    uart_set_baudrate(UARTB_UART_INST, baud);
}

void uartb_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    uart_set_baudrate(UARTB_UART_INST, baud);
    uart_parity_t sdk_parity = UART_PARITY_NONE;
    if (parity == XUART_PARITY_EVEN) sdk_parity = UART_PARITY_EVEN;
    else if (parity == XUART_PARITY_ODD) sdk_parity = UART_PARITY_ODD;
    uint stop = (stopbits == UART_STOPBIT_2) ? 2 : 1;
    uart_set_format(UARTB_UART_INST, 8, stop, sdk_parity);
}

#elif defined(UARTB_IS_PIO_SERIAL)

void _uartb_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    (void)parity;    // PIO UART only supports 8N1 (see limitations above)
    (void)stopbits;

    // initialize software buffers
    uartb_txwritepos = 0;
    uartb_txreadpos = 0;
    uartb_rxwritepos = 0;
    uartb_rxreadpos = 0;

    // calculate clock divider for baud rate
    // PIO runs at 8 cycles per bit for these programs
    float div = (float)clock_get_hz(clk_sys) / (8.0f * (float)baud);

    // --- TX setup ---
    // only claim SM and configure if TX pin is valid
    if (UARTB_TX_PIN >= 0) {
        uartb_pio_sm_tx = pio_claim_unused_sm(UARTB_PIO_INST, true);
        static const pio_program_t tx_prog = {
            .instructions = uartb_pio_uart_tx_program,
            .length = UARTB_PIO_TX_PROG_LEN,
            .origin = -1
        };
        uartb_pio_tx_offset = pio_add_program(UARTB_PIO_INST, &tx_prog);

        pio_sm_config c = pio_get_default_sm_config();
        sm_config_set_wrap(&c, uartb_pio_tx_offset, uartb_pio_tx_offset + UARTB_PIO_TX_PROG_LEN - 1);
        sm_config_set_out_shift(&c, true, false, 32);  // shift right, no autopull
        sm_config_set_out_pins(&c, UARTB_TX_PIN, 1);
        sm_config_set_sideset_pins(&c, UARTB_TX_PIN);
        sm_config_set_sideset(&c, 2, true, false);  // 1 bit sideset, optional, no pindirs
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
        sm_config_set_clkdiv(&c, div);

        pio_gpio_init(UARTB_PIO_INST, UARTB_TX_PIN);
        pio_sm_set_consecutive_pindirs(UARTB_PIO_INST, uartb_pio_sm_tx, UARTB_TX_PIN, 1, true);
        #ifdef UARTB_INVERT_TX
            gpio_set_outover(UARTB_TX_PIN, GPIO_OVERRIDE_INVERT);
        #endif
        pio_sm_init(UARTB_PIO_INST, uartb_pio_sm_tx, uartb_pio_tx_offset, &c);
        pio_sm_set_enabled(UARTB_PIO_INST, uartb_pio_sm_tx, true);
    }

    // --- RX setup ---
    // only claim SM and configure if RX pin is valid
    if (UARTB_RX_PIN >= 0) {
        uartb_pio_sm_rx = pio_claim_unused_sm(UARTB_PIO_INST, true);

        static const pio_program_t rx_prog = {
            .instructions = uartb_pio_uart_rx_program,
            .length = UARTB_PIO_RX_PROG_LEN,
            .origin = -1
        };
        uartb_pio_rx_offset = pio_add_program(UARTB_PIO_INST, &rx_prog);

        pio_sm_config c = pio_get_default_sm_config();
        sm_config_set_wrap(&c, uartb_pio_rx_offset, uartb_pio_rx_offset + UARTB_PIO_RX_PROG_LEN - 1);
        sm_config_set_in_pins(&c, UARTB_RX_PIN);
        sm_config_set_in_shift(&c, true, true, 8);  // shift right, autopush at 8 bits
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
        sm_config_set_clkdiv(&c, div);

        pio_gpio_init(UARTB_PIO_INST, UARTB_RX_PIN);
        pio_sm_set_consecutive_pindirs(UARTB_PIO_INST, uartb_pio_sm_rx, UARTB_RX_PIN, 1, false);
        gpio_set_pulls(UARTB_RX_PIN, true, false);  // pull-up for idle high
        #ifdef UARTB_INVERT_RX
            gpio_set_inover(UARTB_RX_PIN, GPIO_OVERRIDE_INVERT);
        #endif
        pio_sm_init(UARTB_PIO_INST, uartb_pio_sm_rx, uartb_pio_rx_offset, &c);
        pio_sm_set_enabled(UARTB_PIO_INST, uartb_pio_sm_rx, true);

        // enable RX FIFO not-empty IRQ
        UARTB_PIO_SET_IRQ_SOURCE_ENABLED(UARTB_PIO_INST,
            (pio_interrupt_source)(pis_sm0_rx_fifo_not_empty + uartb_pio_sm_rx), true);
    }

    // set up shared IRQ handler
    irq_set_exclusive_handler(UARTB_PIO_IRQ, uartb_pio_irq_handler);
    irq_set_enabled(UARTB_PIO_IRQ, true);
}

void uartb_setbaudrate(uint32_t baud)
{
    float div = (float)clock_get_hz(clk_sys) / (8.0f * (float)baud);
    if (UARTB_TX_PIN >= 0) {
        pio_sm_set_clkdiv(UARTB_PIO_INST, uartb_pio_sm_tx, div);
    }
    if (UARTB_RX_PIN >= 0) {
        pio_sm_set_clkdiv(UARTB_PIO_INST, uartb_pio_sm_rx, div);
    }
}

void uartb_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    (void)parity;    // PIO UART only supports 8N1 currently
    (void)stopbits;
    uartb_setbaudrate(baud);
}

#else  // USB serial

void _uartb_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
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

    UARTB_SERIAL_NO.begin(baud, config);
}

void uartb_setbaudrate(uint32_t baud)
{
    UARTB_SERIAL_NO.end();
    _uartb_initit(baud, XUART_PARITY_NO, UART_STOPBIT_1);
}

void uartb_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    UARTB_SERIAL_NO.end();
    _uartb_initit(baud, parity, stopbits);
}

#endif


void uartb_init_isroff(void)
{
#if defined(UARTB_IS_HW_SERIAL) || defined(UARTB_IS_PIO_SERIAL)
    _uartb_initit(UARTB_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
#else
    // usb serial ignores baud rate, use 0 as default if not defined
    #ifndef UARTB_BAUD
      #define UARTB_BAUD 0
    #endif
    UARTB_SERIAL_NO.end();
    _uartb_initit(UARTB_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
#endif
}


void uartb_init(void)
{
    uartb_init_isroff();
}

void uartb_rx_enableisr(FunctionalState flag) {}


//-------------------------------------------------------
// System bootloader
//-------------------------------------------------------

inline uint8_t uartb_has_systemboot(void)
{
    return 0;
}


#endif // RPLIB_UARTB_H
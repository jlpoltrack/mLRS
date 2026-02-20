//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP UART$
// 12.Feb.2026
// IRQ-driven RX/TX buffering for hardware UARTs (Serial1/Serial2)
// with software ring buffers
//********************************************************
#ifndef RPLIB_UART$_H
#define RPLIB_UART$_H

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
    //  UART_STOPBIT_0_5 = 0, // not supported
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
// Software buffers
//-------------------------------------------------------

#ifdef UART$_IS_HW_SERIAL

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
// HW UART IRQ handler (in RAM)
//-------------------------------------------------------

#ifdef UART$_IS_HW_SERIAL

void __not_in_flash_func(uart$_irq_handler)(void)
{
    uart_hw_t* hw = uart_get_hw(UART$_UART_INST);

    // clear RX interrupt flags (write-to-clear)
    hw->icr = UART_UARTICR_RTIC_BITS | UART_UARTICR_RXIC_BITS;

    // RX: drain HW FIFO to software buffer
    while (!(hw->fr & UART_UARTFR_RXFE_BITS)) {
        uint32_t dr = hw->dr;
        if (dr & 0x700) continue;  // discard framing/parity/break errors
        uint16_t next = (uart$_rxwritepos + 1) & UART$_RXBUFSIZEMASK;
        if (next != uart$_rxreadpos) {  // not full
            uart$_rxbuf[next] = dr & 0xFF;
            uart$_rxwritepos = next;
        }
    }

    // TX: drain software buffer to HW FIFO
    while (!(hw->fr & UART_UARTFR_TXFF_BITS) && (uart$_txwritepos != uart$_txreadpos)) {
        uart$_txreadpos = (uart$_txreadpos + 1) & UART$_TXBUFSIZEMASK;
        hw->dr = uart$_txbuf[uart$_txreadpos];
    }

    // disable TX IRQ when buffer empty (prevents endless IRQ)
    if (uart$_txwritepos == uart$_txreadpos) {
        hw_clear_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
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

#ifdef UART$_IS_HW_SERIAL

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
    return UART$_SERIAL_NO.available();
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
#ifdef UART$_IS_HW_SERIAL
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


void uart$_init(void) { uart$_init_isroff(); }


void uart$_rx_enableisr(FunctionalState flag) {}


//-------------------------------------------------------
// System bootloader
//-------------------------------------------------------

inline uint8_t uart$_has_systemboot(void) { return 0; }


#endif // RPLIB_UART$_H
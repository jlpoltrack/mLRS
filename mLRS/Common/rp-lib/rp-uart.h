//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP UART
// 12.Feb.2026
// IRQ-driven RX/TX buffering for hardware UARTs (Serial1/Serial2)
// with software ring buffers
//********************************************************
#ifndef RPLIB_UART_H
#define RPLIB_UART_H

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
    //  UART_STOPBIT_0_5 = 0, // not supported
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
// Software buffers
//-------------------------------------------------------

#ifdef UART_IS_HW_SERIAL

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
// HW UART IRQ handler (in RAM)
//-------------------------------------------------------

#ifdef UART_IS_HW_SERIAL

void __not_in_flash_func(uart_irq_handler)(void)
{
    uart_hw_t* hw = uart_get_hw(UART_UART_INST);

    // clear RX interrupt flags (write-to-clear)
    hw->icr = UART_UARTICR_RTIC_BITS | UART_UARTICR_RXIC_BITS;

    // RX: drain HW FIFO to software buffer
    while (!(hw->fr & UART_UARTFR_RXFE_BITS)) {
        uint32_t dr = hw->dr;
        if (dr & 0x700) continue;  // discard framing/parity/break errors
        uint16_t next = (uart_rxwritepos + 1) & UART_RXBUFSIZEMASK;
        if (next != uart_rxreadpos) {  // not full
            uart_rxbuf[next] = dr & 0xFF;
            uart_rxwritepos = next;
        }
    }

    // TX: drain software buffer to HW FIFO
    while (!(hw->fr & UART_UARTFR_TXFF_BITS) && (uart_txwritepos != uart_txreadpos)) {
        uart_txreadpos = (uart_txreadpos + 1) & UART_TXBUFSIZEMASK;
        hw->dr = uart_txbuf[uart_txreadpos];
    }

    // disable TX IRQ when buffer empty (prevents endless IRQ)
    if (uart_txwritepos == uart_txreadpos) {
        hw_clear_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
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

#ifdef UART_IS_HW_SERIAL

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
    return UART_SERIAL_NO.available();
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
#ifdef UART_IS_HW_SERIAL
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


void uart_init(void) { uart_init_isroff(); }


void uart_rx_enableisr(FunctionalState flag) {}


//-------------------------------------------------------
// System bootloader
//-------------------------------------------------------

inline uint8_t uart_has_systemboot(void) { return 0; }


#endif // RPLIB_UART_H
//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP UARTC
// 12.Feb.2026
// IRQ-driven RX/TX buffering for hardware UARTs (Serial1/Serial2)
// with software ring buffers
//********************************************************
#ifndef RPLIB_UARTC_H
#define RPLIB_UARTC_H

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
    //  UART_STOPBIT_0_5 = 0, // not supported
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
// Software buffers
//-------------------------------------------------------

#ifdef UARTC_IS_HW_SERIAL

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
// HW UART IRQ handler (in RAM)
//-------------------------------------------------------

#ifdef UARTC_IS_HW_SERIAL

void __not_in_flash_func(uartc_irq_handler)(void)
{
    uart_hw_t* hw = uart_get_hw(UARTC_UART_INST);

    // clear RX interrupt flags (write-to-clear)
    hw->icr = UART_UARTICR_RTIC_BITS | UART_UARTICR_RXIC_BITS;

    // RX: drain HW FIFO to software buffer
    while (!(hw->fr & UART_UARTFR_RXFE_BITS)) {
        uint32_t dr = hw->dr;
        if (dr & 0x700) continue;  // discard framing/parity/break errors
        uint16_t next = (uartc_rxwritepos + 1) & UARTC_RXBUFSIZEMASK;
        if (next != uartc_rxreadpos) {  // not full
            uartc_rxbuf[next] = dr & 0xFF;
            uartc_rxwritepos = next;
        }
    }

    // TX: drain software buffer to HW FIFO
    while (!(hw->fr & UART_UARTFR_TXFF_BITS) && (uartc_txwritepos != uartc_txreadpos)) {
        uartc_txreadpos = (uartc_txreadpos + 1) & UARTC_TXBUFSIZEMASK;
        hw->dr = uartc_txbuf[uartc_txreadpos];
    }

    // disable TX IRQ when buffer empty (prevents endless IRQ)
    if (uartc_txwritepos == uartc_txreadpos) {
        hw_clear_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
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

#ifdef UARTC_IS_HW_SERIAL

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
    return UARTC_SERIAL_NO.available();
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
#ifdef UARTC_IS_HW_SERIAL
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


void uartc_init(void) { uartc_init_isroff(); }


void uartc_rx_enableisr(FunctionalState flag) {}


//-------------------------------------------------------
// System bootloader
//-------------------------------------------------------

inline uint8_t uartc_has_systemboot(void) { return 0; }


#endif // RPLIB_UARTC_H
//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP UARTE
// 12.Feb.2026
// IRQ-driven RX/TX buffering for hardware UARTs (Serial1/Serial2)
// with software ring buffers
//********************************************************
#ifndef RPLIB_UARTE_H
#define RPLIB_UARTE_H

// helper to detect hardware UART usage - use pure SDK with IRQ
#if defined(UARTE_USE_SERIAL1) || defined(UARTE_USE_SERIAL2)
  #define UARTE_IS_HW_SERIAL
  #include <hardware/uart.h>
  #include <hardware/irq.h>
  #include <hardware/gpio.h>
#endif

// helper to detect USB serial
#ifdef UARTE_USE_SERIAL
  #define UARTE_IS_USB_SERIAL
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

#ifdef UARTE_USE_SERIAL
  #define UARTE_SERIAL_NO       Serial
#elif defined UARTE_USE_SERIAL1
  #define UARTE_UART_INST       uart0
  #define UARTE_UART_IRQ        UART0_IRQ
#elif defined UARTE_USE_SERIAL2
  #define UARTE_UART_INST       uart1
  #define UARTE_UART_IRQ        UART1_IRQ
#else
  #error UARTE serial type must be defined!
#endif


#ifndef UARTE_TX_PIN
  #define UARTE_TX_PIN          -1
#endif
#ifndef UARTE_RX_PIN
  #define UARTE_RX_PIN          -1
#endif
#ifndef UARTE_TXBUFSIZE
  #define UARTE_TXBUFSIZE       256 // MUST be 2^N
#endif
#ifndef UARTE_RXBUFSIZE
  #define UARTE_RXBUFSIZE       256 // MUST be 2^N
#endif


//-------------------------------------------------------
// Software buffers
//-------------------------------------------------------

#ifdef UARTE_IS_HW_SERIAL

#define UARTE_TXBUFSIZEMASK  (UARTE_TXBUFSIZE - 1)
#define UARTE_RXBUFSIZEMASK  (UARTE_RXBUFSIZE - 1)

// TX buffer - writepos/readpos point to LAST written/read position (STM32 pattern)
static volatile uint8_t uarte_txbuf[UARTE_TXBUFSIZE];
static volatile uint16_t uarte_txwritepos;
static volatile uint16_t uarte_txreadpos;

// RX buffer
static volatile uint8_t uarte_rxbuf[UARTE_RXBUFSIZE];
static volatile uint16_t uarte_rxwritepos;
static volatile uint16_t uarte_rxreadpos;

#endif


//-------------------------------------------------------
// HW UART IRQ handler (in RAM)
//-------------------------------------------------------

#ifdef UARTE_IS_HW_SERIAL

void __not_in_flash_func(uarte_irq_handler)(void)
{
    uart_hw_t* hw = uart_get_hw(UARTE_UART_INST);

    // clear RX interrupt flags (write-to-clear)
    hw->icr = UART_UARTICR_RTIC_BITS | UART_UARTICR_RXIC_BITS;

    // RX: drain HW FIFO to software buffer
    while (!(hw->fr & UART_UARTFR_RXFE_BITS)) {
        uint32_t dr = hw->dr;
        if (dr & 0x700) continue;  // discard framing/parity/break errors
        uint16_t next = (uarte_rxwritepos + 1) & UARTE_RXBUFSIZEMASK;
        if (next != uarte_rxreadpos) {  // not full
            uarte_rxbuf[next] = dr & 0xFF;
            uarte_rxwritepos = next;
        }
    }

    // TX: drain software buffer to HW FIFO
    while (!(hw->fr & UART_UARTFR_TXFF_BITS) && (uarte_txwritepos != uarte_txreadpos)) {
        uarte_txreadpos = (uarte_txreadpos + 1) & UARTE_TXBUFSIZEMASK;
        hw->dr = uarte_txbuf[uarte_txreadpos];
    }

    // disable TX IRQ when buffer empty (prevents endless IRQ)
    if (uarte_txwritepos == uarte_txreadpos) {
        hw_clear_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
    }
}

#endif // UARTE_IS_HW_SERIAL


//-------------------------------------------------------
// TX routines
//-------------------------------------------------------

#ifdef UARTE_IS_HW_SERIAL

// non-blocking single char - returns 0 if buffer full
inline uint16_t uarte_putc(char c)
{
    uint16_t next = (uarte_txwritepos + 1) & UARTE_TXBUFSIZEMASK;
    if (uarte_txreadpos != next) {  // not full
        uarte_txbuf[next] = c;
        uarte_txwritepos = next;
        // enable TX IRQ using direct register access (faster than uart_set_irq_enables)
        hw_set_bits(&uart_get_hw(UARTE_UART_INST)->imsc, UART_UARTIMSC_TXIM_BITS);
        return 1;
    }
    return 0;
}


// blocking buffer write
// hybrid: fill HW FIFO directly first (fast path), then use SW buffer for overflow
inline void uarte_putbuf(uint8_t* buf, uint16_t len)
{
    uart_hw_t* hw = uart_get_hw(UARTE_UART_INST);
    uint16_t i = 0;

    // fast path: if SW buffer is empty, write directly to HW FIFO
    if (uarte_txwritepos == uarte_txreadpos) {
        while (i < len && uart_is_writable(UARTE_UART_INST)) {
            hw->dr = buf[i++];
        }
    }

    // remaining bytes go to software buffer
    for (; i < len; i++) {
        uint16_t next = (uarte_txwritepos + 1) & UARTE_TXBUFSIZEMASK;
        // spin wait if buffer full
        while (uarte_txreadpos == next) {
            // enable TX IRQ to drain
            hw_set_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
        }
        uarte_txbuf[next] = buf[i];
        uarte_txwritepos = next;
    }

    // enable TX IRQ if we have data in SW buffer
    if (uarte_txwritepos != uarte_txreadpos) {
        hw_set_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
    }
}


inline uint16_t uarte_tx_notfull(void)
{
    uint16_t next = (uarte_txwritepos + 1) & UARTE_TXBUFSIZEMASK;
    return (uarte_txreadpos != next) ? 1 : 0;
}


inline void uarte_tx_flush(void)
{
    // wait for software buffer to drain
    while (uarte_txwritepos != uarte_txreadpos) {}
    // wait for HW FIFO to empty
    while (!(uart_get_hw(UARTE_UART_INST)->fr & UART_UARTFR_TXFE_BITS)) {}
}


#else  // USB serial

inline void uarte_putbuf(uint8_t* buf, uint16_t len)
{
    UARTE_SERIAL_NO.write((uint8_t*)buf, len);
}


inline uint16_t uarte_tx_notfull(void)
{
    return (UARTE_SERIAL_NO.availableForWrite() > 0) ? 1 : 0;
}


inline void uarte_tx_flush(void)
{
    UARTE_SERIAL_NO.flush();
}


#endif


//-------------------------------------------------------
// RX routines
//-------------------------------------------------------

#ifdef UARTE_IS_HW_SERIAL

inline char uarte_getc(void)
{
    if (uarte_rxwritepos == uarte_rxreadpos) return 0;  // empty
    uarte_rxreadpos = (uarte_rxreadpos + 1) & UARTE_RXBUFSIZEMASK;
    return uarte_rxbuf[uarte_rxreadpos];
}


inline void uarte_rx_flush(void)
{
    uarte_rxreadpos = uarte_rxwritepos;
}


inline uint16_t uarte_rx_bytesavailable(void)
{
    int16_t count = uarte_rxwritepos - uarte_rxreadpos;
    if (count < 0) count += UARTE_RXBUFSIZE;
    return count;
}


inline uint16_t uarte_rx_available(void)
{
    return (uarte_rxwritepos != uarte_rxreadpos) ? 1 : 0;
}


#else  // USB serial

inline char uarte_getc(void)
{
    return (char)UARTE_SERIAL_NO.read();
}


inline void uarte_rx_flush(void)
{
    while (UARTE_SERIAL_NO.available() > 0) UARTE_SERIAL_NO.read();
}


inline uint16_t uarte_rx_bytesavailable(void)
{
    return UARTE_SERIAL_NO.available();
}


inline uint16_t uarte_rx_available(void)
{
    return (UARTE_SERIAL_NO.available() > 0) ? 1 : 0;
}


#endif


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

#ifdef UARTE_IS_HW_SERIAL

void _uarte_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    // initialize software buffers
    uarte_txwritepos = 0;
    uarte_txreadpos = 0;
    uarte_rxwritepos = 0;
    uarte_rxreadpos = 0;

    // pure SDK initialization
    uart_init(UARTE_UART_INST, baud);

    // enable FIFOs - critical for batched transfers
    uart_set_fifo_enabled(UARTE_UART_INST, true);

    // set pins
    if (UARTE_TX_PIN >= 0) {
        gpio_set_function(UARTE_TX_PIN, GPIO_FUNC_UART);
    }
    if (UARTE_RX_PIN >= 0) {
        gpio_set_function(UARTE_RX_PIN, GPIO_FUNC_UART);
    }

    // set format
    uart_parity_t sdk_parity = UART_PARITY_NONE;
    if (parity == XUART_PARITY_EVEN) sdk_parity = UART_PARITY_EVEN;
    else if (parity == XUART_PARITY_ODD) sdk_parity = UART_PARITY_ODD;

    uint stop = (stopbits == UART_STOPBIT_2) ? 2 : 1;
    uart_set_format(UARTE_UART_INST, 8, stop, sdk_parity);

    // inversion
    #ifdef UARTE_INVERT_TX
        gpio_set_outover(UARTE_TX_PIN, GPIO_OVERRIDE_INVERT);
    #endif
    #ifdef UARTE_INVERT_RX
        gpio_set_inover(UARTE_RX_PIN, GPIO_OVERRIDE_INVERT);
    #endif

    // set up IRQ handler
    irq_set_exclusive_handler(UARTE_UART_IRQ, uarte_irq_handler);
    irq_set_enabled(UARTE_UART_IRQ, true);

    // enable RX IRQ only (TX enabled on demand when data to send)
    uart_set_irq_enables(UARTE_UART_INST, true, false);
}


void uarte_setbaudrate(uint32_t baud)
{
    uart_set_baudrate(UARTE_UART_INST, baud);
}


void uarte_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    uart_set_baudrate(UARTE_UART_INST, baud);
    uart_parity_t sdk_parity = UART_PARITY_NONE;
    if (parity == XUART_PARITY_EVEN) sdk_parity = UART_PARITY_EVEN;
    else if (parity == XUART_PARITY_ODD) sdk_parity = UART_PARITY_ODD;
    uint stop = (stopbits == UART_STOPBIT_2) ? 2 : 1;
    uart_set_format(UARTE_UART_INST, 8, stop, sdk_parity);
}


#else  // USB serial

void _uarte_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
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

    UARTE_SERIAL_NO.begin(baud, config);
}


void uarte_setbaudrate(uint32_t baud)
{
    UARTE_SERIAL_NO.end();
    _uarte_initit(baud, XUART_PARITY_NO, UART_STOPBIT_1);
}


void uarte_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    UARTE_SERIAL_NO.end();
    _uarte_initit(baud, parity, stopbits);
}


#endif


void uarte_init_isroff(void)
{
#ifdef UARTE_IS_HW_SERIAL
    _uarte_initit(UARTE_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
#else
    // usb serial ignores baud rate, use 0 as default if not defined
    #ifndef UARTE_BAUD
      #define UARTE_BAUD 0
    #endif
    UARTE_SERIAL_NO.end();
    _uarte_initit(UARTE_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
#endif
}


void uarte_init(void) { uarte_init_isroff(); }


void uarte_rx_enableisr(FunctionalState flag) {}


//-------------------------------------------------------
// System bootloader
//-------------------------------------------------------

inline uint8_t uarte_has_systemboot(void) { return 0; }


#endif // RPLIB_UARTE_H
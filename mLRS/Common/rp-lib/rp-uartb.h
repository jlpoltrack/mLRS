//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP UARTB
// 12.Feb.2026
// IRQ-driven RX/TX buffering for hardware UARTs (Serial1/Serial2)
// with software ring buffers
//********************************************************
#ifndef RPLIB_UARTB_H
#define RPLIB_UARTB_H

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
    //  UART_STOPBIT_0_5 = 0, // not supported
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
// Software buffers
//-------------------------------------------------------

#ifdef UARTB_IS_HW_SERIAL

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
// HW UART IRQ handler (in RAM)
//-------------------------------------------------------

#ifdef UARTB_IS_HW_SERIAL

void __not_in_flash_func(uartb_irq_handler)(void)
{
    uart_hw_t* hw = uart_get_hw(UARTB_UART_INST);

    // clear RX interrupt flags (write-to-clear)
    hw->icr = UART_UARTICR_RTIC_BITS | UART_UARTICR_RXIC_BITS;

    // RX: drain HW FIFO to software buffer
    while (!(hw->fr & UART_UARTFR_RXFE_BITS)) {
        uint32_t dr = hw->dr;
        if (dr & 0x700) continue;  // discard framing/parity/break errors
        uint16_t next = (uartb_rxwritepos + 1) & UARTB_RXBUFSIZEMASK;
        if (next != uartb_rxreadpos) {  // not full
            uartb_rxbuf[next] = dr & 0xFF;
            uartb_rxwritepos = next;
        }
    }

    // TX: drain software buffer to HW FIFO
    while (!(hw->fr & UART_UARTFR_TXFF_BITS) && (uartb_txwritepos != uartb_txreadpos)) {
        uartb_txreadpos = (uartb_txreadpos + 1) & UARTB_TXBUFSIZEMASK;
        hw->dr = uartb_txbuf[uartb_txreadpos];
    }

    // disable TX IRQ when buffer empty (prevents endless IRQ)
    if (uartb_txwritepos == uartb_txreadpos) {
        hw_clear_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
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

#ifdef UARTB_IS_HW_SERIAL

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
    return UARTB_SERIAL_NO.available();
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
#ifdef UARTB_IS_HW_SERIAL
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


void uartb_init(void) { uartb_init_isroff(); }


void uartb_rx_enableisr(FunctionalState flag) {}


//-------------------------------------------------------
// System bootloader
//-------------------------------------------------------

inline uint8_t uartb_has_systemboot(void) { return 0; }


#endif // RPLIB_UARTB_H
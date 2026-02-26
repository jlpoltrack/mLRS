//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP UARTF
// 12.Feb.2026
// IRQ-driven RX/TX buffering for hardware UARTs (Serial1/Serial2)
// with software ring buffers
//********************************************************
#ifndef RPLIB_UARTF_H
#define RPLIB_UARTF_H

// helper to detect hardware UART usage - use pure SDK with IRQ
#if defined(UARTF_USE_SERIAL1) || defined(UARTF_USE_SERIAL2)
  #define UARTF_IS_HW_SERIAL
  #include <hardware/uart.h>
  #include <hardware/irq.h>
  #include <hardware/gpio.h>
#endif

// helper to detect USB serial
#ifdef UARTF_USE_SERIAL
  #define UARTF_IS_USB_SERIAL
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

#ifdef UARTF_USE_SERIAL
  #define UARTF_SERIAL_NO       Serial
#elif defined UARTF_USE_SERIAL1
  #define UARTF_UART_INST       uart0
  #define UARTF_UART_IRQ        UART0_IRQ
#elif defined UARTF_USE_SERIAL2
  #define UARTF_UART_INST       uart1
  #define UARTF_UART_IRQ        UART1_IRQ
#else
  #error UARTF serial type must be defined!
#endif


#ifndef UARTF_TX_PIN
  #define UARTF_TX_PIN          -1
#endif
#ifndef UARTF_RX_PIN
  #define UARTF_RX_PIN          -1
#endif
#ifndef UARTF_TXBUFSIZE
  #define UARTF_TXBUFSIZE       256 // MUST be 2^N
#endif
#ifndef UARTF_RXBUFSIZE
  #define UARTF_RXBUFSIZE       256 // MUST be 2^N
#endif


//-------------------------------------------------------
// Software buffers
//-------------------------------------------------------

#ifdef UARTF_IS_HW_SERIAL

#define UARTF_TXBUFSIZEMASK  (UARTF_TXBUFSIZE - 1)
#define UARTF_RXBUFSIZEMASK  (UARTF_RXBUFSIZE - 1)

// TX buffer - writepos/readpos point to LAST written/read position (STM32 pattern)
static volatile uint8_t uartf_txbuf[UARTF_TXBUFSIZE];
static volatile uint16_t uartf_txwritepos;
static volatile uint16_t uartf_txreadpos;

// RX buffer
static volatile uint8_t uartf_rxbuf[UARTF_RXBUFSIZE];
static volatile uint16_t uartf_rxwritepos;
static volatile uint16_t uartf_rxreadpos;

#endif


//-------------------------------------------------------
// HW UART IRQ handler (in RAM)
//-------------------------------------------------------

#ifdef UARTF_IS_HW_SERIAL

void __not_in_flash_func(uartf_irq_handler)(void)
{
    uart_hw_t* hw = uart_get_hw(UARTF_UART_INST);

    // clear RX interrupt flags (write-to-clear)
    hw->icr = UART_UARTICR_RTIC_BITS | UART_UARTICR_RXIC_BITS;

    // RX: drain HW FIFO to software buffer
    while (!(hw->fr & UART_UARTFR_RXFE_BITS)) {
        uint32_t dr = hw->dr;
        if (dr & 0x700) continue;  // discard framing/parity/break errors
        uint16_t next = (uartf_rxwritepos + 1) & UARTF_RXBUFSIZEMASK;
        if (next != uartf_rxreadpos) {  // not full
            uartf_rxbuf[next] = dr & 0xFF;
            uartf_rxwritepos = next;
        }
    }

    // TX: drain software buffer to HW FIFO
    while (!(hw->fr & UART_UARTFR_TXFF_BITS) && (uartf_txwritepos != uartf_txreadpos)) {
        uartf_txreadpos = (uartf_txreadpos + 1) & UARTF_TXBUFSIZEMASK;
        hw->dr = uartf_txbuf[uartf_txreadpos];
    }

    // disable TX IRQ when buffer empty (prevents endless IRQ)
    if (uartf_txwritepos == uartf_txreadpos) {
        hw_clear_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
    }
}

#endif // UARTF_IS_HW_SERIAL


//-------------------------------------------------------
// TX routines
//-------------------------------------------------------

#ifdef UARTF_IS_HW_SERIAL

// non-blocking single char - returns 0 if buffer full
inline uint16_t uartf_putc(char c)
{
    uint16_t next = (uartf_txwritepos + 1) & UARTF_TXBUFSIZEMASK;
    if (uartf_txreadpos != next) {  // not full
        uartf_txbuf[next] = c;
        uartf_txwritepos = next;
        // enable TX IRQ using direct register access (faster than uart_set_irq_enables)
        hw_set_bits(&uart_get_hw(UARTF_UART_INST)->imsc, UART_UARTIMSC_TXIM_BITS);
        return 1;
    }
    return 0;
}


// blocking buffer write
// hybrid: fill HW FIFO directly first (fast path), then use SW buffer for overflow
inline void uartf_putbuf(uint8_t* buf, uint16_t len)
{
    uart_hw_t* hw = uart_get_hw(UARTF_UART_INST);
    uint16_t i = 0;

    // fast path: if SW buffer is empty, write directly to HW FIFO
    if (uartf_txwritepos == uartf_txreadpos) {
        while (i < len && uart_is_writable(UARTF_UART_INST)) {
            hw->dr = buf[i++];
        }
    }

    // remaining bytes go to software buffer
    for (; i < len; i++) {
        uint16_t next = (uartf_txwritepos + 1) & UARTF_TXBUFSIZEMASK;
        // spin wait if buffer full
        while (uartf_txreadpos == next) {
            // enable TX IRQ to drain
            hw_set_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
        }
        uartf_txbuf[next] = buf[i];
        uartf_txwritepos = next;
    }

    // enable TX IRQ if we have data in SW buffer
    if (uartf_txwritepos != uartf_txreadpos) {
        hw_set_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
    }
}


inline uint16_t uartf_tx_notfull(void)
{
    uint16_t next = (uartf_txwritepos + 1) & UARTF_TXBUFSIZEMASK;
    return (uartf_txreadpos != next) ? 1 : 0;
}


inline void uartf_tx_flush(void)
{
    // wait for software buffer to drain
    while (uartf_txwritepos != uartf_txreadpos) {}
    // wait for HW FIFO to empty
    while (!(uart_get_hw(UARTF_UART_INST)->fr & UART_UARTFR_TXFE_BITS)) {}
}


#else  // USB serial

inline void uartf_putbuf(uint8_t* buf, uint16_t len)
{
    UARTF_SERIAL_NO.write((uint8_t*)buf, len);
}


inline uint16_t uartf_tx_notfull(void)
{
    return (UARTF_SERIAL_NO.availableForWrite() > 0) ? 1 : 0;
}


inline void uartf_tx_flush(void)
{
    UARTF_SERIAL_NO.flush();
}


#endif


//-------------------------------------------------------
// RX routines
//-------------------------------------------------------

#ifdef UARTF_IS_HW_SERIAL

inline char uartf_getc(void)
{
    if (uartf_rxwritepos == uartf_rxreadpos) return 0;  // empty
    uartf_rxreadpos = (uartf_rxreadpos + 1) & UARTF_RXBUFSIZEMASK;
    return uartf_rxbuf[uartf_rxreadpos];
}


inline void uartf_rx_flush(void)
{
    uartf_rxreadpos = uartf_rxwritepos;
}


inline uint16_t uartf_rx_bytesavailable(void)
{
    int16_t count = uartf_rxwritepos - uartf_rxreadpos;
    if (count < 0) count += UARTF_RXBUFSIZE;
    return count;
}


inline uint16_t uartf_rx_available(void)
{
    return (uartf_rxwritepos != uartf_rxreadpos) ? 1 : 0;
}


#else  // USB serial

inline char uartf_getc(void)
{
    return (char)UARTF_SERIAL_NO.read();
}


inline void uartf_rx_flush(void)
{
    while (UARTF_SERIAL_NO.available() > 0) UARTF_SERIAL_NO.read();
}


inline uint16_t uartf_rx_bytesavailable(void)
{
    return UARTF_SERIAL_NO.available();
}


inline uint16_t uartf_rx_available(void)
{
    return (UARTF_SERIAL_NO.available() > 0) ? 1 : 0;
}


#endif


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

#ifdef UARTF_IS_HW_SERIAL

void _uartf_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    // initialize software buffers
    uartf_txwritepos = 0;
    uartf_txreadpos = 0;
    uartf_rxwritepos = 0;
    uartf_rxreadpos = 0;

    // pure SDK initialization
    uart_init(UARTF_UART_INST, baud);

    // enable FIFOs - critical for batched transfers
    uart_set_fifo_enabled(UARTF_UART_INST, true);

    // set pins
    if (UARTF_TX_PIN >= 0) {
        gpio_set_function(UARTF_TX_PIN, GPIO_FUNC_UART);
    }
    if (UARTF_RX_PIN >= 0) {
        gpio_set_function(UARTF_RX_PIN, GPIO_FUNC_UART);
    }

    // set format
    uart_parity_t sdk_parity = UART_PARITY_NONE;
    if (parity == XUART_PARITY_EVEN) sdk_parity = UART_PARITY_EVEN;
    else if (parity == XUART_PARITY_ODD) sdk_parity = UART_PARITY_ODD;

    uint stop = (stopbits == UART_STOPBIT_2) ? 2 : 1;
    uart_set_format(UARTF_UART_INST, 8, stop, sdk_parity);

    // inversion
    #ifdef UARTF_INVERT_TX
        gpio_set_outover(UARTF_TX_PIN, GPIO_OVERRIDE_INVERT);
    #endif
    #ifdef UARTF_INVERT_RX
        gpio_set_inover(UARTF_RX_PIN, GPIO_OVERRIDE_INVERT);
    #endif

    // set up IRQ handler
    irq_set_exclusive_handler(UARTF_UART_IRQ, uartf_irq_handler);
    irq_set_enabled(UARTF_UART_IRQ, true);

    // enable RX IRQ only (TX enabled on demand when data to send)
    uart_set_irq_enables(UARTF_UART_INST, true, false);
}


void uartf_setbaudrate(uint32_t baud)
{
    uart_set_baudrate(UARTF_UART_INST, baud);
}


void uartf_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    uart_set_baudrate(UARTF_UART_INST, baud);
    uart_parity_t sdk_parity = UART_PARITY_NONE;
    if (parity == XUART_PARITY_EVEN) sdk_parity = UART_PARITY_EVEN;
    else if (parity == XUART_PARITY_ODD) sdk_parity = UART_PARITY_ODD;
    uint stop = (stopbits == UART_STOPBIT_2) ? 2 : 1;
    uart_set_format(UARTF_UART_INST, 8, stop, sdk_parity);
}


#else  // USB serial

void _uartf_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
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

    UARTF_SERIAL_NO.begin(baud, config);
}


void uartf_setbaudrate(uint32_t baud)
{
    UARTF_SERIAL_NO.end();
    _uartf_initit(baud, XUART_PARITY_NO, UART_STOPBIT_1);
}


void uartf_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    UARTF_SERIAL_NO.end();
    _uartf_initit(baud, parity, stopbits);
}


#endif


void uartf_init_isroff(void)
{
#ifdef UARTF_IS_HW_SERIAL
    _uartf_initit(UARTF_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
#else
    // usb serial ignores baud rate, use 0 as default if not defined
    #ifndef UARTF_BAUD
      #define UARTF_BAUD 0
    #endif
    UARTF_SERIAL_NO.end();
    _uartf_initit(UARTF_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
#endif
}


void uartf_init(void) { uartf_init_isroff(); }


void uartf_rx_enableisr(FunctionalState flag) {}


//-------------------------------------------------------
// System bootloader
//-------------------------------------------------------

inline uint8_t uartf_has_systemboot(void) { return 0; }


#endif // RPLIB_UARTF_H
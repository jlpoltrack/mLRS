//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP2040 UART$
//********************************************************
#ifndef RPLIB_UART$_H
#define RPLIB_UART$_H

#include <Arduino.h>

#ifndef ESPLIB_UART_ENUMS
#define ESPLIB_UART_ENUMS
typedef enum {
    XUART_PARITY_NO = 0,
    XUART_PARITY_EVEN,
    XUART_PARITY_ODD,
} UARTPARITYENUM;

typedef enum {
    UART_STOPBIT_1 = 0,
    UART_STOPBIT_2,
} UARTSTOPBITENUM;
#endif

#ifdef UART$_USE_SERIAL
  #define UART$_SERIAL_NO       Serial
#elif defined UART$_USE_SERIAL1
  #define UART$_SERIAL_NO       Serial1
#elif defined UART$_USE_SERIAL2
  #define UART$_SERIAL_NO       Serial2
#else
  #error UART$_SERIAL_NO must be defined!
#endif

//-------------------------------------------------------
// TX routines
//-------------------------------------------------------

inline void uart$_putbuf(uint8_t* buf, uint16_t len)
{
    UART$_SERIAL_NO.write((uint8_t*)buf, len);
}

inline uint16_t uart$_tx_notfull(void)
{
    return UART$_SERIAL_NO.availableForWrite() > 0;
}

inline void uart$_tx_flush(void)
{
    UART$_SERIAL_NO.flush();
}

//-------------------------------------------------------
// RX routines
//-------------------------------------------------------

inline char uart$_getc(void)
{
    return (char)UART$_SERIAL_NO.read();
}

inline void uart$_getbuf(char* buf, uint16_t len)
{
    UART$_SERIAL_NO.readBytes(buf, len);
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

//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

inline void _uart$_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    uint32_t config = SERIAL_8N1;
    switch (parity) {
        case XUART_PARITY_NO:
            config = (stopbits == UART_STOPBIT_1) ? SERIAL_8N1 : SERIAL_8N2;
            break;
        case XUART_PARITY_EVEN:
            config = (stopbits == UART_STOPBIT_1) ? SERIAL_8E1 : SERIAL_8E2;
            break;
        case XUART_PARITY_ODD:
            config = (stopbits == UART_STOPBIT_1) ? SERIAL_8O1 : SERIAL_8O2;
            break;
    }

#if defined UART$_TX_PIN && defined UART$_RX_PIN
    UART$_SERIAL_NO.setTX(UART$_TX_PIN);
    UART$_SERIAL_NO.setRX(UART$_RX_PIN);
#endif

    // Hardware inversion support for SBUS
#ifdef UART$_INVERT_TX
    UART$_SERIAL_NO.setInvertTX(true);
#endif
#ifdef UART$_INVERT_RX
    UART$_SERIAL_NO.setInvertRX(true);
#endif

    UART$_SERIAL_NO.begin(baud, config);
}

inline void uart$_setbaudrate(uint32_t baud)
{
    UART$_SERIAL_NO.end();
    _uart$_initit(baud, XUART_PARITY_NO, UART_STOPBIT_1);
}

inline void uart$_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    UART$_SERIAL_NO.end();
    _uart$_initit(baud, parity, stopbits);
}

inline void uart$_init(void)
{
    _uart$_initit(UART$_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
}

inline void uart$_rx_enableisr(FunctionalState flag)
{
    // Arduino HardwareSerial doesn't expose ISR control easily, 
    // but the background buffer handles it.
}

inline uint8_t uart$_has_systemboot(void)
{
    return 0;
}

#endif // RPLIB_UART$_H

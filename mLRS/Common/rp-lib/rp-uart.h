#define UART_USE_SERIAL
#define UART_BAUD 115200
//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP2040 UART
//********************************************************
#ifndef RPLIB_UART_H
#define RPLIB_UART_H

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

#ifdef UART_USE_SERIAL
  #define UART_SERIAL_NO       Serial
#elif defined UART_USE_SERIAL1
  #define UART_SERIAL_NO       Serial1
#elif defined UART_USE_SERIAL2
  #define UART_SERIAL_NO       Serial2
#else
  #error UART_SERIAL_NO must be defined!
#endif

//-------------------------------------------------------
// TX routines
//-------------------------------------------------------

inline void uart_putbuf(uint8_t* buf, uint16_t len)
{
    UART_SERIAL_NO.write((uint8_t*)buf, len);
}

inline uint16_t uart_tx_notfull(void)
{
    return UART_SERIAL_NO.availableForWrite() > 0;
}

inline void uart_tx_flush(void)
{
    UART_SERIAL_NO.flush();
}

//-------------------------------------------------------
// RX routines
//-------------------------------------------------------

inline char uart_getc(void)
{
    return (char)UART_SERIAL_NO.read();
}

inline void uart_getbuf(char* buf, uint16_t len)
{
    UART_SERIAL_NO.readBytes(buf, len);
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

//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

inline void _uart_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
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

#if defined UART_TX_PIN && defined UART_RX_PIN
    UART_SERIAL_NO.setTX(UART_TX_PIN);
    UART_SERIAL_NO.setRX(UART_RX_PIN);
#endif

    // Hardware inversion support for SBUS
#ifdef UART_INVERT_TX
    UART_SERIAL_NO.setInvertTX(true);
#endif
#ifdef UART_INVERT_RX
    UART_SERIAL_NO.setInvertRX(true);
#endif

    UART_SERIAL_NO.begin(baud, config);
}

inline void uart_setbaudrate(uint32_t baud)
{
    UART_SERIAL_NO.end();
    _uart_initit(baud, XUART_PARITY_NO, UART_STOPBIT_1);
}

inline void uart_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    UART_SERIAL_NO.end();
    _uart_initit(baud, parity, stopbits);
}

inline void uart_init(void)
{
    _uart_initit(UART_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
}

inline void uart_rx_enableisr(FunctionalState flag)
{
    // Arduino HardwareSerial doesn't expose ISR control easily, 
    // but the background buffer handles it.
}

inline uint8_t uart_has_systemboot(void)
{
    return 0;
}

#endif // RPLIB_UART_H

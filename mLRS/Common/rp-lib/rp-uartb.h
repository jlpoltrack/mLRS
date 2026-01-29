#define UARTB_USE_SERIAL1
#define UARTB_BAUD 115200
//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP2040 UARTB
//********************************************************
#ifndef RPLIB_UARTB_H
#define RPLIB_UARTB_H

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

#ifdef UARTB_USE_SERIAL
  #define UARTB_SERIAL_NO       Serial
#elif defined UARTB_USE_SERIAL1
  #define UARTB_SERIAL_NO       Serial1
#elif defined UARTB_USE_SERIAL2
  #define UARTB_SERIAL_NO       Serial2
#else
  #error UARTB_SERIAL_NO must be defined!
#endif

//-------------------------------------------------------
// TX routines
//-------------------------------------------------------

inline void uartb_putbuf(uint8_t* buf, uint16_t len)
{
    UARTB_SERIAL_NO.write((uint8_t*)buf, len);
}

inline uint16_t uartb_tx_notfull(void)
{
    return UARTB_SERIAL_NO.availableForWrite() > 0;
}

inline void uartb_tx_flush(void)
{
    UARTB_SERIAL_NO.flush();
}

//-------------------------------------------------------
// RX routines
//-------------------------------------------------------

inline char uartb_getc(void)
{
    return (char)UARTB_SERIAL_NO.read();
}

inline void uartb_getbuf(char* buf, uint16_t len)
{
    UARTB_SERIAL_NO.readBytes(buf, len);
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

//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

inline void _uartb_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
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

#if defined UARTB_TX_PIN && defined UARTB_RX_PIN
    UARTB_SERIAL_NO.setTX(UARTB_TX_PIN);
    UARTB_SERIAL_NO.setRX(UARTB_RX_PIN);
#endif

    // Hardware inversion support for SBUS
#ifdef UARTB_INVERT_TX
    UARTB_SERIAL_NO.setInvertTX(true);
#endif
#ifdef UARTB_INVERT_RX
    UARTB_SERIAL_NO.setInvertRX(true);
#endif

    UARTB_SERIAL_NO.begin(baud, config);
}

inline void uartb_setbaudrate(uint32_t baud)
{
    UARTB_SERIAL_NO.end();
    _uartb_initit(baud, XUART_PARITY_NO, UART_STOPBIT_1);
}

inline void uartb_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    UARTB_SERIAL_NO.end();
    _uartb_initit(baud, parity, stopbits);
}

inline void uartb_init(void)
{
    _uartb_initit(UARTB_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
}

inline void uartb_rx_enableisr(FunctionalState flag)
{
    // Arduino HardwareSerial doesn't expose ISR control easily, 
    // but the background buffer handles it.
}

inline uint8_t uartb_has_systemboot(void)
{
    return 0;
}

#endif // RPLIB_UARTB_H

//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP2040 UARTF
//********************************************************
#ifndef RPLIB_UARTF_H
#define RPLIB_UARTF_H

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

#ifdef UARTF_USE_SERIAL
  #define UARTF_SERIAL_NO       Serial
#elif defined UARTF_USE_SERIAL1
  #define UARTF_SERIAL_NO       Serial1
#elif defined UARTF_USE_SERIAL2
  #define UARTF_SERIAL_NO       Serial2
#else
  #error UARTF_SERIAL_NO must be defined!
#endif

//-------------------------------------------------------
// TX routines
//-------------------------------------------------------

inline void uartf_putbuf(uint8_t* buf, uint16_t len)
{
    UARTF_SERIAL_NO.write((uint8_t*)buf, len);
}

inline uint16_t uartf_tx_notfull(void)
{
    return UARTF_SERIAL_NO.availableForWrite() > 0;
}

inline void uartf_tx_flush(void)
{
    UARTF_SERIAL_NO.flush();
}

//-------------------------------------------------------
// RX routines
//-------------------------------------------------------

inline char uartf_getc(void)
{
    return (char)UARTF_SERIAL_NO.read();
}

inline void uartf_getbuf(char* buf, uint16_t len)
{
    UARTF_SERIAL_NO.readBytes(buf, len);
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

//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

inline void _uartf_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
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

#if defined UARTF_TX_PIN && defined UARTF_RX_PIN
    UARTF_SERIAL_NO.setTX(UARTF_TX_PIN);
    UARTF_SERIAL_NO.setRX(UARTF_RX_PIN);
#endif

    // Hardware inversion support for SBUS
#ifdef UARTF_INVERT_TX
    UARTF_SERIAL_NO.setInvertTX(true);
#endif
#ifdef UARTF_INVERT_RX
    UARTF_SERIAL_NO.setInvertRX(true);
#endif

    UARTF_SERIAL_NO.begin(baud, config);
}

inline void uartf_setbaudrate(uint32_t baud)
{
    UARTF_SERIAL_NO.end();
    _uartf_initit(baud, XUART_PARITY_NO, UART_STOPBIT_1);
}

inline void uartf_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    UARTF_SERIAL_NO.end();
    _uartf_initit(baud, parity, stopbits);
}

inline void uartf_init(void)
{
    _uartf_initit(UARTF_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
}

inline void uartf_rx_enableisr(FunctionalState flag)
{
    // Arduino HardwareSerial doesn't expose ISR control easily, 
    // but the background buffer handles it.
}

inline uint8_t uartf_has_systemboot(void)
{
    return 0;
}

#endif // RPLIB_UARTF_H

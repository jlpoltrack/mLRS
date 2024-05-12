//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP UARTF
//********************************************************
#ifndef ESPLIB_UARTF_H
#define ESPLIB_UARTF_H

#include "driver/uart.h"
#include "hal/uart_ll.h"

#ifndef ESPLIB_UART_ENUMS
#define ESPLIB_UART_ENUMS

typedef enum {
    XUART_PARITY_NO = 0,
    XUART_PARITY_EVEN,
    XUART_PARITY_ODD,
} UARTPARITYENUM;

typedef enum {
//    UART_STOPBIT_0_5 = 0, // not supported by ESP
    UART_STOPBIT_1 = 0,
    UART_STOPBIT_2,
} UARTSTOPBITENUM;

#endif


#ifdef UARTF_USE_SERIAL
#ifdef ESP32
  #define UARTF_SERIAL_NO       UART_NUM_0
  #define UARTF_SERIAL_NO_LL    UART0
#elif
  #define UARTF_SERIAL_NO       Serial
#endif
#elif defined UARTF_USE_SERIAL1
#ifdef ESP32
  #define UARTF_SERIAL_NO       UART_NUM_1
  #define UARTF_SERIAL_NO_LL    UART1
#elif
  #define UARTF_SERIAL_NO       Serial1
#endif
#elif defined UARTF_USE_SERIAL2
#ifdef ESP32
  #define UARTF_SERIAL_NO       UART_NUM_2
  #define UARTF_SERIAL_NO_LL    UART2
#endif
#else
  #error UARTF_SERIAL_NO must be defined!
#endif

#ifndef UARTF_TXBUFSIZE
  #define UARTF_TXBUFSIZE       256 // MUST be 2^N
#endif
#ifndef UARTF_RXBUFSIZE
  #define UARTF_RXBUFSIZE       256 // MUST be 2^N
#endif

#ifdef UARTF_USE_TX_ISR
  #define UARTF_TXBUFSIZEMASK  (UARTF_TXBUFSIZE-1)

  volatile char uartf_txbuf[UARTF_TXBUFSIZE];
  volatile uint16_t uartf_txwritepos; // pos at which the last byte was stored
  volatile uint16_t uartf_txreadpos; // pos at which the next byte is to be fetched

  #define UARTF_RXBUFSIZEMASK  (UARTF_RXBUFSIZE-1)

  volatile char uartf_rxbuf[UARTF_RXBUFSIZE];
  volatile uint16_t uartf_rxwritepos; // pos at which the last byte was stored
  volatile uint16_t uartf_rxreadpos; // pos at which the next byte is to be fetched
#endif


IRAM_ATTR void uartf_putbuf(uint8_t* buf, uint16_t len)
{
#ifdef ESP32
    uart_write_bytes(UARTF_SERIAL_NO, (uint8_t*)buf, len);
#elif
    UARTF_SERIAL_NO.write((uint8_t*)buf, len);
#endif
}


IRAM_ATTR char uartf_getc(void)
{
#ifdef ESP32
    uint8_t c = 0;
    uart_read_bytes(UARTF_SERIAL_NO, &c, 1, 0);
    return (char)c;
#elif
    return (char)UARTF_SERIAL_NO.read();
#endif
}


IRAM_ATTR void uartf_rx_flush(void)
{
#ifdef ESP32
    uart_flush(UARTF_SERIAL_NO);
#elif
    while (UARTF_SERIAL_NO.available() > 0) UARTF_SERIAL_NO.read();
#endif
}


IRAM_ATTR void uartf_tx_flush(void)
{
#ifdef ESP32
    uart_wait_tx_done(UARTF_SERIAL_NO, 100);  // 100 ms - what should be used?
#elif
    UARTF_SERIAL_NO.flush();
#endif
}


IRAM_ATTR uint16_t uartf_rx_bytesavailable(void)
{
#ifdef ESP32
    uint32_t bytesAvailable = 0;
    uart_get_buffered_data_len(UARTF_SERIAL_NO, &bytesAvailable);
    return (uint16_t)bytesAvailable;
#elif
    return (UARTF_SERIAL_NO.available() > 0) ? UARTF_SERIAL_NO.available() : 0;
#endif
}


IRAM_ATTR uint16_t uartf_rx_available(void)
{
#ifdef ESP32
    uint32_t bytesAvailable = 0;
    uart_get_buffered_data_len(UARTF_SERIAL_NO, &bytesAvailable);
    return ((uint16_t)bytesAvailable > 0) ? 1 : 0;
#elif
    return (UARTF_SERIAL_NO.available() > 0) ? 1 : 0;
#endif
}

IRAM_ATTR void uartf_intr_handle(void *arg)   // UART ISR
{
    uint32_t uart_intr_status = UARTF_SERIAL_NO_LL.int_st.val;

    if (uart_intr_status & UART_INTR_RXFIFO_FULL || uart_intr_status & UART_INTR_RXFIFO_TOUT) {
        char d = UARTF_SERIAL_NO_LL.fifo.rw_byte;
        UART_RX_CALLBACK_FULL(d);
        uart_clear_intr_status(UARTF_SERIAL_NO, UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);
    }
    
}

IRAM_ATTR void uartf_rx_enableisr(FunctionalState flag)
{
    // Do we need to disable the ISR in a half duplex situation?
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------
// Note: ESP32 has a hardware fifo for tx, which is 128 bytes in size. However, MAVLink messages
// can be larger than this, and data would thus be lost when put only into the fifo. It is therefore
// crucial to set a Tx buffer size of sufficient size. setTxBufferSize() is not available for ESP82xx.

void _uartf_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
#ifdef ESP32

    uart_parity_t _parity = UART_PARITY_DISABLE;
    switch (parity) {
        case XUART_PARITY_NO:
            _parity = UART_PARITY_DISABLE; break;
        case XUART_PARITY_EVEN:
            _parity = UART_PARITY_EVEN; break;        
        case XUART_PARITY_ODD:
            _parity = UART_PARITY_ODD; break;
    }

    uart_stop_bits_t _stopbits = UART_STOP_BITS_1;
    switch (parity) {
        case UART_STOPBIT_1:
            _stopbits = UART_STOP_BITS_1; break;
        case UART_STOPBIT_2:
            _stopbits = UART_STOP_BITS_2; break;        
    }

    uart_config_t uart_config = {
        .baud_rate  = (int)baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = _parity,
        .stop_bits  = _stopbits,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(UARTF_SERIAL_NO, &uart_config));

#if defined UARTF_USE_TX_IO || defined UARTF_USE_RX_IO // both need to be defined
    ESP_ERROR_CHECK(uart_set_pin(UARTF_SERIAL_NO, UARTF_USE_TX_IO, UARTF_USE_RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#else
    ESP_ERROR_CHECK(uart_set_pin(UARTF_SERIAL_NO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#endif

    ESP_ERROR_CHECK(uart_driver_install(UARTF_SERIAL_NO, UARTF_RXBUFSIZE, UARTF_TXBUFSIZE, 0, NULL, 0));  // rx buf size needs to be > 128
    ESP_ERROR_CHECK(uart_set_rx_full_threshold(UARTF_SERIAL_NO, 8)); // default is 120 which is too much, buffer only 128 bytes
    ESP_ERROR_CHECK(uart_set_rx_timeout(UARTF_SERIAL_NO, 1));        // wait for 1 symbol (~11 bits) to trigger Rx ISR, default 2


#elif defined ESP8266
    UARTF_SERIAL_NO.setRxBufferSize(UARTF_RXBUFSIZE);
    UARTF_SERIAL_NO.begin(baud);
#endif
}


void _uartf_initit_halfduplex(void)
{
    uart_intr_config_t uart_intr = {
        .intr_enable_mask = UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT | UART_INTR_TX_DONE,  // FIFO Full, FIFO Timeout, TX Done
        .rx_timeout_thresh = 1,  // 1 symbol ~ 11 bits
        .rxfifo_full_thresh = 1,  // interrupt every byte
    };

    ESP_ERROR_CHECK(uart_isr_free(UARTF_SERIAL_NO));  // Diasble the 'built-in' ISR
    ESP_ERROR_CHECK(uart_isr_register(UARTF_SERIAL_NO, uartf_intr_handle, NULL, ESP_INTR_FLAG_IRAM, NULL));
    ESP_ERROR_CHECK(uart_intr_config(UARTF_SERIAL_NO, &uart_intr));   // Configure the new interrupt
}

void uartf_setbaudrate(uint32_t baud)
{
#ifdef ESP32
    ESP_ERROR_CHECK(uart_driver_delete(UARTF_SERIAL_NO));
#elif
    UARTF_SERIAL_NO.end();
#endif
    _uartf_initit(baud, XUART_PARITY_NO, UART_STOPBIT_1);
}


void uartf_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
#ifdef ESP32
    ESP_ERROR_CHECK(uart_driver_delete(UARTF_SERIAL_NO));
#elif
    UARTF_SERIAL_NO.end();
#endif
    _uartf_initit(baud, parity, stopbits);
}


void uartf_init(void)
{
#ifdef ESP32
    ESP_ERROR_CHECK(uart_driver_delete(UARTF_SERIAL_NO));
#elif
    UARTF_SERIAL_NO.end();
#endif
    _uartf_initit(UARTF_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
}

void uartf_init_isroff(void)
{
#ifdef ESP32
    ESP_ERROR_CHECK(uart_driver_delete(UARTF_SERIAL_NO));
#elif
    UARTF_SERIAL_NO.end();
#endif
    _uartf_initit(UARTF_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
}

void uartf_init_halfduplex(void)
{
    ESP_ERROR_CHECK(uart_driver_delete(UARTF_SERIAL_NO));
    _uartf_initit(UARTF_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
    _uartf_initit_halfduplex();
}


#endif // ESPLIB_UARTF_H

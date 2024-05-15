//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP UARTC
//********************************************************
#ifndef ESPLIB_UARTC_H
#define ESPLIB_UARTC_H

#include "driver/uart.h"
#include "esp_check.h"
#include "hal/uart_ll.h"

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


#ifdef UARTC_USE_SERIAL
#ifdef ESP32
  #define UARTC_SERIAL_NO       UART_NUM_0
  #define UARTC_SERIAL_NO_LL    UART0
#elif
  #define UARTC_SERIAL_NO       Serial
#endif
#elif defined UARTC_USE_SERIAL1
#ifdef ESP32
  #define UARTC_SERIAL_NO       UART_NUM_1
  #define UARTC_SERIAL_NO_LL    UART1
#elif
  #define UARTC_SERIAL_NO       Serial1
#endif
#elif defined UARTC_USE_SERIAL2
#ifdef ESP32
  #define UARTC_SERIAL_NO       UART_NUM_2
  #define UARTC_SERIAL_NO_LL    UART2
#endif
#else
  #error UARTC_SERIAL_NO must be defined!
#endif


#ifndef UARTC_TXBUFSIZE
  #define UARTC_TXBUFSIZE       256 // MUST be 2^N
#endif
#ifndef UARTC_RXBUFSIZE
  #define UARTC_RXBUFSIZE       256 // MUST be 2^N
#endif

#ifdef UARTC_USE_TX_ISR
  #define UARTC_TXBUFSIZEMASK  (UARTC_TXBUFSIZE-1)

  volatile char uartc_txbuf[UARTC_TXBUFSIZE];
  volatile uint16_t uartc_txwritepos; // pos at which the last byte was stored
  volatile uint16_t uartc_txreadpos; // pos at which the next byte is to be fetched

  #define UARTC_RXBUFSIZEMASK  (UARTC_RXBUFSIZE-1)

  volatile char uartc_rxbuf[UARTC_RXBUFSIZE];
  volatile uint16_t uartc_rxwritepos; // pos at which the last byte was stored
  volatile uint16_t uartc_rxreadpos; // pos at which the next byte is to be fetched
#endif


//-------------------------------------------------------
// ISR routine
//-------------------------------------------------------
static void IRAM_ATTR uartc_intr_handle(void *arg)   // UART ISR
{
  uint32_t uart_intr_status = UARTC_SERIAL_NO_LL.int_st.val;

  if (uart_intr_status & UART_INTR_RXFIFO_FULL || uart_intr_status & UART_INTR_RXFIFO_TOUT)  // The interrupt was from one of the Rx interrupts
  {
    uint8_t usart_dr = UARTC_SERIAL_NO_LL.fifo.rw_byte;  // Read a byte, can also use UART0.status.rxfifo_cnt in a loop

    uint16_t next = (uartc_rxwritepos + 1) & UARTC_RXBUFSIZEMASK;
    if (uartc_rxreadpos != next) { // fifo not full
      uartc_rxbuf[next] = usart_dr;
      uartc_rxwritepos = next;
      uart_clear_intr_status(UARTC_SERIAL_NO, UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);  // Clear the Interrupt Status
    }
    
  }

  if (uart_intr_status & UART_INTR_TX_DONE) {
    /* if (uartc_txwritepos != uartc_txreadpos) { // fifo not empty
      uartc_txreadpos = (uartc_txreadpos + 1) & UARTC_TXBUFSIZEMASK;
      uart_tx_chars(UARTC_SERIAL_NO, (const char*)&uartc_txbuf[uartc_txreadpos], 1);  // write the byte
    } */
    uart_clear_intr_status(UARTC_SERIAL_NO, UART_TX_DONE_INT_CLR);  // clear the interrupt status
  }
}


//-------------------------------------------------------
// TX routines
//-------------------------------------------------------
IRAM_ATTR void uartc_putbuf(uint8_t* buf, uint16_t len)
{
#ifdef ESP32
    uart_tx_chars(UARTC_SERIAL_NO, (const char*)buf, len);  // Fix this
#elif
    UARTC_SERIAL_NO.write((uint8_t*)buf, len);
#endif
}

IRAM_ATTR void uartc_tx_flush(void)
{
#ifdef ESP32
    uart_wait_tx_done(UARTC_SERIAL_NO, 100);  // Fix this // 100 ms - what should be used?
#elif
    UARTC_SERIAL_NO.flush();
#endif
}


//-------------------------------------------------------
// RX routines
//-------------------------------------------------------
IRAM_ATTR char uartc_getc(void)
{
#ifdef ESP32
    while (uartc_rxwritepos == uartc_rxreadpos) {};
    uartc_rxreadpos = (uartc_rxreadpos + 1) & UARTC_RXBUFSIZEMASK;
    return uartc_rxbuf[uartc_rxreadpos];
#elif
    return (char)UARTC_SERIAL_NO.read();
#endif
}


IRAM_ATTR void uartc_rx_flush(void)
{
#ifdef ESP32
    uartc_rxwritepos = uartc_rxreadpos = 0;
#elif
    while (UARTC_SERIAL_NO.available() > 0) UARTC_SERIAL_NO.read();
#endif
}


IRAM_ATTR uint16_t uartc_rx_bytesavailable(void)
{
#ifdef ESP32
    int16_t d;
    d = (int16_t)uartc_rxwritepos - (int16_t)uartc_rxreadpos;
    return (d < 0) ? d + (UARTC_RXBUFSIZEMASK + 1) : d;
#elif
    return (UARTC_SERIAL_NO.available() > 0) ? UARTC_SERIAL_NO.available() : 0;
#endif
}


IRAM_ATTR uint16_t uartc_rx_available(void)
{
#ifdef ESP32
    if (uartc_rxwritepos == uartc_rxreadpos) return 0; // fifo empty
    return 1;
#elif
    return (UARTC_SERIAL_NO.available() > 0) ? 1 : 0;
#endif
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------
// Note: ESP32 has a hardware fifo for tx, which is 128 bytes in size. However, MAVLink messages
// can be larger than this, and data would thus be lost when put only into the fifo. It is therefore
// crucial to set a Tx buffer size of sufficient size. setTxBufferSize() is not available for ESP82xx.

void _uartc_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
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
    switch (stopbits) {
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

    uart_intr_config_t uart_intr = {
        .intr_enable_mask = UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT | UART_INTR_TX_DONE,  // FIFO Full, FIFO Timeout, TX Done
        .rx_timeout_thresh = 1,  // 1 symbols ~ 11 bits
        .txfifo_empty_intr_thresh = 10,  // we don't use - doesn't matter
        .rxfifo_full_thresh = 1,  // interrupt every byte
  };

    ESP_ERROR_CHECK(uart_param_config(UARTC_SERIAL_NO, &uart_config));

#if defined UARTC_USE_TX_IO || defined UARTC_USE_RX_IO // both need to be defined
    ESP_ERROR_CHECK(uart_set_pin(UARTC_SERIAL_NO, UARTC_USE_TX_IO, UARTC_USE_RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#else
    ESP_ERROR_CHECK(uart_set_pin(UARTC_SERIAL_NO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#endif

    ESP_ERROR_CHECK(uart_driver_install(UARTC_SERIAL_NO, UARTC_RXBUFSIZE, UARTC_TXBUFSIZE, 0, NULL, 0));  // rx buf size needs to be > 128
    ESP_ERROR_CHECK(uart_isr_free(UARTC_SERIAL_NO));  // diasble the 'built-in' ISR
    ESP_ERROR_CHECK(uart_isr_register(UARTC_SERIAL_NO, uartc_intr_handle, NULL, ESP_INTR_FLAG_IRAM, NULL));  // register our ISR
    ESP_ERROR_CHECK(uart_intr_config(UARTC_SERIAL_NO, &uart_intr)); // configure the ISR conditions


#elif defined ESP8266
    UARTC_SERIAL_NO.setRxBufferSize(UARTC_RXBUFSIZE);
    UARTC_SERIAL_NO.begin(baud);
#endif
}


void uartc_setbaudrate(uint32_t baud)
{
#ifdef ESP32
    ESP_ERROR_CHECK(uart_driver_delete(UARTC_SERIAL_NO));
#elif
    UARTC_SERIAL_NO.end();
#endif
    _uartc_initit(baud, XUART_PARITY_NO, UART_STOPBIT_1);
}


void uartc_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
#ifdef ESP32
    ESP_ERROR_CHECK(uart_driver_delete(UARTC_SERIAL_NO));
#elif
    UARTC_SERIAL_NO.end();
#endif
    _uartc_initit(baud, parity, stopbits);
}


void uartc_init(void)
{
#ifdef ESP32
    ESP_ERROR_CHECK(uart_driver_delete(UARTC_SERIAL_NO));
#elif
    UARTC_SERIAL_NO.end();
#endif
    _uartc_initit(UARTC_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
}

void uartc_init_isroff(void)
{
#ifdef ESP32
    ESP_ERROR_CHECK(uart_driver_delete(UARTC_SERIAL_NO));
#elif
    UARTC_SERIAL_NO.end();
#endif
    _uartc_initit(UARTC_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
}


#endif // ESPLIB_UARTC_H

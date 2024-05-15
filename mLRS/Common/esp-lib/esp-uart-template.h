//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP UART$
//********************************************************
#ifndef ESPLIB_UART$_H
#define ESPLIB_UART$_H

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


#ifdef UART$_USE_SERIAL
#ifdef ESP32
  #define UART$_SERIAL_NO       UART_NUM_0
  #define UART$_SERIAL_NO_LL    UART0
#elif
  #define UART$_SERIAL_NO       Serial
#endif
#elif defined UART$_USE_SERIAL1
#ifdef ESP32
  #define UART$_SERIAL_NO       UART_NUM_1
  #define UART$_SERIAL_NO_LL    UART1
#elif
  #define UART$_SERIAL_NO       Serial1
#endif
#elif defined UART$_USE_SERIAL2
#ifdef ESP32
  #define UART$_SERIAL_NO       UART_NUM_2
  #define UART$_SERIAL_NO_LL    UART2
#endif
#else
  #error UART$_SERIAL_NO must be defined!
#endif


#ifndef UART$_TXBUFSIZE
  #define UART$_TXBUFSIZE       256 // MUST be 2^N
#endif
#ifndef UART$_RXBUFSIZE
  #define UART$_RXBUFSIZE       256 // MUST be 2^N
#endif

#ifdef UART$_USE_TX_ISR
  #define UART$_TXBUFSIZEMASK  (UART$_TXBUFSIZE-1)

  volatile char uart$_txbuf[UART$_TXBUFSIZE];
  volatile uint16_t uart$_txwritepos; // pos at which the last byte was stored
  volatile uint16_t uart$_txreadpos; // pos at which the next byte is to be fetched

  #define UART$_RXBUFSIZEMASK  (UART$_RXBUFSIZE-1)

  volatile char uart$_rxbuf[UART$_RXBUFSIZE];
  volatile uint16_t uart$_rxwritepos; // pos at which the last byte was stored
  volatile uint16_t uart$_rxreadpos; // pos at which the next byte is to be fetched
#endif


//-------------------------------------------------------
// ISR routine
//-------------------------------------------------------
static void IRAM_ATTR uart_intr_handle(void *arg)   // UART ISR
{
  uint32_t uart_intr_status = UART$_SERIAL_NO_LL.int_st.val;

  if (uart_intr_status & UART_INTR_RXFIFO_FULL || uart_intr_status & UART_INTR_RXFIFO_TOUT)  // The interrupt was from one of the Rx interrupts
  {
    uint8_t usart_dr = UART$_SERIAL_NO_LL.fifo.rw_byte;  // Read a byte, can also use UART0.status.rxfifo_cnt in a loop

    uint16_t next = (uart$_rxwritepos + 1) & UART$_RXBUFSIZEMASK;
    if (uart$_rxreadpos != next) { // fifo not full
      uart$_rxbuf[next] = usart_dr;
      uart$_rxwritepos = next;
      uart_clear_intr_status(UART$_SERIAL_NO, UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);  // Clear the Interrupt Status
    }
    
  }

  if (uart_intr_status & UART_INTR_TX_DONE) {
    if (uart$_txwritepos != uart$_txreadpos) { // fifo not empty
      uart$_txreadpos = (uart$_txreadpos + 1) & UART$_TXBUFSIZEMASK;
      uart_tx_chars(UART$_SERIAL_NO, (const char*) uart$_txbuf[uart$_txreadpos], 1);  // write the byte
      uart_clear_intr_status(UART$_SERIAL_NO, UART_TX_DONE_INT_CLR);  // clear the interrupt status
    }
  }
}


//-------------------------------------------------------
// TX routines
//-------------------------------------------------------
IRAM_ATTR void uart$_putbuf(uint8_t* buf, uint16_t len)
{
#ifdef ESP32
    uart_write_bytes(UART$_SERIAL_NO, (uint8_t*)buf, len);
#elif
    UART$_SERIAL_NO.write((uint8_t*)buf, len);
#endif
}


IRAM_ATTR char uart$_getc(void)
{
#ifdef ESP32
    uint8_t c = 0;
    uart_read_bytes(UART$_SERIAL_NO, &c, 1, 0);
    return (char)c;
#elif
    return (char)UART$_SERIAL_NO.read();
#endif
}


IRAM_ATTR void uart$_rx_flush(void)
{
#ifdef ESP32
    uart_flush(UART$_SERIAL_NO);
#elif
    while (UART$_SERIAL_NO.available() > 0) UART$_SERIAL_NO.read();
#endif
}


IRAM_ATTR void uart$_tx_flush(void)
{
#ifdef ESP32
    uart_wait_tx_done(UART$_SERIAL_NO, 100);  // 100 ms - what should be used?
#elif
    UART$_SERIAL_NO.flush();
#endif
}


IRAM_ATTR uint16_t uart$_rx_bytesavailable(void)
{
#ifdef ESP32
    uint32_t bytesAvailable = 0;
    uart_get_buffered_data_len(UART$_SERIAL_NO, &bytesAvailable);
    return (uint16_t)bytesAvailable;
#elif
    return (UART$_SERIAL_NO.available() > 0) ? UART$_SERIAL_NO.available() : 0;
#endif
}


IRAM_ATTR uint16_t uart$_rx_available(void)
{
#ifdef ESP32
    uint32_t bytesAvailable = 0;
    uart_get_buffered_data_len(UART$_SERIAL_NO, &bytesAvailable);
    return ((uint16_t)bytesAvailable > 0) ? 1 : 0;
#elif
    return (UART$_SERIAL_NO.available() > 0) ? 1 : 0;
#endif
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------
// Note: ESP32 has a hardware fifo for tx, which is 128 bytes in size. However, MAVLink messages
// can be larger than this, and data would thus be lost when put only into the fifo. It is therefore
// crucial to set a Tx buffer size of sufficient size. setTxBufferSize() is not available for ESP82xx.

void _uart$_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
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

    ESP_ERROR_CHECK(uart_param_config(UART$_SERIAL_NO, &uart_config));

#if defined UART$_USE_TX_IO || defined UART$_USE_RX_IO // both need to be defined
    ESP_ERROR_CHECK(uart_set_pin(UART$_SERIAL_NO, UART$_USE_TX_IO, UART$_USE_RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#else
    ESP_ERROR_CHECK(uart_set_pin(UART$_SERIAL_NO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#endif

    ESP_ERROR_CHECK(uart_driver_install(UART$_SERIAL_NO, UART$_RXBUFSIZE, UART$_TXBUFSIZE, 0, NULL, 0));  // rx buf size needs to be > 128
    ESP_ERROR_CHECK(uart_isr_free(UART$_SERIAL_NO));  // diasble the 'built-in' ISR
    ESP_ERROR_CHECK(uart_isr_register(UART$_SERIAL_NO, uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, NULL));  // register our ISR
    ESP_ERROR_CHECK(uart_intr_config(UART$_SERIAL_NO, &uart_intr)); // configure the ISR conditions


#elif defined ESP8266
    UART$_SERIAL_NO.setRxBufferSize(UART$_RXBUFSIZE);
    UART$_SERIAL_NO.begin(baud);
#endif
}


void uart$_setbaudrate(uint32_t baud)
{
#ifdef ESP32
    ESP_ERROR_CHECK(uart_driver_delete(UART$_SERIAL_NO));
#elif
    UART$_SERIAL_NO.end();
#endif
    _uart$_initit(baud, XUART_PARITY_NO, UART_STOPBIT_1);
}


void uart$_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
#ifdef ESP32
    ESP_ERROR_CHECK(uart_driver_delete(UART$_SERIAL_NO));
#elif
    UART$_SERIAL_NO.end();
#endif
    _uart$_initit(baud, parity, stopbits);
}


void uart$_init(void)
{
#ifdef ESP32
    ESP_ERROR_CHECK(uart_driver_delete(UART$_SERIAL_NO));
#elif
    UART$_SERIAL_NO.end();
#endif
    _uart$_initit(UART$_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
}

void uart$_init_isroff(void)
{
#ifdef ESP32
    ESP_ERROR_CHECK(uart_driver_delete(UART$_SERIAL_NO));
#elif
    UART$_SERIAL_NO.end();
#endif
    _uart$_initit(UART$_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
}


#endif // ESPLIB_UART$_H

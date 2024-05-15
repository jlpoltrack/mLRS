//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP UARTB
//********************************************************
#ifndef ESPLIB_UARTB_H
#define ESPLIB_UARTB_H

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


#ifdef UARTB_USE_SERIAL
#ifdef ESP32
  #define UARTB_SERIAL_NO       UART_NUM_0
  #define UARTB_SERIAL_NO_LL    UART0
#elif
  #define UARTB_SERIAL_NO       Serial
#endif
#elif defined UARTB_USE_SERIAL1
#ifdef ESP32
  #define UARTB_SERIAL_NO       UART_NUM_1
  #define UARTB_SERIAL_NO_LL    UART1
#elif
  #define UARTB_SERIAL_NO       Serial1
#endif
#elif defined UARTB_USE_SERIAL2
#ifdef ESP32
  #define UARTB_SERIAL_NO       UART_NUM_2
  #define UARTB_SERIAL_NO_LL    UART2
#endif
#else
  #error UARTB_SERIAL_NO must be defined!
#endif


#ifndef UARTB_TXBUFSIZE
  #define UARTB_TXBUFSIZE       256 // MUST be 2^N
#endif
#ifndef UARTB_RXBUFSIZE
  #define UARTB_RXBUFSIZE       256 // MUST be 2^N
#endif

#ifdef UARTB_USE_TX_ISR
  #define UARTB_TXBUFSIZEMASK  (UARTB_TXBUFSIZE-1)

  volatile char uartb_txbuf[UARTB_TXBUFSIZE];
  volatile uint16_t uartb_txwritepos; // pos at which the last byte was stored
  volatile uint16_t uartb_txreadpos; // pos at which the next byte is to be fetched

  #define UARTB_RXBUFSIZEMASK  (UARTB_RXBUFSIZE-1)

  volatile char uartb_rxbuf[UARTB_RXBUFSIZE];
  volatile uint16_t uartb_rxwritepos; // pos at which the last byte was stored
  volatile uint16_t uartb_rxreadpos; // pos at which the next byte is to be fetched
#endif


//-------------------------------------------------------
// ISR routine
//-------------------------------------------------------
static void IRAM_ATTR uart_intr_handle(void *arg)   // UART ISR
{
  uint32_t uart_intr_status = UARTB_SERIAL_NO_LL.int_st.val;

  if (uart_intr_status & UART_INTR_RXFIFO_FULL || uart_intr_status & UART_INTR_RXFIFO_TOUT)  // The interrupt was from one of the Rx interrupts
  {
    uint8_t usart_dr = UARTB_SERIAL_NO_LL.fifo.rw_byte;  // Read a byte, can also use UART0.status.rxfifo_cnt in a loop

    uint16_t next = (uartb_rxwritepos + 1) & UARTB_RXBUFSIZEMASK;
    if (uartb_rxreadpos != next) { // fifo not full
      uartb_rxbuf[next] = usart_dr;
      uartb_rxwritepos = next;
      uart_clear_intr_status(UARTB_SERIAL_NO, UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);  // Clear the Interrupt Status
    }
    
  }

  if (uart_intr_status & UART_INTR_TX_DONE) {
    if (uartb_txwritepos != uartb_txreadpos) { // fifo not empty
      uartb_txreadpos = (uartb_txreadpos + 1) & UARTB_TXBUFSIZEMASK;
      uart_tx_chars(UARTB_SERIAL_NO, (const char*)uartb_txbuf[uartb_txreadpos], 1);  // write the byte
      uart_clear_intr_status(UARTB_SERIAL_NO, UART_TX_DONE_INT_CLR);  // clear the interrupt status
    }
  }
}




IRAM_ATTR void uartb_putbuf(uint8_t* buf, uint16_t len)
{
#ifdef ESP32
    uart_write_bytes(UARTB_SERIAL_NO, (uint8_t*)buf, len);
#elif
    UARTB_SERIAL_NO.write((uint8_t*)buf, len);
#endif
}


IRAM_ATTR char uartb_getc(void)
{
#ifdef ESP32
    uint8_t c = 0;
    uart_read_bytes(UARTB_SERIAL_NO, &c, 1, 0);
    return (char)c;
#elif
    return (char)UARTB_SERIAL_NO.read();
#endif
}


IRAM_ATTR void uartb_rx_flush(void)
{
#ifdef ESP32
    uart_flush(UARTB_SERIAL_NO);
#elif
    while (UARTB_SERIAL_NO.available() > 0) UARTB_SERIAL_NO.read();
#endif
}


IRAM_ATTR void uartb_tx_flush(void)
{
#ifdef ESP32
    uart_wait_tx_done(UARTB_SERIAL_NO, 100);  // 100 ms - what should be used?
#elif
    UARTB_SERIAL_NO.flush();
#endif
}


IRAM_ATTR uint16_t uartb_rx_bytesavailable(void)
{
#ifdef ESP32
    uint32_t bytesAvailable = 0;
    uart_get_buffered_data_len(UARTB_SERIAL_NO, &bytesAvailable);
    return (uint16_t)bytesAvailable;
#elif
    return (UARTB_SERIAL_NO.available() > 0) ? UARTB_SERIAL_NO.available() : 0;
#endif
}


IRAM_ATTR uint16_t uartb_rx_available(void)
{
#ifdef ESP32
    uint32_t bytesAvailable = 0;
    uart_get_buffered_data_len(UARTB_SERIAL_NO, &bytesAvailable);
    return ((uint16_t)bytesAvailable > 0) ? 1 : 0;
#elif
    return (UARTB_SERIAL_NO.available() > 0) ? 1 : 0;
#endif
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------
// Note: ESP32 has a hardware fifo for tx, which is 128 bytes in size. However, MAVLink messages
// can be larger than this, and data would thus be lost when put only into the fifo. It is therefore
// crucial to set a Tx buffer size of sufficient size. setTxBufferSize() is not available for ESP82xx.

void _uartb_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
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

    ESP_ERROR_CHECK(uart_param_config(UARTB_SERIAL_NO, &uart_config));

#if defined UARTB_USE_TX_IO || defined UARTB_USE_RX_IO // both need to be defined
    ESP_ERROR_CHECK(uart_set_pin(UARTB_SERIAL_NO, UARTB_USE_TX_IO, UARTB_USE_RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#else
    ESP_ERROR_CHECK(uart_set_pin(UARTB_SERIAL_NO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#endif

    ESP_ERROR_CHECK(uart_driver_install(UARTB_SERIAL_NO, UARTB_RXBUFSIZE, UARTB_TXBUFSIZE, 0, NULL, 0));  // rx buf size needs to be > 128
    ESP_ERROR_CHECK(uart_isr_free(UARTB_SERIAL_NO));  // diasble the 'built-in' ISR
    ESP_ERROR_CHECK(uart_isr_register(UARTB_SERIAL_NO, uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, NULL));  // register our ISR
    ESP_ERROR_CHECK(uart_intr_config(UARTB_SERIAL_NO, &uart_intr)); // configure the ISR conditions


#elif defined ESP8266
    UARTB_SERIAL_NO.setRxBufferSize(UARTB_RXBUFSIZE);
    UARTB_SERIAL_NO.begin(baud);
#endif
}


void uartb_setbaudrate(uint32_t baud)
{
#ifdef ESP32
    ESP_ERROR_CHECK(uart_driver_delete(UARTB_SERIAL_NO));
#elif
    UARTB_SERIAL_NO.end();
#endif
    _uartb_initit(baud, XUART_PARITY_NO, UART_STOPBIT_1);
}


void uartb_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
#ifdef ESP32
    ESP_ERROR_CHECK(uart_driver_delete(UARTB_SERIAL_NO));
#elif
    UARTB_SERIAL_NO.end();
#endif
    _uartb_initit(baud, parity, stopbits);
}


void uartb_init(void)
{
#ifdef ESP32
    ESP_ERROR_CHECK(uart_driver_delete(UARTB_SERIAL_NO));
#elif
    UARTB_SERIAL_NO.end();
#endif
    _uartb_initit(UARTB_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
}

void uartb_init_isroff(void)
{
#ifdef ESP32
    ESP_ERROR_CHECK(uart_driver_delete(UARTB_SERIAL_NO));
#elif
    UARTB_SERIAL_NO.end();
#endif
    _uartb_initit(UARTB_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
}


#endif // ESPLIB_UARTB_H

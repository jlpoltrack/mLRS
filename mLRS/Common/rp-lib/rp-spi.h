#ifndef RP_SPI_H
#define RP_SPI_H

#include <Arduino.h>
#include <SPI.h>
#include "rp-peripherals.h"

// Define default SPI instance if not specified
#ifndef RX_SPI
#define RX_SPI SPI
#endif

// Ensure SPI_FREQUENCY is defined
#ifndef SPI_FREQUENCY
#define SPI_FREQUENCY 10000000L
#endif

inline void spi_init(uint32_t frequency) {
#if defined(SPI_MISO) && defined(SPI_MOSI) && defined(SPI_SCK)
    // Explicitly set SPI pins if defined
    RX_SPI.setRX(SPI_MISO);
    RX_SPI.setTX(SPI_MOSI);
    RX_SPI.setSCK(SPI_SCK);
#ifdef ARDUINO_ARCH_RP2040
    Serial1.print("  spi_init: configured SPI on MISO=");
    Serial1.print(SPI_MISO);
    Serial1.print(" MOSI=");
    Serial1.print(SPI_MOSI);
    Serial1.print(" SCK=");
    Serial1.println(SPI_SCK);
#endif
#endif
    RX_SPI.begin();
#ifdef ARDUINO_ARCH_RP2040
    Serial1.println("  spi_init: SPI.begin() done");
#endif
    // Transaction handling moved to spi_select/spi_deselect to ensure interrupts are enabled between transactions

    // Initialize Chip Select pin
    // Must be done AFTER begin() to override any default hardware CS configuration
#ifdef RX_SPI_NSS
    pinMode(RX_SPI_NSS, OUTPUT);
    gpio_high(RX_SPI_NSS);
#ifdef ARDUINO_ARCH_RP2040
    Serial1.print("  spi_init: CS pin ");
    Serial1.print(RX_SPI_NSS);
    Serial1.println(" configured");
#endif
#endif
    (void)frequency; // frequency is now used in spi_select via SPI_FREQUENCY macro
}

// default spi_init for sx driver compatibility
inline void spi_init(void) {
    spi_init(SPI_FREQUENCY);
}

inline void spi_select(uint8_t pin) {
    RX_SPI.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
    gpio_low(pin);
}
inline void spi_deselect(uint8_t pin) {
    gpio_high(pin);
    RX_SPI.endTransaction();
}

inline void spi_select(void) {
    RX_SPI.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
    gpio_low(RX_SPI_NSS);
}
inline void spi_deselect(void) {
    gpio_high(RX_SPI_NSS);
    RX_SPI.endTransaction();
}

inline uint8_t spi_transfer(uint8_t data) {
    return RX_SPI.transfer(data);
}

inline void spi_transfer(uint8_t* dataout, uint8_t* datain, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        datain[i] = RX_SPI.transfer(dataout[i]);
    }
}

inline void spi_write(uint8_t data) {
    RX_SPI.transfer(data);
}

inline void spi_write(uint8_t* dataout, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        RX_SPI.transfer(dataout[i]);
    }
}

inline uint8_t spi_read(void) {
    return RX_SPI.transfer(0x00);
}

inline void spi_read(uint8_t* datain, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        datain[i] = RX_SPI.transfer(0x00);
    }
}

inline void spi_setnop(uint8_t nop) { (void)nop; }

#endif // RP_SPI_H
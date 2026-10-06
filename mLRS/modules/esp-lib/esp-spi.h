//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP SPI Interface
//********************************************************
#ifndef ESPLIB_SPI_H
#define ESPLIB_SPI_H


#include <SPI.h>

#ifdef ESP8266
#define SPI_MSBFIRST  MSBFIRST // for some reasons not defined for EPR82xx
#endif

#ifdef ESP32
#include <soc/spi_struct.h>

static spi_dev_t* spi_dev; // set in spi_init()

// does what spiTransferBytesNL() does, but from RAM and not from flash
// sends 0xFF if no out data, the SPI data buffer holds 64 bytes
IRAM_ATTR __attribute__((noinline)) void _spi_transfer(const uint8_t* dataout, uint8_t* datain, uint16_t len)
{
    while (len) {
        uint16_t chunk = (len > 64) ? 64 : len;

#if defined CONFIG_IDF_TARGET_ESP32C3 || defined CONFIG_IDF_TARGET_ESP32S3
        spi_dev->ms_dlen.ms_data_bitlen = chunk * 8 - 1;
#else
        spi_dev->mosi_dlen.usr_mosi_dbitlen = chunk * 8 - 1;
        spi_dev->miso_dlen.usr_miso_dbitlen = chunk * 8 - 1;
#endif

        for (uint16_t n = 0; n < chunk; n += 4) {
            uint32_t w = 0xFFFFFFFF;
            if (dataout) {
                uint8_t* w8 = (uint8_t*)&w;
                for (uint16_t i = 0; i < 4 && n + i < chunk; i++) w8[i] = dataout[n + i];
            }
            spi_dev->data_buf[n / 4] = w;
        }

#if defined CONFIG_IDF_TARGET_ESP32C3 || defined CONFIG_IDF_TARGET_ESP32S3
        spi_dev->cmd.update = 1;
        while (spi_dev->cmd.update) {}
#endif
        spi_dev->cmd.usr = 1;
        while (spi_dev->cmd.usr) {}

        if (datain) {
            for (uint16_t n = 0; n < chunk; n += 4) {
                uint32_t w = spi_dev->data_buf[n / 4];
                uint8_t* w8 = (uint8_t*)&w;
                for (uint16_t i = 0; i < 4 && n + i < chunk; i++) datain[n + i] = w8[i];
            }
            datain += chunk;
        }
        if (dataout) dataout += chunk;
        len -= chunk;
    }
}
#endif


//-- select functions

#ifdef SPI_CS_IO

IRAM_ATTR void spi_select(void)
{
    gpio_low(SPI_CS_IO);
}

IRAM_ATTR void spi_deselect(void)
{
    gpio_high(SPI_CS_IO);
}

#endif // #ifdef SPI_CS_IO


//-- transmit, transfer, read, write functions

// these are blocking
// utilize ESP SPI buffer
// transferBytes()
// - does while(SPI1CMD & SPIBUSY) {} as needed
// - if dataout or datain are not aligned, then it does memcpy to aligned buffer on stack
// - sends 0xFFFFFFFF if no out data!! Problem: sx datasheet says 0x00

IRAM_ATTR void spi_transfer(const uint8_t* dataout, uint8_t* datain, const uint8_t len)
{
#ifdef ESP32
    _spi_transfer(dataout, datain, len);
#elif defined ESP8266
    SPI.transferBytes(dataout, datain, len);
#endif
}


IRAM_ATTR void spi_read(uint8_t* datain, const uint8_t len)
{
#ifdef ESP32
    _spi_transfer(nullptr, datain, len);
#elif defined ESP8266
    SPI.transferBytes(nullptr, datain, len);
#endif
}


IRAM_ATTR void spi_write(const uint8_t* dataout, uint8_t len)
{
#ifdef ESP32
    _spi_transfer(dataout, nullptr, len);
#elif defined ESP8266
    SPI.transferBytes(dataout, nullptr, len);
#endif
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

void spi_setnop(uint8_t nop)
{
    // currently not supported, should be 0x00 fo sx, is 0xFF per ESP SPI library
}


void spi_init(void)
{
#ifdef SPI_CS_IO
    gpio_init(SPI_CS_IO, IO_MODE_OUTPUT_PP_HIGH);
#endif

#ifdef ESP32
    spiEndTransaction(SPI.bus());
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS_IO);
    spiSimpleTransaction(SPI.bus());
    spi_dev = *(spi_dev_t**)SPI.bus(); // dev is the first member of the opaque spi_t
#elif defined ESP8266
    SPI.begin();
#endif
    SPI.setFrequency(SPI_FREQUENCY);
    SPI.setBitOrder(SPI_MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
}


#endif // ESPLIB_SPI_H
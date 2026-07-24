//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP SPI
//********************************************************
#ifndef RPLIB_SPI_H
#define RPLIB_SPI_H

#include <SPI.h>
#include <hardware/spi.h>
#include "rp-peripherals.h"

// select hardware SPI instance
#ifdef SPI_USE_SPI1
  #define SPI_BUS   SPI1
  #define SPI_INST  spi1
#else
  #define SPI_BUS   SPI   // default to SPI0
  #define SPI_INST  spi0
#endif

static uint8_t spi_nop = 0; // fill byte sent during reads, set via spi_setnop()

//-- select functions

void spi_select(void)
{
    gpio_low(SPI_CS_IO);
}

void spi_deselect(void)
{
    gpio_high(SPI_CS_IO);
}


//-- transmit, transfer, read, write functions

void spi_transfer(const uint8_t* dataout, uint8_t* datain, const uint8_t len)
{
    SPI_BUS.transfer(dataout, datain, len);
}


void spi_read(uint8_t* datain, const uint8_t len)
{
    // SDK call to control the fill byte: the radios require their NOP byte
    // during reads, Arduino's transfer(nullptr, ...) would send 0xFF
    spi_read_blocking(SPI_INST, spi_nop, datain, len);
}


void spi_write(const uint8_t* dataout, uint8_t len)
{
    SPI_BUS.transfer(dataout, nullptr, len);
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

void spi_setnop(uint8_t nop)
{
    spi_nop = nop;
}


void spi_init(void)
{
    gpio_init(SPI_CS_IO, IO_MODE_OUTPUT_PP_HIGH);
    SPI_BUS.setRX(SPI_MISO);
    SPI_BUS.setTX(SPI_MOSI);
    SPI_BUS.setSCK(SPI_SCK);
    SPI_BUS.begin();
    SPI_BUS.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
}


#endif // RPLIB_SPI_H

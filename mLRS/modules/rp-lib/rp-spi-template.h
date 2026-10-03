//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP SPI$
//********************************************************
#ifndef RPLIB_SPI$_H
#define RPLIB_SPI$_H

#include <SPI.h>
#include <hardware/spi.h>
#include "rp-peripherals.h"

// select hardware SPI instance
#ifdef SPI$_USE_SPI1
  #define SPI$_BUS   SPI1
  #define SPI$_INST  spi1
#else
  #define SPI$_BUS   SPI   // default to SPI0
  #define SPI$_INST  spi0
#endif

static uint8_t spi$_nop = 0; // fill byte sent during reads, set via spi$_setnop()

//-- select functions

void spi$_select(void)
{
    gpio_low(SPI$_CS_IO);
}

void spi$_deselect(void)
{
    gpio_high(SPI$_CS_IO);
}


//-- transmit, transfer, read, write functions

void spi$_transfer(const uint8_t* dataout, uint8_t* datain, const uint8_t len)
{
    SPI$_BUS.transfer(dataout, datain, len);
}


void spi$_read(uint8_t* datain, const uint8_t len)
{
    // SDK call to control the fill byte: the radios require their NOP byte
    // during reads, Arduino's transfer(nullptr, ...) would send 0xFF
    spi_read_blocking(SPI$_INST, spi$_nop, datain, len);
}


void spi$_write(const uint8_t* dataout, uint8_t len)
{
    SPI$_BUS.transfer(dataout, nullptr, len);
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

void spi$_setnop(uint8_t nop)
{
    spi$_nop = nop;
}


void spi$_init(void)
{
    gpio_init(SPI$_CS_IO, IO_MODE_OUTPUT_PP_HIGH);
    SPI$_BUS.setRX(SPI$_MISO);
    SPI$_BUS.setTX(SPI$_MOSI);
    SPI$_BUS.setSCK(SPI$_SCK);
    SPI$_BUS.begin();
    SPI$_BUS.beginTransaction(SPISettings(SPI$_FREQUENCY, MSBFIRST, SPI_MODE0));
}


#endif // RPLIB_SPI$_H

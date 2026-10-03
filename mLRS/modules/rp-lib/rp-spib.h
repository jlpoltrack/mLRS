//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP SPIB
//********************************************************
#ifndef RPLIB_SPIB_H
#define RPLIB_SPIB_H

#include <SPI.h>
#include <hardware/spi.h>
#include "rp-peripherals.h"

// select hardware SPI instance
#ifdef SPIB_USE_SPI1
  #define SPIB_BUS   SPI1
  #define SPIB_INST  spi1
#else
  #define SPIB_BUS   SPI   // default to SPI0
  #define SPIB_INST  spi0
#endif

static uint8_t spib_nop = 0; // fill byte sent during reads, set via spib_setnop()

//-- select functions

void spib_select(void)
{
    gpio_low(SPIB_CS_IO);
}

void spib_deselect(void)
{
    gpio_high(SPIB_CS_IO);
}


//-- transmit, transfer, read, write functions

void spib_transfer(const uint8_t* dataout, uint8_t* datain, const uint8_t len)
{
    SPIB_BUS.transfer(dataout, datain, len);
}


void spib_read(uint8_t* datain, const uint8_t len)
{
    // SDK call to control the fill byte: the radios require their NOP byte
    // during reads, Arduino's transfer(nullptr, ...) would send 0xFF
    spi_read_blocking(SPIB_INST, spib_nop, datain, len);
}


void spib_write(const uint8_t* dataout, uint8_t len)
{
    SPIB_BUS.transfer(dataout, nullptr, len);
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

void spib_setnop(uint8_t nop)
{
    spib_nop = nop;
}


void spib_init(void)
{
    gpio_init(SPIB_CS_IO, IO_MODE_OUTPUT_PP_HIGH);
    SPIB_BUS.setRX(SPIB_MISO);
    SPIB_BUS.setTX(SPIB_MOSI);
    SPIB_BUS.setSCK(SPIB_SCK);
    SPIB_BUS.begin();
    SPIB_BUS.beginTransaction(SPISettings(SPIB_FREQUENCY, MSBFIRST, SPI_MODE0));
}


#endif // RPLIB_SPIB_H

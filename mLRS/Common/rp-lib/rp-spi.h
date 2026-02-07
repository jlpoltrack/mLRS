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
#include "rp-peripherals.h"

// select hardware SPI instance
#ifdef SPI_USE_SPI1
  #define SPI_BUS  SPI1
#else
  #define SPI_BUS  SPI   // default to SPI0
#endif

//-- select functions

void __not_in_flash_func(spi_select)(void)
{
    gpio_low(SPI_CS_IO);
}

void __not_in_flash_func(spi_deselect)(void)
{
    gpio_high(SPI_CS_IO);
}


//-- transmit, transfer, read, write functions

void __not_in_flash_func(spi_transfer)(const uint8_t* dataout, uint8_t* datain, const uint8_t len)
{
    SPI_BUS.transfer(dataout, datain, len);
}


void __not_in_flash_func(spi_read)(uint8_t* datain, const uint8_t len)
{
    SPI_BUS.transfer(nullptr, datain, len);
}


void __not_in_flash_func(spi_write)(const uint8_t* dataout, uint8_t len)
{
    SPI_BUS.transfer(dataout, nullptr, len);
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

void spi_setnop(uint8_t nop) {}


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

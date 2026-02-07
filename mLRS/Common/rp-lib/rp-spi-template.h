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
#include "rp-peripherals.h"

// select hardware SPI instance
#ifdef SPI$_USE_SPI1
  #define SPI$_BUS  SPI1
#else
  #define SPI$_BUS  SPI   // default to SPI0
#endif

//-- select functions

void __not_in_flash_func(spi$_select)(void)
{
    gpio_low(SPI$_CS_IO);
}

void __not_in_flash_func(spi$_deselect)(void)
{
    gpio_high(SPI$_CS_IO);
}


//-- transmit, transfer, read, write functions

void __not_in_flash_func(spi$_transfer)(const uint8_t* dataout, uint8_t* datain, const uint8_t len)
{
    SPI$_BUS.transfer(dataout, datain, len);
}


void __not_in_flash_func(spi$_read)(uint8_t* datain, const uint8_t len)
{
    SPI$_BUS.transfer(nullptr, datain, len);
}


void __not_in_flash_func(spi$_write)(const uint8_t* dataout, uint8_t len)
{
    SPI$_BUS.transfer(dataout, nullptr, len);
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

void spi$_setnop(uint8_t nop) {}


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

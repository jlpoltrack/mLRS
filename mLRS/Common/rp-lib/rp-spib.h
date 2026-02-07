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
#include "rp-peripherals.h"

// select hardware SPI instance
#ifdef SPIB_USE_SPI1
  #define SPIB_BUS  SPI1
#else
  #define SPIB_BUS  SPI   // default to SPI0
#endif

//-- select functions

void __not_in_flash_func(spib_select)(void)
{
    gpio_low(SPIB_CS_IO);
}

void __not_in_flash_func(spib_deselect)(void)
{
    gpio_high(SPIB_CS_IO);
}


//-- transmit, transfer, read, write functions

void __not_in_flash_func(spib_transfer)(const uint8_t* dataout, uint8_t* datain, const uint8_t len)
{
    SPIB_BUS.transfer(dataout, datain, len);
}


void __not_in_flash_func(spib_read)(uint8_t* datain, const uint8_t len)
{
    SPIB_BUS.transfer(nullptr, datain, len);
}


void __not_in_flash_func(spib_write)(const uint8_t* dataout, uint8_t len)
{
    SPIB_BUS.transfer(dataout, nullptr, len);
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

void spib_setnop(uint8_t nop) {}


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

//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP I2C with DMA
//*******************************************************
#ifndef RPLIB_I2C_H
#define RPLIB_I2C_H


//-------------------------------------------------------
// Defines
//-------------------------------------------------------

#include "rp-peripherals.h"
#include <Wire.h>
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

// select i2c bus based on hal define
#ifdef I2C_USE_WIRE1
  #define I2C_WIRE  Wire1
  #define I2C_INST  i2c1
  #define I2C_IRQ   I2C1_IRQ
#else
  #define I2C_WIRE  Wire
  #define I2C_INST  i2c0
  #define I2C_IRQ   I2C0_IRQ
#endif


//-------------------------------------------------------
// DMA I2C state
//-------------------------------------------------------
// DMA transfers 16-bit command words to the I2C data_cmd register.
// Each word contains a data byte [7:0] plus control bits (RESTART, STOP).
// The I2C peripheral handles bus timing; DMA is paced by the TX FIFO DREQ.
// Transfer completion is detected via STOP_DET interrupt, not DMA IRQ,
// because the DMA finishes writing to the FIFO before I2C finishes on the bus.

// 1 register byte + 1024 data bytes = 1025 DMA words
#define I2C_DMA_CMD_BUF_SIZE  1025

typedef struct {
    int dma_chan;
    uint16_t cmd_buffer[I2C_DMA_CMD_BUF_SIZE];
    volatile bool transfer_active;
    volatile bool stop_detected;
    volatile bool abort_detected;
} i2c_dma_state_t;

static i2c_dma_state_t i2c_dma;


//-------------------------------------------------------
// I2C IRQ handler (STOP_DET and TX_ABRT)
//-------------------------------------------------------

void __not_in_flash_func(i2c_dma_irq_handler)(void)
{
    i2c_hw_t* hw = i2c_get_hw(I2C_INST);
    uint32_t status = hw->intr_stat;

    if (status & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) {
        (void)hw->clr_tx_abrt; // read-to-clear
        i2c_dma.abort_detected = true;
        i2c_dma.transfer_active = false;
        dma_channel_abort(i2c_dma.dma_chan);
    }

    if (status & I2C_IC_INTR_STAT_R_STOP_DET_BITS) {
        (void)hw->clr_stop_det; // read-to-clear
        i2c_dma.stop_detected = true;
        i2c_dma.transfer_active = false;
    }
}


//-------------------------------------------------------
// DMA I2C internal helpers
//-------------------------------------------------------

static void i2c_dma_set_target_addr(uint8_t addr)
{
    i2c_hw_t* hw = i2c_get_hw(I2C_INST);
    hw->enable = 0;
    hw->tar = addr;
    hw->enable = 1;
}


// format cmd_buffer for a write: reg byte + data bytes
// returns total word count, or 0 on error
static uint16_t i2c_dma_format_write(uint8_t reg, const uint8_t* data, uint16_t len)
{
    if (len == 0 || len > I2C_DMA_CMD_BUF_SIZE - 1) return 0;

    // first word: register byte with RESTART to generate START condition
    i2c_dma.cmd_buffer[0] = (uint16_t)reg | I2C_IC_DATA_CMD_RESTART_BITS;

    // data bytes (all except last)
    for (uint16_t i = 0; i < len - 1; i++) {
        i2c_dma.cmd_buffer[1 + i] = (uint16_t)data[i];
    }

    // last data byte with STOP
    i2c_dma.cmd_buffer[len] = (uint16_t)data[len - 1] | I2C_IC_DATA_CMD_STOP_BITS;

    return len + 1; // reg byte + data bytes
}


static bool i2c_dma_start(uint16_t total_words)
{
    if (i2c_dma.transfer_active) return false;

    i2c_dma.transfer_active = true;
    i2c_dma.stop_detected = false;
    i2c_dma.abort_detected = false;

    dma_channel_set_read_addr(i2c_dma.dma_chan, i2c_dma.cmd_buffer, false);
    dma_channel_set_trans_count(i2c_dma.dma_chan, total_words, true);

    return true;
}


static void i2c_dma_handle_error(void)
{
    dma_channel_abort(i2c_dma.dma_chan);

    // reinitialize I2C peripheral to recover
    i2c_deinit(I2C_INST);
    i2c_init(I2C_INST, I2C_CLOCKSPEED);

    // re-enable DMA requests and interrupts
    i2c_get_hw(I2C_INST)->dma_cr = I2C_IC_DMA_CR_TDMAE_BITS;
    i2c_get_hw(I2C_INST)->intr_mask =
        I2C_IC_INTR_MASK_M_STOP_DET_BITS |
        I2C_IC_INTR_MASK_M_TX_ABRT_BITS;

    i2c_dma.transfer_active = false;
    i2c_dma.stop_detected = false;
    i2c_dma.abort_detected = false;
}


//-------------------------------------------------------
//  I2C user routines
//-------------------------------------------------------

uint8_t i2c_dev_adr;


extern "C" void i2c_setdeviceadr(uint8_t dev_adr)
{
    i2c_dev_adr = dev_adr;
}


// blocking transfer via Wire - used for display init commands
// disable DMA IRQ to prevent handler from clearing STOP_DET before
// the SDK's i2c_write_blocking_internal can poll it (race on RP2350)
extern "C" HAL_StatusTypeDef i2c_put_blocked(uint8_t reg_adr, uint8_t* buf, uint16_t len)
{
    irq_set_enabled(I2C_IRQ, false);
    I2C_WIRE.beginTransmission(i2c_dev_adr);
    I2C_WIRE.write(reg_adr);
    I2C_WIRE.write(buf, len);
    uint8_t error = I2C_WIRE.endTransmission(true);
    irq_set_enabled(I2C_IRQ, true);
    return (error == 0) ? HAL_OK : HAL_ERROR;
}


// async DMA transfer - used for framebuffer updates
// SSD1306 must be in horizontal addressing mode (0x20, 0x00) with full
// column range (0x21, 0, 127) and page range (0x22, 0, 7) so that a single
// 1024-byte data write covers the entire framebuffer without per-page commands.
extern "C" HAL_StatusTypeDef i2c_put(uint8_t reg_adr, uint8_t* buf, uint16_t len)
{
    if (i2c_dma.transfer_active) return HAL_BUSY;

    if (i2c_dma.abort_detected) {
        i2c_dma_handle_error();
    }

    i2c_dma_set_target_addr(i2c_dev_adr);
    uint16_t words = i2c_dma_format_write(reg_adr, buf, len);
    if (words == 0) return HAL_ERROR;
    if (!i2c_dma_start(words)) return HAL_BUSY;

    return HAL_OK;
}



extern "C" HAL_StatusTypeDef i2c_device_ready(void)
{
    if (i2c_dma.transfer_active) return HAL_BUSY;
    return HAL_OK;
}


// stub: RP uses Core 1 DMA for I2C, no CPU spinning needed
void i2c_spin(uint16_t chunksize) {}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

void i2c_init(void)
{
    // Wire is still used for blocking init commands (i2c_put_blocked)
    I2C_WIRE.setSDA(I2C_SDA_IO);
    I2C_WIRE.setSCL(I2C_SCL_IO);
    I2C_WIRE.setClock(I2C_CLOCKSPEED);
    I2C_WIRE.begin();

    // DMA I2C setup - uses the same hardware peripheral that Wire configured
    // GPIO and I2C clock are already set up by Wire.begin() above

    // enable TX DMA requests on the I2C peripheral
    i2c_get_hw(I2C_INST)->dma_cr = I2C_IC_DMA_CR_TDMAE_BITS;

    // enable STOP_DET and TX_ABRT interrupts
    i2c_get_hw(I2C_INST)->intr_mask =
        I2C_IC_INTR_MASK_M_STOP_DET_BITS |
        I2C_IC_INTR_MASK_M_TX_ABRT_BITS;

    // claim and configure DMA channel
    i2c_dma.dma_chan = dma_claim_unused_channel(true);

    dma_channel_config cfg = dma_channel_get_default_config(i2c_dma.dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, i2c_get_dreq(I2C_INST, true)); // TX DREQ

    dma_channel_configure(i2c_dma.dma_chan, &cfg,
        &i2c_get_hw(I2C_INST)->data_cmd, // destination: I2C data_cmd register
        i2c_dma.cmd_buffer,               // source (updated before each transfer)
        0,                                 // count set at start time
        false);                            // don't start yet

    // I2C IRQ for transfer completion (STOP_DET) and errors (TX_ABRT)
    irq_set_exclusive_handler(I2C_IRQ, i2c_dma_irq_handler);
    irq_set_enabled(I2C_IRQ, true);

    i2c_dma.transfer_active = false;
    i2c_dma.stop_detected = false;
    i2c_dma.abort_detected = false;
}


#endif // RPLIB_I2C_H

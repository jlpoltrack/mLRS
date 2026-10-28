//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// DroneCAN Driver for RP2350 using can2040
// for use with libcanard
//*******************************************************
// this driver implements the same dc_hal_* API as the
// STM32 driver, backed by the can2040 PIO-based CAN
// implementation.
//*******************************************************
#ifndef RP_DRONECAN_DRIVER_H
#define RP_DRONECAN_DRIVER_H

#include <stdint.h>
#include "../../modules/stm32-dronecan-lib/libcanard/canard.h"

// library configuration
#define DRONECAN_USE_RX_ISR
#define DRONECAN_RXFRAMEBUFSIZE   64 // actual size is RXFRAMEBUFSIZE * sizeof(struct can2040_msg)
#define DRONECAN_IRQ_PRIORITY     PICO_HIGHEST_IRQ_PRIORITY // can2040 needs low irq latency, it is alone on core 1

// build with CAN2040_DEBUG_STATS=1 for the extended can2040 driver statistics
#ifndef CAN2040_DEBUG_STATS
#define CAN2040_DEBUG_STATS       0
#endif


typedef enum
{
    DC_HAL_ERROR_INVALID_ARGUMENT             = 1000,
    DC_HAL_ERROR_UNSUPPORTED_BIT_RATE         = 1001,
    DC_HAL_ERROR_UNSUPPORTED_FRAME_FORMAT     = 1002,

    DC_HAL_ERROR_CAN_INIT                     = 2000,
    DC_HAL_ERROR_CAN_CONFIG_FILTER            = 2001,
    DC_HAL_ERROR_CAN_CONFIG_GLOBAL_FILTER     = 2002,
    DC_HAL_ERROR_CAN_START                    = 2003,
    DC_HAL_ERROR_CAN_ADD_TX_MESSAGE           = 2004,
    DC_HAL_ERROR_CAN_GET_RX_MESSAGE           = 2005,

    DC_HAL_ERROR_UNSUPPORTED_CLOCK_FREQUENCY  = 3000,
    DC_HAL_ERROR_TIMING                       = 3001,

    DC_HAL_ERROR_ISR_CONFIG                   = 4000,
} DC_HAL_ERROR_ENUM;


typedef enum
{
    DC_HAL_IFACE_MODE_NORMAL = 0,
    DC_HAL_IFACE_MODE_SILENT,
    DC_HAL_IFACE_MODE_AUTOMATIC_TX_ABORT_ON_ERROR
} DC_HAL_IFACE_MODE_ENUM;


typedef struct
{
    uint32_t rx_overflow_count;
    uint32_t parse_error_count;
    uint32_t rx_total;
    uint32_t tx_total;
    uint32_t tx_attempt;
    uint32_t tx_abort_count;
    uint32_t error_sum_count;
    // MCP251xFD only, tec, rec are the current values as on STM32
    uint32_t tec_count;
    uint32_t rec_count;
    uint32_t bo_count; // only counts when toggled from bus on to bus off
    uint32_t data_tx_error_count; // data bit rate phase, CAN FD
    uint32_t data_rx_error_count;
    uint32_t ecc_error_count;
    uint32_t opmode; // CiCON.OPMOD
    uint32_t iocon; // XSTBYEN = bit 6, GPIO0 = bit 16 is the XSTBY pin level
#if CAN2040_DEBUG_STATS // can2040 only
    uint32_t can2040_parse_error;
    uint32_t tx_reject_count;
    uint32_t irq_max_us, irq_busy_us; // irq_busy_us is cumulative
    uint32_t rx_buf_max, tx_buf_max; // ring buffer high-water marks
#endif
} tDcHalStatistics;


typedef enum
{
    DC_HAL_RX_FIFO_DEFAULT = 0,
    DC_HAL_RX_FIFO0,
    DC_HAL_RX_FIFO1,
} DC_HAL_RX_FIFO_ENUM;


typedef struct
{
    uint32_t id;
    uint32_t mask;
    uint8_t rx_fifo;
} tDcHalAcceptanceFilterConfiguration;


typedef struct
{
    uint16_t bit_rate_prescaler;
    uint8_t bit_segment_1;
    uint8_t bit_segment_2;
    uint8_t sync_jump_width;
} tDcHalCanTimings;


typedef enum
{
    DC_HAL_CAN1 = 0,
    DC_HAL_CAN2,
} DC_HAL_CAN_ENUM;


void dc_hal_set_gpio(uint32_t gpio_rx, uint32_t gpio_tx);

// MCP2518FD/MCP251863 driver only, replaces dc_hal_set_gpio()
typedef struct
{
    uint8_t spi_num;
    uint8_t sck_pin;
    uint8_t mosi_pin;
    uint8_t miso_pin;
    uint8_t cs_pin;
    uint8_t int_pin; // INT, not INT0 or INT1
    uint32_t spi_hz;
    bool canfd;
    bool standby; // reset the chip into low power mode, INT0/XSTBY stays an input and the transceiver in standby
} tDcHalMcpConfig;

void dc_hal_set_mcp_config(const tDcHalMcpConfig* const config);

int16_t dc_hal_init(
    DC_HAL_CAN_ENUM can_instance,
    const tDcHalCanTimings* const timings,
    const DC_HAL_IFACE_MODE_ENUM iface_mode);

int16_t dc_hal_start(void);

uint8_t dc_hal_is_canfd(void);

int16_t dc_hal_transmit(const CanardCANFrame* const frame, uint32_t tnow_ms);

int16_t dc_hal_receive(CanardCANFrame* const frame);

int16_t dc_hal_config_acceptance_filters(
    const tDcHalAcceptanceFilterConfiguration* const filter_configs,
    const uint8_t num_filter_configs);

tDcHalStatistics dc_hal_get_stats(void);

int16_t dc_hal_compute_timings(
    const uint32_t peripheral_clock_rate,
    const uint32_t target_bitrate,
    tDcHalCanTimings* const timings);


#ifdef DRONECAN_USE_RX_ISR
int16_t dc_hal_enable_isr(void);
void dc_hal_rx_flush(void);
#endif

// called from the core 1 loop to run IRQ registration + can2040_start
// on the correct core (core 1), keeping CAN IRQ off the radio core (core 0)
void dc_hal_poll_core1(void);


#endif // RP_DRONECAN_DRIVER_H

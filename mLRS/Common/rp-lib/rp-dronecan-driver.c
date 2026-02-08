//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// 2026-02-08
//*******************************************************
// DroneCAN Driver for RP2040/RP2350 using can2040
// for use with libcanard
//*******************************************************
#if defined ARDUINO_ARCH_RP2040 || defined ARDUINO_ARCH_RP2350

#include <string.h>
#include <hardware/irq.h>
#include "can2040.h"
#include "rp-dronecan-driver.h"


//-------------------------------------------------------
// rx ring buffer
//-------------------------------------------------------

#define DRONECAN_RXFRAMEBUFSIZEMASK  (DRONECAN_RXFRAMEBUFSIZE - 1)

typedef struct
{
    CanardCANFrame frames[DRONECAN_RXFRAMEBUFSIZE];
    volatile uint16_t writepos;
    volatile uint16_t readpos;
} tDcHalRxBuf;

static tDcHalRxBuf dc_hal_rxbuf;

static tDcHalStatistics dc_hal_stats;

static struct can2040 dc_hal_cbus;

// stored from init, used by dc_hal_start
static uint32_t dc_hal_gpio_rx;
static uint32_t dc_hal_gpio_tx;
static uint32_t dc_hal_bitrate;


//-------------------------------------------------------
// can2040 callback
//-------------------------------------------------------
// called in irq context on every received/transmitted frame

static void dc_hal_can2040_cb(struct can2040* cd, uint32_t notify, struct can2040_msg* msg)
{
    (void)cd;

    if (notify == CAN2040_NOTIFY_RX) {
        uint16_t wp = dc_hal_rxbuf.writepos;
        uint16_t next = (wp + 1) & DRONECAN_RXFRAMEBUFSIZEMASK;
        if (next != dc_hal_rxbuf.readpos) { // fifo not full
            CanardCANFrame* frame = &dc_hal_rxbuf.frames[wp];

            frame->id = msg->id & 0x1FFFFFFF; // 29-bit id
            if (msg->id & CAN2040_ID_EFF) {
                frame->id |= CANARD_CAN_FRAME_EFF;
            }
            if (msg->id & CAN2040_ID_RTR) {
                frame->id |= CANARD_CAN_FRAME_RTR;
            }

            frame->data_len = (msg->dlc > 8) ? 8 : msg->dlc;
            memcpy((uint8_t*)frame->data, msg->data, frame->data_len);
            // zero remaining bytes
            for (uint8_t n = frame->data_len; n < CANARD_CAN_FRAME_MAX_DATA_LEN; n++) {
                frame->data[n] = 0;
            }

            frame->iface_id = 0;

            dc_hal_rxbuf.writepos = next;
        } else {
            dc_hal_stats.rx_overflow_count++;
        }
    } else if (notify == CAN2040_NOTIFY_ERROR) {
        dc_hal_stats.parse_error_count++;
    }
}


//-------------------------------------------------------
// PIO1 IRQ handler
//-------------------------------------------------------

static void dc_hal_pio1_irq_handler(void)
{
    can2040_pio_irq_handler(&dc_hal_cbus);
}


//-------------------------------------------------------
// gpio pin configuration
//-------------------------------------------------------

void dc_hal_set_gpio(uint32_t gpio_rx, uint32_t gpio_tx)
{
    dc_hal_gpio_rx = gpio_rx;
    dc_hal_gpio_tx = gpio_tx;
}


//-------------------------------------------------------
// init
//-------------------------------------------------------

int16_t dc_hal_init(
    DC_HAL_CAN_ENUM can_instance,
    const tDcHalCanTimings* const timings,
    const DC_HAL_IFACE_MODE_ENUM iface_mode)
{
    (void)can_instance;
    (void)iface_mode;

    memset(&dc_hal_stats, 0, sizeof(dc_hal_stats));
    memset(&dc_hal_rxbuf, 0, sizeof(dc_hal_rxbuf));

    // store bitrate from timings for later use in dc_hal_start
    // for can2040, timings are not directly used — it computes its own
    // from sys_clock and bitrate. we use the prescaler field to pass
    // the bitrate (in kbit/s) if needed, but default to 1000000.
    if (timings != NULL && timings->bit_rate_prescaler != 0) {
        dc_hal_bitrate = (uint32_t)timings->bit_rate_prescaler * 1000;
    } else {
        dc_hal_bitrate = 1000000; // 1 mbit/s default
    }

    // set up can2040 on PIO1
    can2040_setup(&dc_hal_cbus, 1); // pio_num = 1 (PIO1)
    can2040_callback_config(&dc_hal_cbus, dc_hal_can2040_cb);

    return 0;
}


int16_t dc_hal_start(void)
{
    uint32_t sys_clock = F_CPU; // set by platformio board_build.f_cpu

    can2040_start(&dc_hal_cbus, sys_clock, dc_hal_bitrate,
                  dc_hal_gpio_rx, dc_hal_gpio_tx);

    return 0;
}


//-------------------------------------------------------
// transmit
//-------------------------------------------------------

int16_t dc_hal_transmit(const CanardCANFrame* const frame, uint32_t tnow_ms)
{
    (void)tnow_ms;

    if (frame == NULL) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    if (frame->id & CANARD_CAN_FRAME_ERR) {
        return -DC_HAL_ERROR_UNSUPPORTED_FRAME_FORMAT;
    }

    if (frame->data_len > 8) {
        return -DC_HAL_ERROR_UNSUPPORTED_FRAME_FORMAT;
    }

    struct can2040_msg msg;
    msg.id = frame->id & CANARD_CAN_EXT_ID_MASK;
    if (frame->id & CANARD_CAN_FRAME_EFF) {
        msg.id |= CAN2040_ID_EFF;
    }
    if (frame->id & CANARD_CAN_FRAME_RTR) {
        msg.id |= CAN2040_ID_RTR;
    }
    msg.dlc = frame->data_len;
    memcpy(msg.data, frame->data, frame->data_len);

    int res = can2040_transmit(&dc_hal_cbus, &msg);
    if (res < 0) {
        return 0; // tx queue full, postpone
    }

    return 1;
}


//-------------------------------------------------------
// receive
//-------------------------------------------------------

int16_t dc_hal_receive(CanardCANFrame* const frame)
{
    if (frame == NULL) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    if (dc_hal_rxbuf.writepos == dc_hal_rxbuf.readpos) {
        return 0; // fifo empty
    }

    uint16_t rp = dc_hal_rxbuf.readpos;
    memcpy(frame, &dc_hal_rxbuf.frames[rp], sizeof(CanardCANFrame));
    dc_hal_rxbuf.readpos = (rp + 1) & DRONECAN_RXFRAMEBUFSIZEMASK;

    return 1;
}


void dc_hal_rx_flush(void)
{
    irq_set_enabled(PIO1_IRQ_0, false);
    dc_hal_rxbuf.writepos = 0;
    dc_hal_rxbuf.readpos = 0;
    dc_hal_stats.rx_overflow_count = 0;
    irq_set_enabled(PIO1_IRQ_0, true);
}


//-------------------------------------------------------
// ISR setup
//-------------------------------------------------------

int16_t dc_hal_enable_isr(void)
{
    // PIO1_IRQ_0 for can2040 on PIO1
    irq_set_exclusive_handler(PIO1_IRQ_0, dc_hal_pio1_irq_handler);
    irq_set_priority(PIO1_IRQ_0, DRONECAN_IRQ_PRIORITY);
    irq_set_enabled(PIO1_IRQ_0, true);

    return 0;
}


//-------------------------------------------------------
// filters (no-op for can2040)
//-------------------------------------------------------
// can2040 does not support hardware filtering.
// all filtering is done in software by libcanard's
// dronecan_should_accept_transfer callback.

int16_t dc_hal_config_acceptance_filters(
    const tDcHalAcceptanceFilterConfiguration* const filter_configs,
    const uint8_t num_filter_configs)
{
    (void)filter_configs;
    (void)num_filter_configs;
    return 0;
}


//-------------------------------------------------------
// statistics
//-------------------------------------------------------

tDcHalStatistics dc_hal_get_stats(void)
{
    struct can2040_stats stats;
    can2040_get_statistics(&dc_hal_cbus, &stats);

    dc_hal_stats.rx_total = stats.rx_total;
    dc_hal_stats.tx_total = stats.tx_total;
    dc_hal_stats.tx_attempt = stats.tx_attempt;
    // parse_error_count is accumulated by the ISR callback, don't overwrite
    dc_hal_stats.error_sum_count = dc_hal_stats.rx_overflow_count +
                                   dc_hal_stats.parse_error_count;

    return dc_hal_stats;
}


//-------------------------------------------------------
// timings (simplified for can2040)
//-------------------------------------------------------
// can2040 computes its own bit timing from sys_clock
// and bitrate. this function just validates the request.

int16_t dc_hal_compute_timings(
    const uint32_t peripheral_clock_rate,
    const uint32_t target_bit_rate,
    tDcHalCanTimings* const timings)
{
    (void)peripheral_clock_rate;

    if (target_bit_rate != 1000000) {
        return -DC_HAL_ERROR_UNSUPPORTED_BIT_RATE;
    }

    if (timings == NULL) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    // store bitrate in prescaler field as a convention
    // dc_hal_init will read this
    timings->bit_rate_prescaler = (uint16_t)(target_bit_rate / 1000);
    timings->bit_segment_1 = 1;
    timings->bit_segment_2 = 1;
    timings->sync_jump_width = 1;

    return 0;
}


#endif // ARDUINO_ARCH_RP2040 || ARDUINO_ARCH_RP2350

//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// CAN initialization for RP2040/RP2350
//*******************************************************
#ifndef RP_CAN_H
#define RP_CAN_H

#include <hardware/clocks.h>
#include <hardware/pwm.h>
#include "rp-dronecan-driver.h"


#ifdef DEVICE_HAS_DRONECAN_FD

void can_init(bool canfd)
{
#ifdef CAN_CLK_OUT_PIN
    // 40 MHz chip clock from pwm, sys clock must be a multiple of 80 MHz for 50% duty
    gpio_set_function(CAN_CLK_OUT_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(CAN_CLK_OUT_PIN);
    uint16_t period = clock_get_hz(clk_sys) / 40000000;
    pwm_set_clkdiv_int_frac(slice, 1, 0);
    pwm_set_wrap(slice, period - 1);
    pwm_set_gpio_level(CAN_CLK_OUT_PIN, period / 2);
    pwm_set_enabled(slice, true);
#endif

    // MCP2518FD/MCP251863 on its own spi bus, bit timings are fixed in the driver
    tDcHalMcpConfig config = {
        .spi_num = CAN_SPI_NUM,
        .sck_pin = CAN_SPI_SCK,
        .mosi_pin = CAN_SPI_MOSI,
        .miso_pin = CAN_SPI_MISO,
        .cs_pin = CAN_SPI_CS_IO,
        .int_pin = CAN_INT,
        .spi_hz = CAN_SPI_FREQUENCY,
        .canfd = canfd,
    };
    dc_hal_set_mcp_config(&config);

    dc_hal_init(DC_HAL_CAN1, nullptr, DC_HAL_IFACE_MODE_NORMAL);
}

#else

void can_init(bool canfd)
{
    (void)canfd; // can2040 is classic CAN only, rp boards declare DEVICE_HAS_DRONECAN

    // set can gpio pins from hal defines
    dc_hal_set_gpio(CAN_RX_PIN, CAN_TX_PIN);

    // can2040 does its own timing from sys_clock + bitrate,
    // but the dc_hal api expects timings to be passed through
    tDcHalCanTimings timings;
    int16_t res = dc_hal_compute_timings(clock_get_hz(clk_sys), 1000000, &timings);
    if (res < 0) return;

    dc_hal_init(DC_HAL_CAN1, &timings, DC_HAL_IFACE_MODE_NORMAL);
}

#endif // DEVICE_HAS_DRONECAN_FD


#endif // RP_CAN_H

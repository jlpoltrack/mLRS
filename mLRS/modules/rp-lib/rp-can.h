//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// CAN initialization for RP2040/RP2350
//*******************************************************
#ifndef RP_CAN_H
#define RP_CAN_H

#include "rp-dronecan-driver.h"


void can_init(void)
{
    // set can gpio pins from hal defines
    dc_hal_set_gpio(CAN_RX_PIN, CAN_TX_PIN);

    // can2040 does its own timing from sys_clock + bitrate,
    // but the dc_hal api expects timings to be passed through
    tDcHalCanTimings timings;
    int16_t res = dc_hal_compute_timings(F_CPU, 1000000, &timings);
    if (res < 0) return;

    res = dc_hal_init(DC_HAL_CAN1, &timings, DC_HAL_IFACE_MODE_NORMAL);
    if (res < 0) return;
}


#endif // RP_CAN_H

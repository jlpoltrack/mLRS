//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// CAN standard library
// only init functions
//*******************************************************
// Interface:
// CAN_USE_FDCAN1_PA11PA12
// CAN_USE_FDCAN2_PB5PB6
// CAN_USE_FDCAN_CLOCK_PCLK1
// CAN_USE_FDCAN_CLOCK_PLL
//*******************************************************
#ifndef STDSTM32_CAN_H
#define STDSTM32_CAN_H
#ifdef __cplusplus
extern "C" {
#endif


//-------------------------------------------------------
// Defines
//-------------------------------------------------------

//#include "stdstm32-peripherals.h"

#ifdef STM32G4
#if defined CAN_USE_FDCAN1_PA11PA12
    #define CAN_DC_HAL_INTFC    DC_HAL_CAN1 // TODO: this is currently defined in stm32-dronecan-driver.h
    #define CAN_RX_IO           IO_PA11
    #define CAN_TX_IO           IO_PA12
#elif defined CAN_USE_FDCAN2_PB5PB6
    #define CAN_DC_HAL_INTFC    DC_HAL_CAN2
    #define CAN_RX_IO           IO_PB5
    #define CAN_TX_IO           IO_PB6
    #ifndef FDCAN2
      #error CAN_USE_FDCAN2_xxxx defined buf FDCAN2 not available!
    #endif
#else
    #warning CAN_USE_FDCANx_xxxx not defined! CAN_USE_FDCAN1_PA11PA12 assumed.
    #defined CAN_USE_FDCAN1_PA11PA12
    #define CAN_DC_HAL_INTFC    DC_HAL_CAN1
    #define CAN_RX_IO           IO_PA11
    #define CAN_TX_IO           IO_PA12
#endif
#endif


#ifdef STM32C5
// the C552 in LQFP48 has FDCAN1 on PB12/PB13 only. Both pins are AF9, as on the G4 and
// the H5 - DS14928 lists the alternate functions in ascending AF order, and on PB13 the
// sequence USART3(AF7), LPUART1(AF8), FDCAN1, UART5 puts FDCAN1 at AF9. PB12 has the same
// sequence without the LPUART1 entry, so FDCAN1_RX is AF9 there too and AF10 is UART5_RX.
#if defined CAN_USE_FDCAN1_PB12PB13
    #define CAN_DC_HAL_INTFC    DC_HAL_CAN1
    #define CAN_RX_IO           IO_PB12
    #define CAN_RX_IO_AF        IO_AF_9
    #define CAN_TX_IO           IO_PB13
    #define CAN_TX_IO_AF        IO_AF_9
#else
    #error CAN_USE_FDCAN1_PB12PB13 must be defined for STM32C5 !
#endif
#endif


#define CAN_BITRATE             1000000
#define CAN_FD_DATA_BITRATE     4000000


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

#ifdef STM32F1
//-- STM32F1

// This code has once worked, but may not anymore today, hasn't been tested/used for a while

#ifndef HAL_CAN_MODULE_ENABLED
  #error HAL_CAN_MODULE_ENABLED not defined, enable it in Core\Inc\stm32f1xx_hal_conf.h!
#endif


void can_init(uint8_t canfd_enable) // canfd_enable is ignored, STM32F1 is classic CAN only
{
    // CAN peripheral initialization

    gpio_init(IO_PA11, IO_MODE_INPUT_PU, IO_SPEED_VERYFAST);
    gpio_init_af(IO_PA12, IO_MODE_OUTPUT_ALTERNATE_PP, IO_AF_9, IO_SPEED_VERYFAST);

    //__HAL_RCC_CAN1_CLK_ENABLE();
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CAN1);

    // DroneCAN Hal initialization

    tDcHalCanTimings timings;
    int16_t res = dc_hal_compute_timings(HAL_RCC_GetPCLK1Freq(), CAN_BITRATE, &timings); // = 36000000, CAN is on slow APB1 bus
    if (res < 0) {
        dbg.puts("\nERROR: Solution for CAN timings could not be found");
        return;
    }

    DBG_DC(dbg.puts("\n  PCLK1: ");dbg.puts(u32toBCD_s(HAL_RCC_GetPCLK1Freq()));
    dbg.puts("\n  Prescaler: ");dbg.puts(u16toBCD_s(timings.bit_rate_prescaler));
    dbg.puts("\n  BS1: ");dbg.puts(u8toBCD_s(timings.bit_segment_1));
    dbg.puts("\n  BS2: ");dbg.puts(u8toBCD_s(timings.bit_segment_2));
    dbg.puts("\n  SJW: ");dbg.puts(u8toBCD_s(timings.sync_jump_width)));
    // 4, 7, 1, 1

    res = dc_hal_init(DC_HAL_CAN1, &timings, NULL, DC_HAL_IFACE_MODE_AUTOMATIC_TX_ABORT_ON_ERROR);
    if (res < 0) {
        dbg.puts("\nERROR: Failed to open CAN iface ");dbg.puts(s16toBCD_s(res));
        return;
    }
}


#elif defined STM32G4
//-- STM32G4
// FDCAN can be clocked by either PCLK1 or PLLQ.
// JLP found:
// - with 170 MHz, FDCAN works reasonable only for 1 Mbps data rate, not useful for FDCAN.
// - 170 MHz PCLK1 with 85 MHz for FDCAN was found to cause excessive stuff errors
//   and timing incompatibilities with ArduPilot at 5 Mbps. ArduPilot uses 80 MHz FDCAN clock,
//   and the sample point / TDC timing differences at 85 MHz are problematic.
// ChatGPT is very clear on that 80 MHz is the best choice, giving many reasons, e.g.,
// better clock stability, more precise/cleaner timing points.
// This requires 160 MHz SYSCLK and 80 MHz for the PLL

#ifndef HAL_FDCAN_MODULE_ENABLED
  #error HAL_FDCAN_MODULE_ENABLED not defined, enable it in Core\Inc\stm32g4xx_hal_conf.h!
#endif


void can_init(uint8_t canfd_enable)
{
    // GPIO initialization
    // PA11 = FDCAN1_RX
    // PA12 = FDCAN1_TX
    gpio_init_af(CAN_RX_IO, IO_MODE_OUTPUT_ALTERNATE_PP, IO_AF_9, IO_SPEED_VERYFAST);
    gpio_init_af(CAN_TX_IO, IO_MODE_OUTPUT_ALTERNATE_PP, IO_AF_9, IO_SPEED_VERYFAST);

    // FDCAN clock initialization
#ifdef CAN_USE_FDCAN_CLOCK_PCLK1
    LL_RCC_SetFDCANClockSource(LL_RCC_FDCAN_CLKSOURCE_PCLK1);
#elif defined CAN_USE_FDCAN_CLOCK_PLL
    // Configure the FDCAN interface clock source
    // HAL function HAL_RCCEx_PeriphCLKConfig() does two things:
    //   __HAL_RCC_FDCAN_CONFIG(RCC_PERIPHCLK_FDCAN);
    //   __HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL_48M1CLK);
    // The first does this:
    //   #define RCC_PERIPHCLK_FDCAN            0x00001000U
    //   MODIFY_REG(RCC->CCIPR, RCC_CCIPR_FDCANSEL, (uint32_t)(__FDCAN_CLKSOURCE__))
    //   STRANGE: The macro's comment says to use RCC_FDCANCLKSOURCE_PLL !!!! (which would matche the LL function)
    //   THIS IS DIFFERENT! IS THIS A BUG??
    // LL function LL_RCC_SetFDCANClockSource(LL_RCC_FDCAN_CLKSOURCE_PLL) does this
    //   #define LL_RCC_FDCAN_CLKSOURCE_PLL     RCC_CCIPR_FDCANSEL_0    // PLL clock used as FDCAN clock source
    //   #define RCC_CCIPR_FDCANSEL_0           (0x1UL << RCC_CCIPR_FDCANSEL_Pos) // 0x01000000
    //   MODIFY_REG(RCC->CCIPR, RCC_CCIPR_FDCANSEL, FDCANxSource);

    LL_RCC_SetFDCANClockSource(LL_RCC_FDCAN_CLKSOURCE_PLL);
    LL_RCC_PLL_EnableDomain_48M();
#else
    #error No clock defined in stdstm32-can.h !
#endif
    uint32_t peripheral_clock_rate = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_FDCAN);
    //LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_FDCAN);
    //LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_FDCAN);

    // DroneCAN HAL initialization

    tDcHalCanTimings timings;
    int16_t res = dc_hal_compute_timings(peripheral_clock_rate, CAN_BITRATE, &timings);
    if (res < 0) {
        DBG_DC(dbg.puts("\nERROR: Solution for CAN timings could not be found");)
        return;
    }

    DBG_DC(dbg.puts("\n  PCLK1: ");dbg.puts(u32toBCD_s(HAL_RCC_GetPCLK1Freq()));
    dbg.puts("\n  FDCAN CLK: ");dbg.puts(u32toBCD_s(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN)));
    dbg.puts("\n  Prescaler: ");dbg.puts(u16toBCD_s(timings.bit_rate_prescaler));
    dbg.puts("\n  BS1: ");dbg.puts(u8toBCD_s(timings.bit_segment_1));
    dbg.puts("\n  BS2: ");dbg.puts(u8toBCD_s(timings.bit_segment_2));
    dbg.puts("\n  SJW: ");dbg.puts(u8toBCD_s(timings.sync_jump_width));)

    tDcHalCanDataTimings data_timings;
    if (canfd_enable) {
        res = dc_hal_compute_data_timings(peripheral_clock_rate, CAN_FD_DATA_BITRATE, &data_timings);
        if (res < 0) {
            DBG_DC(dbg.puts("\nERROR: Solution for CAN FD data timings could not be found");)
            return;
        }
    }

    tDcHalCanDataTimings* data_timings_ptr = (canfd_enable) ? &data_timings : NULL;

    res = dc_hal_init(CAN_DC_HAL_INTFC, &timings, data_timings_ptr, DC_HAL_IFACE_MODE_AUTOMATIC_TX_ABORT_ON_ERROR);
    if (res < 0) {
        DBG_DC(dbg.puts("\nERROR: Failed to open CAN iface ");dbg.puts(s16toBCD_s(res));)
        return;
    }
}

#elif defined STM32C5
//-- STM32C5
// The FDCAN kernel clock can be sourced from PCLK1, PSIS, PSIK or HSE. The C5 has no PLL,
// so there is no free PLL output to dial in the 80 MHz the G4 and H5 targets use (see the
// comments for the G4 above): PSIK is the only source that can be divided independently of
// SYSCLK, and with PSI at 144 MHz the reachable set is 144/n for n in 0.5 steps, so PSIK/2
// = 72 MHz is the closest usable value. dc_hal_compute_timings() is a lookup table, not a
// solver, so 72 MHz needs its own entries there - see stm32-dronecan-driver-c5.c.
// Note this rules out a 5 Mbps data rate: 72/5 is not a whole number of tq, and no
// divider can fix that since 144 has no factor of 5. 1/2/4/8 Mbps are all exact.

#if !defined USE_HAL_FDCAN_MODULE || (USE_HAL_FDCAN_MODULE != 1)
  #error USE_HAL_FDCAN_MODULE not 1, enable it in Core\Inc\stm32c5xx_hal_conf.h!
#endif


void can_init(uint8_t canfd_enable)
{
    // GPIO initialization, both pins are AF9
    gpio_init_af(CAN_RX_IO, IO_MODE_OUTPUT_ALTERNATE_PP, CAN_RX_IO_AF, IO_SPEED_VERYFAST);
    gpio_init_af(CAN_TX_IO, IO_MODE_OUTPUT_ALTERNATE_PP, CAN_TX_IO_AF, IO_SPEED_VERYFAST);

    // FDCAN kernel clock initialization, PSIK = PSI / 2 = 72 MHz.
    // PSIKDIV is owned by FDCAN, the ADC uses HSIK so the two do not fight over it.
    LL_RCC_PSIK_SetDivider(LL_RCC_PSIK_DIV_2);
    LL_RCC_PSIK_Enable();
    while (LL_RCC_PSIK_IsReady() != 1U) {}
    LL_RCC_SetFDCANClockSource(LL_RCC_FDCAN_CLKSOURCE_PSIK);

    uint32_t peripheral_clock_rate = 72000000;

    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_FDCAN); // FDCAN is on APB1H on the C5

    // DroneCAN HAL initialization

    tDcHalCanTimings timings;
    int16_t res = dc_hal_compute_timings(peripheral_clock_rate, CAN_BITRATE, &timings);
    if (res < 0) {
        DBG_DC(dbg.puts("\nERROR: Solution for CAN timings could not be found");)
        return;
    }

    DBG_DC(dbg.puts("\n  FDCAN CLK: ");dbg.puts(u32toBCD_s(peripheral_clock_rate));
    dbg.puts("\n  Prescaler: ");dbg.puts(u16toBCD_s(timings.bit_rate_prescaler));
    dbg.puts("\n  BS1: ");dbg.puts(u8toBCD_s(timings.bit_segment_1));
    dbg.puts("\n  BS2: ");dbg.puts(u8toBCD_s(timings.bit_segment_2));
    dbg.puts("\n  SJW: ");dbg.puts(u8toBCD_s(timings.sync_jump_width));)

    tDcHalCanDataTimings data_timings;
    if (canfd_enable) {
        res = dc_hal_compute_data_timings(peripheral_clock_rate, CAN_FD_DATA_BITRATE, &data_timings);
        if (res < 0) {
            DBG_DC(dbg.puts("\nERROR: Solution for CAN FD data timings could not be found");)
            return;
        }
    }

    tDcHalCanDataTimings* data_timings_ptr = (canfd_enable) ? &data_timings : NULL;

    res = dc_hal_init(CAN_DC_HAL_INTFC, &timings, data_timings_ptr, DC_HAL_IFACE_MODE_AUTOMATIC_TX_ABORT_ON_ERROR);
    if (res < 0) {
        DBG_DC(dbg.puts("\nERROR: Failed to open CAN iface ");dbg.puts(s16toBCD_s(res));)
        return;
    }
}

#endif // STM32C5


//-------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif // STDSTM32_CAN_H

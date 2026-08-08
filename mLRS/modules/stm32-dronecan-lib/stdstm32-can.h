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
// CAN_USE_FDCAN1_RX_xxx, CAN_USE_FDCAN1_TX_xxx (STM32H5, pins are selected separately)
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

#ifdef STM32H5
// The H5 has a lot of FDCAN1 pin options, and RX and TX can be picked independently,
// so they are selected separately here. All of them are AF9, FDCAN1 has no other AF.
// STM32H503 (all pins below exist on the UFQFPN48/LQFP48 package):
//   FDCAN1_RX: PA8, PA11, PB3, PB5, PB8, PB12
//   FDCAN1_TX: PA12, PB4, PB6, PB7, PB10, PB13, PB15
#define CAN_DC_HAL_INTFC        DC_HAL_CAN1 // the H503 has FDCAN1 only

#if defined CAN_USE_FDCAN1_RX_PA8
    #define CAN_RX_IO           IO_PA8
#elif defined CAN_USE_FDCAN1_RX_PA11
    #define CAN_RX_IO           IO_PA11
#elif defined CAN_USE_FDCAN1_RX_PB3
    #define CAN_RX_IO           IO_PB3
#elif defined CAN_USE_FDCAN1_RX_PB5
    #define CAN_RX_IO           IO_PB5
#elif defined CAN_USE_FDCAN1_RX_PB8
    #define CAN_RX_IO           IO_PB8
#elif defined CAN_USE_FDCAN1_RX_PB12
    #define CAN_RX_IO           IO_PB12
#else
    #error CAN_USE_FDCAN1_RX_xxx not defined!
#endif

#if defined CAN_USE_FDCAN1_TX_PA12
    #define CAN_TX_IO           IO_PA12
#elif defined CAN_USE_FDCAN1_TX_PB4
    #define CAN_TX_IO           IO_PB4
#elif defined CAN_USE_FDCAN1_TX_PB6
    #define CAN_TX_IO           IO_PB6
#elif defined CAN_USE_FDCAN1_TX_PB7
    #define CAN_TX_IO           IO_PB7
#elif defined CAN_USE_FDCAN1_TX_PB10
    #define CAN_TX_IO           IO_PB10
#elif defined CAN_USE_FDCAN1_TX_PB13
    #define CAN_TX_IO           IO_PB13
#elif defined CAN_USE_FDCAN1_TX_PB15
    #define CAN_TX_IO           IO_PB15
#else
    #error CAN_USE_FDCAN1_TX_xxx not defined!
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

#elif defined STM32H5
//-- STM32H5
// The FDCAN kernel clock can be sourced from HSE, PLL1Q or PLL2Q only, and it must not
// exceed 80 MHz. On the H503 target PLL1Q is 250 MHz and is the SPI1 kernel clock, so it
// cannot be used here, and HSE would give only 8 tq per bit. PLL2 is otherwise unused and
// is set up here to give exactly the 80 MHz which dc_hal_compute_timings() expects (and
// which ArduPilot uses, see the comments for the G4 above):
//   HSE 8 MHz / M=2 = 4 MHz -> x N=120 -> VCO 480 MHz -> /Q=6 = 80 MHz

#ifndef HAL_FDCAN_MODULE_ENABLED
  #error HAL_FDCAN_MODULE_ENABLED not defined, enable it in Core\Inc\stm32h5xx_hal_conf.h!
#endif


void can_init(uint8_t canfd_enable)
{
    // GPIO initialization
    gpio_init_af(CAN_RX_IO, IO_MODE_OUTPUT_ALTERNATE_PP, IO_AF_9, IO_SPEED_VERYFAST);
    gpio_init_af(CAN_TX_IO, IO_MODE_OUTPUT_ALTERNATE_PP, IO_AF_9, IO_SPEED_VERYFAST);

    // FDCAN kernel clock initialization
    // HAL_RCCEx_PeriphCLKConfig() configures and enables PLL2 and selects it for FDCAN

    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {};

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2Q;
    PeriphClkInitStruct.PLL2.PLL2Source = RCC_PLL2_SOURCE_HSE;
    PeriphClkInitStruct.PLL2.PLL2M = 2; // 8 MHz / 2 = 4 MHz VCO input
    PeriphClkInitStruct.PLL2.PLL2N = 120; // 4 MHz * 120 = 480 MHz VCO output
    PeriphClkInitStruct.PLL2.PLL2P = 2; // not used, but must be a valid value
    PeriphClkInitStruct.PLL2.PLL2Q = 6; // 480 MHz / 6 = 80 MHz
    PeriphClkInitStruct.PLL2.PLL2R = 2; // not used, but must be a valid value
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2_VCIRANGE_2; // VCO input is in 4 .. 8 MHz
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2_VCORANGE_WIDE; // VCO output is in 192 .. 836 MHz
    PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
    PeriphClkInitStruct.PLL2.PLL2ClockOut = RCC_PLL2_DIVQ; // enable the PLL2Q output

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        DBG_DC(dbg.puts("\nERROR: Failed to configure the FDCAN kernel clock");)
        return;
    }

    uint32_t peripheral_clock_rate = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);

    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_FDCAN); // FDCAN is on APB1H on the H5

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

#endif // STM32H5


//-------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif // STDSTM32_CAN_H

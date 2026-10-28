//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP Platform Glue
//*******************************************************
#ifndef RP_GLUE_H
#define RP_GLUE_H


//-------------------------------------------------------
// c-compatible definitions
// (can be included from .c files)
//-------------------------------------------------------

typedef enum {
    DISABLE = 0,
    ENABLE = !DISABLE
} FunctionalState;

typedef enum {
    HAL_OK       = 0x00U,
    HAL_ERROR    = 0x01U,
    HAL_BUSY     = 0x02U,
    HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

#define __NOP() __asm__ __volatile__ ("nop")

#define __REV16(x) __builtin_bswap16(x)
#define __REVSH(x) ((int16_t)__builtin_bswap16((uint16_t)(x)))
#define __REV(x)   __builtin_bswap32(x)


//-------------------------------------------------------
// c++ only: Arduino integration and Core 0/1 setup
//-------------------------------------------------------
#ifdef __cplusplus

#include <Arduino.h>
#include <pico/multicore.h>
#include <hardware/watchdog.h>
#include <hardware/clocks.h>

// undefine MIN/MAX from Pico SDK to prevent redefinition
#undef MIN
#undef MAX

#undef IRQHANDLER
#define IRQHANDLER(__Declaration__)  extern "C" { __Declaration__ }

inline void __disable_irq(void) { noInterrupts(); }
inline void __enable_irq(void)  { interrupts(); }

static uint8_t restart_controller = 0;
void main_loop(void);

#ifdef DEVICE_HAS_CYW_WIFI
void wifi_loop(void);
extern "C" void init_cyw43_wifi(void);
#endif
#if defined DEVICE_HAS_DRONECAN || defined DEVICE_HAS_DRONECAN_FD
void dc_hal_poll_core1(void);
#endif

// core 0 runs main_loop() in loop(), together with all that the framework does on core 0 (USB, alarm pool),
// core 1 services CAN and the CYW43 wifi/BT, which the custom variant leaves for core 1 to bring up
static void core1_entry(void)
{
#ifdef DEVICE_HAS_CYW_WIFI
    // core 0 locks core 1 out while it erases or programs flash, the CYW43 driver reads its firmware from
    // flash. the victim init claims the fifo irq exclusively and asserts if a handler is already there,
    // this arduino-pico (RP2350 build uses doorbells) claims none, the check is for a framework that does
    irq_handler_t handler = irq_get_exclusive_handler(SIO_FIFO_IRQ_NUM(1));
    if (handler) irq_remove_handler(SIO_FIFO_IRQ_NUM(1), handler);
    multicore_lockout_victim_init();
    init_cyw43_wifi(); // its async context, lwIP and btstack then run on core 1
#endif
    while (1) {
#ifdef DEVICE_HAS_CYW_WIFI
        wifi_loop();
        delay(1); // paces wifi_loop(), one datagram per call. sleeps via core 0's alarm pool, one short irq per ms
#endif
#if defined DEVICE_HAS_DRONECAN || defined DEVICE_HAS_DRONECAN_FD
        dc_hal_poll_core1(); // as fast as possible
#endif
    }
}

// all code runs from RAM, so flash erase/program on core 0 needs core 1 locked out only when it reads flash.
// without the lockout nothing core 1 runs or reads may ever be placed in flash (.flashtext, .big_const)
// rp2040.idleOtherCore() is a no-op, arduino-pico only enables it when core 1 is started via setup1()
#ifdef DEVICE_HAS_CYW_WIFI
inline void rp_flash_lockout_start(void) { multicore_lockout_start_blocking(); }
inline void rp_flash_lockout_end(void) { multicore_lockout_end_blocking(); }
#else
inline void rp_flash_lockout_start(void) {}
inline void rp_flash_lockout_end(void) {}
#endif

void setup()
{
    // boot clock change moved clk_peri to the 48 MHz USB PLL, put it back on PLL_SYS
    clock_configure_undivided(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, clock_get_hz(clk_sys));
    multicore_launch_core1(core1_entry);
#ifdef DEVICE_HAS_CYW_WIFI
    while (!multicore_lockout_victim_is_initialized(1)) tight_loop_contents(); // main_loop() may write flash at once
#endif
}

void loop()
{
    main_loop();
}

#define INITCONTROLLER_ONCE \
    if(restart_controller <= 1){ \
    if(restart_controller == 0){
#define RESTARTCONTROLLER \
    }
#define INITCONTROLLER_END \
    restart_controller = UINT8_MAX; \
    }
#define GOTO_RESTARTCONTROLLER \
    watchdog_reboot(0, 0, 0);

#endif // __cplusplus

#endif // RP_GLUE_H

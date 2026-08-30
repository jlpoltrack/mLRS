/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32c5xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32c5xx_it.h"
#include "stm32c5xx_ll_flash.h"

/* TEMP bringup aid: report a fault on the debug uart (LPUART1, PB6) before hanging.
   Polled, so it works with the interrupts off and without the mLRS C++ debug class. */
static void fault_puts(const char* s)
{
  while (*s) {
    while (!(LPUART1->ISR & USART_ISR_TXE_TXFNF)) {}
    LPUART1->TDR = (uint8_t)*s++;
  }
}

static void fault_puthex(const char* name, uint32_t v)
{
  static const char hex[] = "0123456789ABCDEF";
  char buf[12];
  for (int i = 0; i < 8; i++) { buf[i] = hex[(v >> (28 - 4*i)) & 0xF]; }
  buf[8] = '\n'; buf[9] = 0;
  fault_puts(name);
  fault_puts(buf);
}

/* dump the stacked exception frame, so the faulting PC can be resolved with addr2line */
void fault_report(uint32_t* frame)
{
  fault_puthex("  R0   ", frame[0]);
  fault_puthex("  R1   ", frame[1]);
  fault_puthex("  R2   ", frame[2]);
  fault_puthex("  R3   ", frame[3]);
  fault_puthex("  R12  ", frame[4]);
  fault_puthex("  LR   ", frame[5]);
  fault_puthex("  PC   ", frame[6]);
  fault_puthex("  xPSR ", frame[7]);
  fault_puthex("  CFSR ", SCB->CFSR);
  fault_puthex("  HFSR ", SCB->HFSR);
  fault_puthex("  MMFAR", SCB->MMFAR);
  fault_puthex("  BFAR ", SCB->BFAR);
}

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers             */
/******************************************************************************/

/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* An uncorrectable flash ECC error raises an NMI on this family, and reading a word that
     was never programmed is exactly such an error. The emulated eeprom does read erased
     pages - ee_init() reads the two page headers before it knows anything about them - so
     this must be survivable: clear the flag and resume, the read returns all ones which is
     what the eeprom expects to see for an erased page. Only a genuinely unexpected NMI
     (there is no other source enabled here, the CSS is off) is left to hang. */
  if (LL_FLASH_IsActiveFlag_ECCD(FLASH)) /* ECCD lives in ECCDETR, not in SR */
  {
    LL_FLASH_ClearFlag_ECCD(FLASH);
    return;
  }

  fault_puts("\n!! NMI\n");
  while (1)
  {
  }
}

/**
  * @brief This function handles Hard fault interrupt.
  */
__attribute__((naked)) void HardFault_Handler(void)
{
  __asm volatile (
    "tst lr, #4          \n"
    "ite eq              \n"
    "mrseq r0, msp       \n"
    "mrsne r0, psp       \n"
    "b hardfault_report  \n"
  );
}

void hardfault_report(uint32_t* frame)
{
  fault_puts("\n!! HardFault\n");
  fault_report(frame);
  while (1)
  {
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  fault_puts("\n!! MemManage_Handler\n");
  while (1)
  {
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  fault_puts("\n!! BusFault_Handler\n");
  while (1)
  {
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  fault_puts("\n!! UsageFault_Handler\n");
  while (1)
  {
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* STM32C5xx Peripheral Interrupt Handlers                                    */
/* The peripheral handlers mLRS uses (TIM2 for the clock, EXTI2 for the SX     */
/* DIO1, and the USART/LPUART ones) are defined by the mLRS code and the       */
/* stm32ll-lib itself, so they are deliberately not repeated here.            */
/* For the available names refer to startup_stm32c552xx.c.                    */
/******************************************************************************/

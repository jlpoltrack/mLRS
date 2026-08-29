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

  while (1)
  {
  }
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32c5xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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

/**
  * Initializes the Global MSP.
  *
  * Like the H5, the C5 has no SYSCFG - the SBS replaces it, and it is the SBS that
  * carries the things mLRS cares about here.
  */
void HAL_MspInit(void)
{
  LL_APB3_GRP1_EnableClock(LL_APB3_GRP1_PERIPH_SBS);
}

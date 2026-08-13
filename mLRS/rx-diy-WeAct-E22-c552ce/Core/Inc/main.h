/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
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

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c5xx_hal.h"
#include "stm32c5xx_ll_bus.h"
#include "stm32c5xx_ll_rcc.h"
#include "stm32c5xx_ll_gpio.h"
#include "stm32c5xx_ll_system.h"
#include "stm32c5xx_ll_flash.h"
#include "stm32c5xx_ll_icache.h"
#include "stm32c5xx_ll_utils.h"

/* HAL2 renamed the status enum to hal_status_t, the enumerators kept their names.
   gdisp.h includes this header precisely to get the family HAL types, so alias it here. */
typedef hal_status_t HAL_StatusTypeDef;

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

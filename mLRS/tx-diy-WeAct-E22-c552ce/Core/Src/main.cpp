/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body
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

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
/* USER CODE BEGIN PFP */
int main_main();
/* USER CODE END PFP */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ICACHE_Init();
  /* USER CODE BEGIN 2 */
  return main_main();
  /* USER CODE END 2 */

  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  *
  * WeAct STM32C5xxCxTx core board: 8 MHz HSE crystal (Y2).
  *
  * The C5 has NO PLL. SYSCLK can only come from HSIS (144 MHz RC), HSIDIV3 (48 MHz),
  * PSIS or HSE directly - and with no PLL the 8 MHz crystal on its own would give an
  * 8 MHz SYSCLK. The way to get both full speed and crystal accuracy is the PSI in
  * PLL mode: PSI locks to the HSE as its reference and outputs 144 MHz (RM0522 9.4.3).
  * That is the C5 equivalent of the "HSE + PLL" other mLRS targets use.
  *
  * HCLK = PCLK1 = PCLK2 = PCLK3 = 144 MHz, so every peripheral kernel clock left on
  * its APB default (SPI1 on PCLK2, USART1 on PCLK2, USART2 on PCLK1, LPUART1 on
  * PCLK3) also runs at 144 MHz, which is what the stm32ll-lib C5 baudrate and SPI
  * prescaler tables assume.
  *
  * At 144 MHz the flash needs 4 wait states and WRHIGHFREQ = 2 (RM0522 Table 20).
  * Both must be programmed BEFORE switching SYSCLK up.
  */
void SystemClock_Config(void)
{
  /* the flash runs from HSIDIV3 at 48 MHz out of reset, so raise the wait states first */
  LL_FLASH_SetLatency(FLASH, LL_FLASH_LATENCY_4WS);
  LL_FLASH_SetProgrammingDelay(FLASH, LL_FLASH_PROGRAM_DELAY_2);
  while (LL_FLASH_GetLatency(FLASH) != LL_FLASH_LATENCY_4WS) {}

  /* start the 8 MHz crystal, it is the PSI reference */
  LL_RCC_HSE_Enable();
  while (LL_RCC_HSE_IsReady() != 1U) {}

  /* PSI in PLL mode: reference = HSE 8 MHz, output = 144 MHz */
  LL_RCC_SetPSIClkSource(LL_RCC_PSISOURCE_HSE);
  LL_RCC_SetPSIRef(LL_RCC_PSIREF_8MHZ);
  LL_RCC_SetPSIFreqOutput(LL_RCC_PSIFREQ_144MHZ);

  LL_RCC_PSIS_Enable();
  while (LL_RCC_PSIS_IsReady() != 1U) {}

  /* no bus dividers, everything runs at 144 MHz */
  LL_RCC_SetAHBPrescaler(LL_RCC_HCLK_PRESCALER_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_PRESCALER_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_PRESCALER_1);
  LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_PRESCALER_1);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PSIS);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PSIS) {}

  // HAL_Init() sized the SysTick reload for the 48 MHz boot clock, so it has to be redone
  // here or uwTick, and with it the whole mLRS SysTask, runs 3x too fast
  HAL_UpdateCoreClock();
}

/**
  * @brief ICACHE Initialization Function
  * @retval None
  *
  * NOTE: the flash and the OTP/UID area are cacheable, so anything that writes the flash
  * or reads the UID has to bracket itself with the cache off. stdstm32-eeprom.h does that
  * in ee_hal_unlock()/ee_hal_lock(), and stdstm32-mcu.h does it in mcu_uid().
  */
static void MX_ICACHE_Init(void)
{
  LL_ICACHE_Enable(ICACHE);
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* the mLRS hal does the actual pin setup, here we only bring up the port clocks.
     port E is needed for the user LED on PE2. */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif

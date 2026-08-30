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

/* Private define ------------------------------------------------------------*/
/* CRS, see MX_CRS_Init(). ratio = 48 MHz error counter clock / 976.5625 Hz SYNC.
   Trim step is DS14928 Table 36: typ 0.1%, max 0.15%. It sets the trim dead band, so
   the residual HSI144 error is about +-TRIM_STEP_PPM/2, ie +-500 ppm typ. */
#define CRS_SYNC_RATIO     49152U
#define CRS_TRIM_STEP_PPM  1000U
#define CRS_RELOAD_VALUE   (CRS_SYNC_RATIO - 1U)
/* FELIM = ratio * step / 2, RM0522 10.4.6 requires rounding up */
#define CRS_FELIM_VALUE    (((CRS_SYNC_RATIO * CRS_TRIM_STEP_PPM) + 1999999U) / 2000000U)

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_CRS_Init(void);
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
  * WeAct STM32C5xxCxTx core board: 8 MHz HSE crystal (Y2). 144 MHz everywhere,
  * HCLK = PCLK1 = PCLK2 = PCLK3, as the stm32ll-lib C5 tables assume.
  *
  * SYSCLK is HSIS (144 MHz RC) CRS-disciplined against the crystal, not PSI: the Rx
  * needs PSI at 160 MHz for an 80 MHz FDCAN clock, which puts PSIS over the 144 MHz
  * SYSCLK limit. can_init() in stdstm32-can.h owns PSI, we leave it off here.
  *
  * 144 MHz needs 4 wait states and WRHIGHFREQ = 2, programmed BEFORE switching up.
  */
void SystemClock_Config(void)
{
  /* the flash runs from HSIDIV3 at 48 MHz out of reset, so raise the wait states first */
  LL_FLASH_SetLatency(FLASH, LL_FLASH_LATENCY_4WS);
  LL_FLASH_SetProgrammingDelay(FLASH, LL_FLASH_PROGRAM_DELAY_2);
  while (LL_FLASH_GetLatency(FLASH) != LL_FLASH_LATENCY_4WS) {}

  /* the 8 MHz crystal is the CRS reference here, and the PSI reference in can_init() */
  LL_RCC_HSE_Enable();
  while (LL_RCC_HSE_IsReady() != 1U) {}

  /* HSIS is OFF out of reset, RCC_CR1 resets to 0x22 = HSIDIV3 only */
  LL_RCC_HSIS_Enable();
  while (LL_RCC_HSIS_IsReady() != 1U) {}

  /* HSIDIV3 clocks the CRS error counter, so keep it on once sysclk moves off it */
  LL_RCC_HSIDIV3_Enable();
  while (LL_RCC_HSIDIV3_IsReady() != 1U) {}

  /* no bus dividers, everything runs at 144 MHz */
  LL_RCC_SetAHBPrescaler(LL_RCC_HCLK_PRESCALER_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_PRESCALER_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_PRESCALER_1);
  LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_PRESCALER_1);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSIS);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSIS) {}

  MX_CRS_Init();

  // HAL_Init() sized the SysTick reload for the 48 MHz boot clock, so it has to be redone
  // here or uwTick, and with it the whole mLRS SysTask, runs 3x too fast
  HAL_UpdateCoreClock();
}

/**
  * @brief CRS Initialization Function
  * @retval None
  *
  * Trims HSI144 against the HSE crystal, so SYSCLK gets crystal accuracy without PSI.
  * CRS acts on HSI144 itself (RM0522 10.1), so HSIS, HSIDIV3 and HSIK all follow.
  *
  * HSE 8 MHz -> RTCPRE /64 -> 125 kHz -> SYNCDIV /128 -> 976.5625 Hz SYNC.
  * The error counter runs on HSIDIV3 48 MHz, not on HSI144, hence CRS_RELOAD_VALUE.
  *
  * FELIM is the trim dead band, so it fixes the residual accuracy: the loop stops
  * correcting below it. Mean frequency tracks the crystal, residual is +-500 ppm typ,
  * against +-1.5% for an untrimmed HSI over temperature (DS14928 Table 36).
  */
static void MX_CRS_Init(void)
{
  /* RTCPRE output must stay below 1 MHz, and slower buys counter resolution */
  LL_RCC_SetRTC_HSEPrescaler(64);

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CRS);

  LL_CRS_ConfigSynchronization(CRS,
                               LL_CRS_SYNC_SOURCE_HSE_1MHZ | LL_CRS_SYNC_DIV_128 |
                                 LL_CRS_SYNC_POLARITY_RISING,
                               CRS_RELOAD_VALUE,
                               CRS_FELIM_VALUE);

  /* AUTOTRIMEN makes TRIM read-only and hardware controlled, so it goes on before CEN */
  LL_CRS_EnableAutoTrimming(CRS);
  LL_CRS_EnableFreqErrorCounter(CRS);
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

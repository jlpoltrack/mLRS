/**
  ******************************************************************************
  * @file           : Target/usbd_conf.c
  * @version        : v2.0_Cube
  * @brief          : This file implements the board support package for the USB device library
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifdef STDSTM32_USE_USB

#include "usbd_def.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_conf.h"


PCD_HandleTypeDef hpcd_USB_FS;
void Error_Handler(void);


static USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status);
// we do not support low power static void SystemClockConfig_Resume(void);
// we do not support low power extern void SystemClock_Config(void);


#if defined STM32C5
//-------------------------------------------------------
// HAL2 shims
//-------------------------------------------------------
// C5 is the only family here which ships ST's HAL2 package. It offers the same set of PCD
// operations, but under different names, and it dropped three things this glue relies on:
// the MSP hooks, the pData back pointer in the handle, and the per endpoint is_stall flag.
// All of that is bridged here, so the USBD_LL_xxx functions further below stay common.

#include "stm32c5xx_ll_bus.h"
#include "stm32c5xx_ll_rcc.h"

static USBD_HandleTypeDef* usbd_pdev; // there is only ever one instance
static uint16_t usbd_ep_is_stall[2];  // [0] = OUT, [1] = IN, bit n = endpoint n

#define USBD_PDEV(_hpcd)  usbd_pdev

static HAL_StatusTypeDef HAL_PCD_EP_Open(PCD_HandleTypeDef* hpcd, uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type)
{
    return HAL_PCD_OpenEndpoint(hpcd, ep_addr, ep_mps, (hal_pcd_ep_type_t)ep_type);
}

static HAL_StatusTypeDef HAL_PCD_EP_Close(PCD_HandleTypeDef* hpcd, uint8_t ep_addr)
{
    return HAL_PCD_CloseEndpoint(hpcd, ep_addr);
}

static HAL_StatusTypeDef HAL_PCD_EP_Flush(PCD_HandleTypeDef* hpcd, uint8_t ep_addr)
{
    return HAL_PCD_FlushEndpoint(hpcd, ep_addr);
}

static HAL_StatusTypeDef HAL_PCD_EP_SetStall(PCD_HandleTypeDef* hpcd, uint8_t ep_addr)
{
    usbd_ep_is_stall[(ep_addr & 0x80) ? 1 : 0] |= (uint16_t)(1 << (ep_addr & 0x0F));
    return HAL_PCD_SetEndpointStall(hpcd, ep_addr);
}

static HAL_StatusTypeDef HAL_PCD_EP_ClrStall(PCD_HandleTypeDef* hpcd, uint8_t ep_addr)
{
    usbd_ep_is_stall[(ep_addr & 0x80) ? 1 : 0] &= (uint16_t)~(1 << (ep_addr & 0x0F));
    return HAL_PCD_ClearEndpointStall(hpcd, ep_addr);
}

static HAL_StatusTypeDef HAL_PCD_SetAddress(PCD_HandleTypeDef* hpcd, uint8_t dev_addr)
{
    return HAL_PCD_SetDeviceAddress(hpcd, dev_addr);
}

static HAL_StatusTypeDef HAL_PCD_EP_Transmit(PCD_HandleTypeDef* hpcd, uint8_t ep_addr, uint8_t* pbuf, uint32_t size)
{
    return HAL_PCD_SetEndpointTransmit(hpcd, ep_addr, pbuf, size);
}

static HAL_StatusTypeDef HAL_PCD_EP_Receive(PCD_HandleTypeDef* hpcd, uint8_t ep_addr, uint8_t* pbuf, uint32_t size)
{
    return HAL_PCD_SetEndpointReceive(hpcd, ep_addr, pbuf, size);
}


// HAL2 has no HAL_PCD_MspInit(), so the clock, pin and nvic setup that lives there on the
// other families is done here, called from USBD_LL_Init() before HAL_PCD_Init().
static void usbd_hw_init(void)
{
    // CK48, the USB kernel clock, comes from PSI_DIV3. There is no HSI48 on C5, but
    // SystemClock_Config() runs the PSI at 144 MHz locked to the 8 MHz HSE, so PSI/3 is an
    // exact and crystal accurate 48 MHz, which is better than an RC oscillator would be.
    LL_RCC_PSIDIV3_Enable();
    while (!LL_RCC_PSIDIV3_IsReady()) {}
    LL_RCC_SetCK48ClockSource(LL_RCC_CK48_CLKSOURCE_PSIDIV3);

    // nothing to do for PA11/PA12: the datasheet pin table lists USB_DM/USB_DP as additional
    // functions, not alternate ones, so the peripheral drives the pads directly and they stay
    // in their reset state. Same as on G4, and unlike H5, where USB is on AF10.

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USB);

    NVIC_SetPriority(USBD_IRQn, USBD_IRQ_PRIORITY);
    NVIC_EnableIRQ(USBD_IRQn);
}

#else

#define USBD_PDEV(_hpcd)  ((USBD_HandleTypeDef*)(_hpcd)->pData)

#endif // STM32C5



/*******************************************************************************
                       LL Driver Callbacks (PCD -> USB Device Library)
*******************************************************************************/

#if !defined STM32C5 // HAL2 has no MSP hooks, see usbd_hw_init() above
void HAL_PCD_MspInit(PCD_HandleTypeDef* pcdHandle)
{
  if(pcdHandle->Instance==USBD_INST)
  {
#if defined STM32G431xx || defined STM32G441xx || defined STM32G491xx || defined STM32G474xx
    // initialize HSI48, copied with adaption from SystemClock_Config()
    RCC_OscInitTypeDef RCC_OscInitStruct = {};
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE; // important, otherwise HAL_RCC_OscConfig() will modify the PLL setting
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        //Error_Handler();
    }
#endif

    // copied from SystemClock_Config()
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {};
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
#if defined STM32F103xE
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
#elif defined STM32G431xx || defined STM32G441xx || defined STM32G491xx || defined STM32G474xx
    // CubeMX is not adding this to SystemClock_Config(), but it is needed
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
#elif defined STM32F072xB
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
#endif
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        //Error_Handler();
    }

    __HAL_RCC_USB_CLK_ENABLE();

    //OW HAL_NVIC_SetPriority(USBD_IRQn, 0, 0);
    NVIC_SetPriority(USBD_IRQn, USBD_IRQ_PRIORITY);
    HAL_NVIC_EnableIRQ(USBD_IRQn);
  }
}


void HAL_PCD_MspDeInit(PCD_HandleTypeDef* pcdHandle)
{
  if(pcdHandle->Instance==USBD_INST)
  {
    /* Peripheral clock disable */
    __HAL_RCC_USB_CLK_DISABLE();

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(USBD_IRQn);
  }
}
#endif // !STM32C5


void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
#if defined STM32C5
  // the HAL2 pcd reads the setup packet into hpcd->setup and never assigns hpcd->p_setup,
  // which stays NULL and hard faults USBD_ParseSetupRequest on the first SETUP
  USBD_LL_SetupStage(USBD_PDEV(hpcd), (uint8_t *)hpcd->setup);
#else
  USBD_LL_SetupStage(USBD_PDEV(hpcd), (uint8_t *)hpcd->Setup);
#endif
}


void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
#if defined STM32C5
  USBD_LL_DataOutStage(USBD_PDEV(hpcd), epnum, hpcd->out_ep[epnum].p_xfer_buffer);
#else
  USBD_LL_DataOutStage(USBD_PDEV(hpcd), epnum, hpcd->OUT_ep[epnum].xfer_buff);
#endif
}


void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
#if defined STM32C5
  USBD_LL_DataInStage(USBD_PDEV(hpcd), epnum, hpcd->in_ep[epnum].p_xfer_buffer);
#else
  USBD_LL_DataInStage(USBD_PDEV(hpcd), epnum, hpcd->IN_ep[epnum].xfer_buff);
#endif
}


void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SOF(USBD_PDEV(hpcd));
}


void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_SpeedTypeDef speed = USBD_SPEED_FULL;

#if defined STM32C5 // HAL2 has no Init struct, and the DRD FS core is full speed only
  if (HAL_PCD_GetDeviceSpeed(hpcd) != HAL_PCD_DEVICE_SPEED_FS)
  {
    Error_Handler();
  }
#else
  if ( hpcd->Init.speed == PCD_SPEED_FULL)
  {
    speed = USBD_SPEED_FULL;
  }
  else
  {
    Error_Handler();
  }
#endif
    /* Set Speed. */
  USBD_LL_SetSpeed(USBD_PDEV(hpcd), speed);

  /* Reset Device. */
  USBD_LL_Reset(USBD_PDEV(hpcd));
}


void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
  /* Inform USB library that core enters in suspend Mode. */
  USBD_LL_Suspend(USBD_PDEV(hpcd));
  /* Enter in STOP mode. */
#if !defined STM32C5 // low power is never enabled, and HAL2 has no Init struct
  if (hpcd->Init.low_power_enable)
  {
    /* Set SLEEPDEEP bit and SleepOnExit of Cortex System Control Register. */
    SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
  }
#endif
}


void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
#if !defined STM32C5 // as above
  if (hpcd->Init.low_power_enable)
  {
    /* Reset SLEEPDEEP bit of Cortex System Control Register. */
// we do not support low power     SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
// we do not support low power     SystemClockConfig_Resume();
    while (1) {}
  }
#endif
  USBD_LL_Resume(USBD_PDEV(hpcd));
}


void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_IsoOUTIncomplete(USBD_PDEV(hpcd), epnum);
}


void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_IsoINIncomplete(USBD_PDEV(hpcd), epnum);
}


void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevConnected(USBD_PDEV(hpcd));
}


void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevDisconnected(USBD_PDEV(hpcd));
}


/*******************************************************************************
                       LL Driver Interface (USB Device Library --> PCD)
*******************************************************************************/

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
  /* Init USB Ip. */
  /* Link the driver to the stack. */
#if defined STM32C5
  usbd_pdev = pdev; // HAL2 dropped the pData back pointer, see the shims above
  pdev->pData = &hpcd_USB_FS;

  usbd_hw_init(); // does what HAL_PCD_MspInit() does on the other families

  if (HAL_PCD_Init(&hpcd_USB_FS, USBD_INST) != HAL_OK)
  {
    Error_Handler( );
  }

  hal_pcd_config_t pcd_config = {};
  pcd_config.dma_enable = HAL_PCD_DMA_DISABLED;
  pcd_config.pcd_speed = HAL_PCD_SPEED_FS;
  pcd_config.phy_interface = HAL_PCD_PHY_EMBEDDED_FS;
  pcd_config.sof_enable = HAL_PCD_SOF_DISABLED;
  pcd_config.lpm_enable = HAL_PCD_LPM_DISABLED;
  pcd_config.battery_charging_enable = HAL_PCD_BCD_DISABLED;
  pcd_config.vbus_sensing_enable = HAL_PCD_VBUS_SENSE_DISABLED;
  pcd_config.bulk_doublebuffer_enable = HAL_PCD_BULK_DB_DISABLED;
  if (HAL_PCD_SetConfig(&hpcd_USB_FS, &pcd_config) != HAL_OK)
  {
    Error_Handler( );
  }

  // same PMA layout as everywhere else, HAL2 just renamed the call and the buffer kind
  HAL_PCD_PMAConfig(&hpcd_USB_FS, 0x00, HAL_PCD_SNG_BUF, 0x18);
  HAL_PCD_PMAConfig(&hpcd_USB_FS, 0x80, HAL_PCD_SNG_BUF, 0x58);
  HAL_PCD_PMAConfig(&hpcd_USB_FS, 0x81, HAL_PCD_SNG_BUF, 0xC0);
  HAL_PCD_PMAConfig(&hpcd_USB_FS, 0x01, HAL_PCD_SNG_BUF, 0x110);
  HAL_PCD_PMAConfig(&hpcd_USB_FS, 0x82, HAL_PCD_SNG_BUF, 0x100);
#else
  hpcd_USB_FS.pData = pdev;
  pdev->pData = &hpcd_USB_FS;

  hpcd_USB_FS.Instance = USBD_INST;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler( );
  }

  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x00 , PCD_SNG_BUF, 0x18);
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x80 , PCD_SNG_BUF, 0x58);
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x81 , PCD_SNG_BUF, 0xC0);
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x01 , PCD_SNG_BUF, 0x110);
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x82 , PCD_SNG_BUF, 0x100);
#endif

  return USBD_OK;
}


USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

#if defined STM32C5 // HAL2 returns void here
  HAL_PCD_DeInit(pdev->pData);
#else
  hal_status = HAL_PCD_DeInit(pdev->pData);
#endif

  usb_status =  USBD_Get_USB_Status(hal_status);

  return usb_status;
}


USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_Start(pdev->pData);

  usb_status =  USBD_Get_USB_Status(hal_status);

  return usb_status;
}


USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_Stop(pdev->pData);

  usb_status =  USBD_Get_USB_Status(hal_status);

  return usb_status;
}


USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type);

  usb_status =  USBD_Get_USB_Status(hal_status);

  return usb_status;
}


USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_EP_Close(pdev->pData, ep_addr);

  usb_status =  USBD_Get_USB_Status(hal_status);

  return usb_status;
}


USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_EP_Flush(pdev->pData, ep_addr);

  usb_status =  USBD_Get_USB_Status(hal_status);

  return usb_status;
}


USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_EP_SetStall(pdev->pData, ep_addr);

  usb_status =  USBD_Get_USB_Status(hal_status);

  return usb_status;
}


USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);

  usb_status =  USBD_Get_USB_Status(hal_status);

  return usb_status;
}


uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
#if defined STM32C5
  // HAL2 does not keep an is_stall flag per endpoint, so the shims track it themselves
  (void)pdev;
  return (usbd_ep_is_stall[(ep_addr & 0x80) ? 1 : 0] & (1 << (ep_addr & 0x0F))) ? 1 : 0;
#else
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef*) pdev->pData;

  if((ep_addr & 0x80) == 0x80)
  {
    return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
  }
  else
  {
    return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
  }
#endif
}


USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_SetAddress(pdev->pData, dev_addr);

  usb_status =  USBD_Get_USB_Status(hal_status);

  return usb_status;
}


USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);

  usb_status =  USBD_Get_USB_Status(hal_status);

  return usb_status;
}


USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);

  usb_status =  USBD_Get_USB_Status(hal_status);

  return usb_status;
}


uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef*) pdev->pData, ep_addr);
}


void USBD_LL_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}


void *USBD_static_malloc(uint32_t size)
{
  static uint32_t mem[(sizeof(USBD_CDC_HandleTypeDef)/4)+1];/* On 32-bit boundary */
  return mem;
}


void USBD_static_free(void *p)
{

}


// we do not support low power static void SystemClockConfig_Resume(void)
// we do not support low power {
// we do not support low power   SystemClock_Config();
// we do not support low power }


USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status)
{
  USBD_StatusTypeDef usb_status = USBD_OK;

  switch (hal_status)
  {
    case HAL_OK :
      usb_status = USBD_OK;
    break;
    case HAL_ERROR :
      usb_status = USBD_FAIL;
    break;
    case HAL_BUSY :
      usb_status = USBD_BUSY;
    break;
    case HAL_TIMEOUT :
      usb_status = USBD_FAIL;
    break;
    default :
      usb_status = USBD_FAIL;
    break;
  }
  return usb_status;
}


#endif // #ifdef STDSTM32_USE_USB

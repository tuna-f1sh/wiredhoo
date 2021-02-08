/**
  ******************************************************************************
  * @file    usbd_conf_template.c
  * @author  MCD Application Team
  * @brief   USB Device configuration and interface file
  *          This template should be copied to the user folder,
  *          renamed and customized following user needs.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_core.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
/* Private function prototypes -----------------------------------------------*/
/*
   typedef enum              typedef enum
   {                         {
   HAL_OK       = 0x00,      USBD_OK   = 0U,
   HAL_ERROR    = 0x01,      USBD_BUSY,
   HAL_BUSY     = 0x02,      USBD_FAIL
   HAL_TIMEOUT  = 0x03
   } HAL_StatusTypeDef;      } USBD_StatusTypeDef;
*/
static USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status)
{
  if (hal_status == HAL_OK)
    return USBD_OK;
  else if (hal_status == HAL_BUSY)
    return USBD_BUSY;
  else
    return USBD_FAIL;
}
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
  hpcd_USB_OTG_FS.pData = pdev;
  pdev->pData = &hpcd_USB_OTG_FS;
  HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 1024 / 4);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 1024 / 4);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 1024 / 4);
  return USBD_OK;
}

/**
  * @brief  De-Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
  HAL_StatusTypeDef const hal_status = HAL_PCD_DeInit(pdev->pData);
  return USBD_Get_USB_Status(hal_status);

}

/**
  * @brief  Starts the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
  HAL_StatusTypeDef const hal_status = HAL_PCD_Start(pdev->pData);
  return USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Stops the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
  HAL_StatusTypeDef const hal_status = HAL_PCD_Stop(pdev->pData);
  return USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Opens an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  ep_type: Endpoint Type
  * @param  ep_mps: Endpoint Max Packet Size
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                  uint8_t ep_type, uint16_t ep_mps)
{
  HAL_StatusTypeDef const hal_status = HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type);
  pdev->ep_in[ep_addr & 0x7F].is_used = 1;
  return USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Closes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_StatusTypeDef const hal_status = HAL_PCD_EP_Close(pdev->pData, ep_addr);
  pdev->ep_in[ep_addr & 0x7F].is_used = 0;
  return USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Flushes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_StatusTypeDef const hal_status = HAL_PCD_EP_Flush(pdev->pData, ep_addr);
  return USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Sets a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_StatusTypeDef const hal_status = HAL_PCD_EP_SetStall(pdev->pData, ep_addr);
  return USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Clears a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev,
                                        uint8_t ep_addr)
{
  HAL_StatusTypeDef const hal_status = HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);
  return USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Returns Stall condition.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Stall (1: Yes, 0: No)
  */
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  PCD_HandleTypeDef const *hpcd = pdev->pData;
  return ep_addr & 0x80 ? hpcd->IN_ep[ep_addr & 0x7F].is_stall : hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
}

/**
  * @brief  Assigns a USB address to the device.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev,
                                         uint8_t dev_addr)
{
  HAL_StatusTypeDef const hal_status = HAL_PCD_SetAddress(pdev->pData, dev_addr);
  return USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Transmits data over an endpoint.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be sent
  * @param  size: Data size
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                    uint8_t *pbuf, uint32_t size)
{
  HAL_StatusTypeDef const hal_status = HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
  return USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Prepares an endpoint for reception.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be received
  * @param  size: Data size
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                          uint8_t *pbuf, uint32_t size)
{
  HAL_StatusTypeDef const hal_status = HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
  return USBD_Get_USB_Status(hal_status);
}

/**
  * @brief  Returns the last transferred packet size.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Recived Data Size
  */
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef*)pdev->pData, ep_addr);
}

/**
  * @brief  Delays routine for the USB Device Library.
  * @param  Delay: Delay in ms
  * @retval None
  */
void USBD_LL_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

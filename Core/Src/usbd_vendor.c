/**
  ******************************************************************************
  * @file    usbd_template.c
  * @author  MCD Application Team
  * @brief   This file provides the HID core functions.
  *
  * @verbatim
  *
  *          ===================================================================
  *                                TEMPLATE Class  Description
  *          ===================================================================
  *
  *
  *
  *
  *
  *
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *
  *
  *  @endverbatim
  *
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
#include <stdbool.h>
#include "cmsis_os.h"
#include "message_buffer.h"
#include "usbd_vendor.h"
#include "usbd_ctlreq.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_TEMPLATE
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_TEMPLATE_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_TEMPLATE_Private_Defines
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_TEMPLATE_Private_Macros
  * @{
  */

/**
  * @}
  */




/** @defgroup USBD_TEMPLATE_Private_FunctionPrototypes
  * @{
  */


static uint8_t USBD_TEMPLATE_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_TEMPLATE_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_TEMPLATE_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_TEMPLATE_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_TEMPLATE_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t *USBD_TEMPLATE_GetCfgDesc(uint16_t *length);
static uint8_t *USBD_TEMPLATE_GetDeviceQualifierDesc(uint16_t *length);
/**
  * @}
  */

/** @defgroup USBD_TEMPLATE_Private_Variables
  * @{
  */
volatile bool data_in_busy = false;
uint8_t data_out_buffer[1024] = {};
extern MessageBufferHandle_t xMessageBuffer;

USBD_ClassTypeDef USBD_TEMPLATE_ClassDriver =
{
  USBD_TEMPLATE_Init,
  USBD_TEMPLATE_DeInit,
  /* Control Endpoints */
  USBD_TEMPLATE_Setup,
  NULL,
  NULL,
  /* Class Specific */
  USBD_TEMPLATE_DataIn,
  USBD_TEMPLATE_DataOut,
  NULL,
  NULL,
  NULL,

  USBD_TEMPLATE_GetCfgDesc,
  USBD_TEMPLATE_GetCfgDesc,
  USBD_TEMPLATE_GetCfgDesc,
  USBD_TEMPLATE_GetDeviceQualifierDesc,
};

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif
/* USB TEMPLATE device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_TEMPLATE_CfgDesc[USB_TEMPLATE_CONFIG_DESC_SIZ] __ALIGN_END =
{
  0x09, /* bLength: Configuation Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  USB_TEMPLATE_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x01,         /*bNumInterfaces: 1 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x02,         /*iConfiguration: Index of string descriptor describing the configuration*/
  0xC0,         /*bmAttributes: bus powered and Supports Remote Wakeup */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/

    /* Interface */
  0x09,                     /* bLength */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
  0x00,                     /* bInterfaceNumber */
  0x00,                     /* bAlternateSetting */
  0x02,                     /* bNumEndpoints */
  0xFF,                     /* bInterfaceClass: Vendor Specific Class */
  0x00,                     /* bInterfaceSubClass */
  0x00,                     /* bInterfaceProtocol */
  0x02,                     /* iInterface */

  /* Endpoint OUT */
  0x07,                            /* bLength */
  USB_DESC_TYPE_ENDPOINT,          /* bDescriptorType */
  TEMPLATE_EPOUT_ADDR,             /* bEndpointAddress */
  0x02,                            /* bmAttributes */
  LOBYTE(USB_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize */
  HIBYTE(USB_FS_MAX_PACKET_SIZE),
  0x01,                            /* bInterval */

  /* Endpoint IN */
  0x07,                             /* bLength */
  USB_DESC_TYPE_ENDPOINT,           /* bDescriptorType */
  TEMPLATE_EPIN_ADDR,               /* bEndpointAddress */
  0x02,                             /* bmAttributes */
  LOBYTE(USB_FS_MAX_PACKET_SIZE),   /* wMaxPacketSize */
  HIBYTE(USB_FS_MAX_PACKET_SIZE),
  0x01                              /* bInterval */
};

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif
/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_TEMPLATE_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/**
  * @}
  */

/** @defgroup USBD_TEMPLATE_Private_Functions
  * @{
  */

/**
  * @brief  USBD_TEMPLATE_Init
  *         Initialize the TEMPLATE interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_TEMPLATE_Init(USBD_HandleTypeDef *pdev,
                                   uint8_t cfgidx)
{
  USBD_LL_OpenEP(pdev, TEMPLATE_EPIN_ADDR, USBD_EP_TYPE_BULK, USB_FS_MAX_PACKET_SIZE);
  USBD_LL_OpenEP(pdev, TEMPLATE_EPOUT_ADDR, USBD_EP_TYPE_BULK, USB_FS_MAX_PACKET_SIZE);
  USBD_LL_PrepareReceive(pdev, TEMPLATE_EPOUT_ADDR, data_out_buffer, USB_FS_MAX_PACKET_SIZE);
  return USBD_OK;
}

/**
  * @brief  USBD_TEMPLATE_Init
  *         DeInitialize the TEMPLATE layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_TEMPLATE_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_HID_Setup
  *         Handle the HID specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_TEMPLATE_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_TEMPLATE_GetCfgDesc
  *         return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_TEMPLATE_GetCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_TEMPLATE_CfgDesc);
  return USBD_TEMPLATE_CfgDesc;
}

/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t *USBD_TEMPLATE_DeviceQualifierDescriptor(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_TEMPLATE_DeviceQualifierDesc);
  return USBD_TEMPLATE_DeviceQualifierDesc;
}


/**
  * @brief  USBD_TEMPLATE_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_TEMPLATE_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  if (pdev->ep_in[epnum].total_length &&
      !(pdev->ep_in[epnum].total_length % USB_FS_MAX_PACKET_SIZE))
  {
    pdev->ep_in[epnum].total_length = 0;
    USBD_LL_Transmit(pdev, epnum, NULL, 0);
  }
  else
    data_in_busy = false;
  return USBD_OK;
}

static uint8_t  USBD_TEMPLATE_DataOut(USBD_HandleTypeDef *pdev,
                                      uint8_t epnum)
{
  size_t const bytes_received = USBD_LL_GetRxDataSize(pdev, epnum);
  xMessageBufferSendFromISR(xMessageBuffer, data_out_buffer, bytes_received, NULL);
  USBD_LL_PrepareReceive(pdev, TEMPLATE_EPOUT_ADDR, data_out_buffer, USB_FS_MAX_PACKET_SIZE);
  return USBD_OK;
}

/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t *USBD_TEMPLATE_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_TEMPLATE_DeviceQualifierDesc);

  return USBD_TEMPLATE_DeviceQualifierDesc;
}

/**
  * @}
  */


uint8_t USBD_TEMPLATE_Transmit(USBD_HandleTypeDef *pdev, uint8_t* buf, uint16_t length)
{
  if (data_in_busy)
    return USBD_BUSY;
  data_in_busy = true;
  pdev->ep_in[TEMPLATE_EPIN_ADDR & 0x7F].total_length = length;
  return USBD_LL_Transmit(pdev, TEMPLATE_EPIN_ADDR, buf, length);
}

/**
  * @}
  */


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

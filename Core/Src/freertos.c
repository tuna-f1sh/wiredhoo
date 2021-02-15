/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "message_buffer.h"
#include "usart.h"
#include "usbd_vendor.h"
#include "utilities.h"
#include "ant.h"
#include "antmessage.h"
#include "antdefines.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// message buffer holds USBD out (recieved from host)
MessageBufferHandle_t xAntMsgBuffer = NULL;
static uint8_t xAntMsgStorageBuffer[ANT_EP_OUT_BUFFER_SIZE];
static StaticMessageBuffer_t xAntMsgBufferStruct;
// message buffer holds simulated Rx wirelss data from ANT devices for sending to host
MessageBufferHandle_t xAntDeviceRxBuffer = NULL;
static uint8_t xAntDeviceRxStorageBuffer[ANT_EP_OUT_BUFFER_SIZE];
static StaticMessageBuffer_t xAntDeviceRxBufferStruct;

static char printBuffer[64];

osThreadId_t antProtocolTask;
const osThreadAttr_t antProtocolTask_attributes = {
  .name = "antProtocolTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 768
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

extern USBD_HandleTypeDef hUsbDeviceFS;
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void AntProtocolTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  xAntMsgBuffer = xMessageBufferCreateStatic(sizeof(xAntMsgStorageBuffer), xAntMsgStorageBuffer, &xAntMsgBufferStruct);
  xAntDeviceRxBuffer = xMessageBufferCreateStatic(sizeof(xAntDeviceRxStorageBuffer), xAntDeviceRxStorageBuffer, &xAntDeviceRxBufferStruct);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  antProtocolTask = osThreadNew(AntProtocolTask, NULL, &antProtocolTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  uint8_t tdata[8] = {0};
  uint8_t tmsg[ANT_MAX_SIZE] = {0};

  /* USER CODE BEGIN StartDefaultTask */
  printf("Default Task Starting\r\n");
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(BLINK_LED_PORT, BLINK_LED);
    // this disables isr so only for debug
    if (configUSE_STATS_FORMATTING_FUNCTIONS) {
      printf("Task List\tState\tP\tStack\tNum\r\n");
      vTaskList(printBuffer);
      printf(printBuffer);
    }
    ant_construct_data_message(MESG_BROADCAST_DATA_ID, MESG_EXTENDED_SIZE, ANT_FAKE_CH_POWER, DEVICE_ID, ANT_DEVTYPE_POWER, ANT_TXTYPE_POWER, tmsg, tdata);
    xMessageBufferSend(xAntDeviceRxBuffer, tmsg, ANT_MESSAGE_SIZE(tmsg), 0);
    /* osDelay(1000); */
    osDelay(PERIOD_TO_MS(ANT_PERIOD_POWER));
  }
  /* USER CODE END StartDefaultTask */
}

void transmit_message(uint8_t *pBuffer, size_t len, uint8_t block_tick) {
  uint8_t status = USBD_BUSY;
  uint8_t block_ticks = 0;

  while (status != USBD_OK && block_ticks < block_tick) {
    status = USBD_TEMPLATE_Transmit(&hUsbDeviceFS, pBuffer, len);
    block_ticks += 2;
    osDelay(2);
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void AntProtocolTask(void *argument) {
  printf("ANT+ Protocol Task Starting\r\n");

  /* Infinite loop */
  for(;;)
  {
    uint8_t reply[ANT_MAX_SIZE] = {0};
    uint8_t ucRxData[64];
    size_t xReceivedBytes;
    const TickType_t xBlockTime = pdMS_TO_TICKS(10);

    /* Receive the next message from the message buffer.  Wait in the Blocked
       state (so not using any CPU processing time) for a maximum of 100ms for
       a message to become available. */
    xReceivedBytes = xMessageBufferReceive(xAntMsgBuffer, (void *) ucRxData, sizeof(ucRxData), xBlockTime);
    // clear after block allows blink on rx
    HAL_GPIO_WritePin(USBD_RX_LED_PORT, USBD_RX_LED, 1);

    if(xReceivedBytes > 0) {
      if (DEBUG) {
        printf("ANT+ Rx: ");
        print_hex((char*) ucRxData, xReceivedBytes);
      }
      HAL_GPIO_WritePin(USBD_RX_LED_PORT, USBD_RX_LED, 0);


      if (process_ant_message(ucRxData, xReceivedBytes, reply) != MESG_UNKNOWN_ID) {
        if (DEBUG) {
          if (reply[2] == 0xAE) printf("ANT+ Rx Error, Tx: ");
          else printf("ANT+ Tx: ");
          print_hex((char*) reply, ANT_MESSAGE_SIZE(reply));
        }

        // TODO replace with message stream for replies
        transmit_message(reply, ANT_MESSAGE_SIZE(reply), 20);
        // close channel request follow with channel event closed
        if (reply[4] == MESG_CLOSE_CHANNEL_ID && (ucRxData[BUFFER_INDEX_MESG_ID + 1] == MESG_CLOSE_CHANNEL_ID)) {
          uint8_t data[2] = {0x01, EVENT_CHANNEL_CLOSED};
          ant_construct_message(MESG_RESPONSE_EVENT_ID, MESG_RESPONSE_EVENT_SIZE, 0, reply, data);
          transmit_message(reply, ANT_MESSAGE_SIZE(reply), 20);
        }
      } else {
        if (DEBUG) {
          printf("ANT+ Rx Error!");
        }
      }
    }

    // check for virtual master device messages
    xReceivedBytes = xMessageBufferReceive(xAntDeviceRxBuffer, (void *) ucRxData, sizeof(ucRxData), xBlockTime);
    // if there is one, send it on an open assigned channel if there is one
    if (xReceivedBytes > 0) {
      ant_process_tx_event(ucRxData, xReceivedBytes);
    }
  }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

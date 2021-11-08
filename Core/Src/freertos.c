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
#include "timers.h"
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

// timers for emulated master data sending
TimerHandle_t xAntPowerTimer;
StaticTimer_t xAntPowerTimerStatic;
extern ANT_Device_t power_device;
TimerHandle_t xAntFECTimer;
StaticTimer_t xAntFECTimerStatic;
extern ANT_Device_t fec_device;
static uint8_t timer_data[8] = {0};
static uint8_t timer_msg[ANT_MAX_SIZE] = {0};

static char printBuffer[64];

osThreadId_t antProtocolTask;
const osThreadAttr_t antProtocolTask_attributes = {
  .name = "antProtocolTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 6
};

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void AntProtocolTask(void *argument);

// this timer callback is designed to emulate the master update period to put data broadcast events in the Tx stream
void vTimerCallback(TimerHandle_t xTimer) {
  uint32_t id;

  /* Optionally do something if the pxTimer parameter is NULL. */
  configASSERT(xTimer);

  /* The number of times this timer has expired is saved as the
     timer's ID.  Obtain the count. */
  id = (uint32_t) pvTimerGetTimerID(xTimer);

  if (id == power_device.tid) {
    ant_generate_data_page(&power_device, timer_data);
    ant_construct_data_message(MESG_BROADCAST_DATA_ID, MESG_EXTENDED_SIZE, &power_device, timer_msg, timer_data);
    xMessageBufferSend(xAntDeviceRxBuffer, timer_msg, ANT_MESSAGE_SIZE(timer_msg), 0);
    HAL_GPIO_TogglePin(GLED_GPIO_Port, GLED_Pin);

  } else if (id == fec_device.tid) {
    ant_generate_data_page(&fec_device, timer_data);
    ant_construct_data_message(MESG_BROADCAST_DATA_ID, MESG_EXTENDED_SIZE, &fec_device, timer_msg, timer_data);
    xMessageBufferSend(xAntDeviceRxBuffer, timer_msg, ANT_MESSAGE_SIZE(timer_msg), 0);
    HAL_GPIO_TogglePin(GLED_GPIO_Port, GLED_Pin);
  }

}

void ant_device_init(ANT_Device_t *dev, TimerHandle_t *timer, StaticTimer_t *stimer, char *name) {
  *timer = xTimerCreateStatic(
      name,
      pdMS_TO_TICKS(PERIOD_TO_MS(dev->period)),
      pdTRUE,
      // assign timer id
      (void*) (uint32_t) dev->tid,
      vTimerCallback,
      stimer
  );

  if( *timer == NULL ) {
    Error_Handler();
  } else {
    dev->timer = timer;
    // don't start here, start when channel is configured
    /* xTimerStart(*timer, 0); */
  }
}
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
  ant_device_init(&power_device, &xAntPowerTimer, &xAntPowerTimerStatic, "PowerTimer");
  ant_device_init(&fec_device, &xAntFECTimer, &xAntFECTimerStatic, "FECTimer");
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
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void transmit_message(uint8_t *pBuffer, size_t len, uint8_t block_tick) {
  uint8_t status = USBD_BUSY;
  uint8_t block_ticks = 0;

  while (status != USBD_OK && block_ticks < block_tick) {
    status = USBD_TEMPLATE_Transmit(&hUsbDeviceFS, pBuffer, len);
    block_ticks += 2;
    osDelay(2);
  }
}

void AntProtocolTask(void *argument) {
  printf("ANT+ Protocol Task Starting\r\n");

  /* Infinite loop */
  for(;;)
  {
    uint8_t reply[ANT_MAX_SIZE] = {0};
    uint8_t ucRxData[64];
    size_t xReceivedBytes;
    // the block time needs to be faster than emulated devices are loading up transmit buffer
    const TickType_t xBlockTime = pdMS_TO_TICKS(5);
    ANT_MessageStatus_t status = 0;

    /* Receive the next message from the message buffer.  Wait in the Blocked
       state (so not using any CPU processing time) for a maximum of 100ms for
       a message to become available. */
    xReceivedBytes = xMessageBufferReceive(xAntMsgBuffer, (void *) ucRxData, sizeof(ucRxData), xBlockTime);
    // clear after block allows blink on rx
    HAL_GPIO_WritePin(USBD_RX_LED_PORT, USBD_RX_LED, 1);

    if(xReceivedBytes > 0) {
      if (DEBUG_ANT) {
        printf("ANT+ Rx: ");
        print_hex((char*) ucRxData, xReceivedBytes);
      }
      HAL_GPIO_WritePin(USBD_RX_LED_PORT, USBD_RX_LED, 0);

      status = process_ant_message(ucRxData, xReceivedBytes, reply);
      if (status != MESG_UNKNOWN_ID) {
        if (DEBUG_ANT) {
          if (reply[2] == 0xAE) printf("ANT+ Rx Error, Tx: ");
          else printf("ANT+ Tx: ");
          print_hex((char*) reply, ANT_MESSAGE_SIZE(reply));
        }

        // TODO replace with message stream for replies
        if (status != MESG_NO_REPLY) {
          transmit_message(reply, ANT_MESSAGE_SIZE(reply), 20);
          // close channel request follow with channel event closed
          if (reply[4] == MESG_CLOSE_CHANNEL_ID && (ucRxData[BUFFER_INDEX_MESG_ID + 1] == MESG_CLOSE_CHANNEL_ID)) {
            uint8_t data[2] = {0x01, EVENT_CHANNEL_CLOSED};
            ant_construct_message(MESG_RESPONSE_EVENT_ID, MESG_RESPONSE_EVENT_SIZE, 0, reply, data);
            transmit_message(reply, ANT_MESSAGE_SIZE(reply), 20);
          }
        } else {
          printf("ANT+ No reply");
        }
      } else {
        if (DEBUG_ANT) {
          printf("ANT+ Rx Error!");
        }
      }
    }

    // check for virtual master device messages
    xReceivedBytes = xMessageBufferReceive(xAntDeviceRxBuffer, (void *) ucRxData, sizeof(ucRxData), 0);
    // if there is one, send it on an open assigned channel if there is one
    if (xReceivedBytes > 0) {
      ant_process_tx_event(ucRxData, xReceivedBytes);
    }
  }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

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
MessageBufferHandle_t xAntMsgBuffer = NULL;
static uint8_t ucStorageBuffer[ANT_EP_OUT_BUFFER_SIZE];
static StaticMessageBuffer_t xAntMsgBufferStruct;
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
  xAntMsgBuffer = xMessageBufferCreateStatic(sizeof(ucStorageBuffer), ucStorageBuffer, &xAntMsgBufferStruct);
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
    HAL_GPIO_TogglePin(BLED_GPIO_Port, BLED_Pin);
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
void AntProtocolTask(void *argument) {
  printf("ANT+ Protocol Task Starting\r\n");

  /* Infinite loop */
  for(;;)
  {
    uint8_t reply[ANT_MAX_SIZE] = {0};
    uint8_t ucRxData[64];
    size_t xReceivedBytes;
    const TickType_t xBlockTime = pdMS_TO_TICKS( 100 );
    uint8_t status = USBD_BUSY;

    /* Receive the next message from the message buffer.  Wait in the Blocked
       state (so not using any CPU processing time) for a maximum of 100ms for
       a message to become available. */
    xReceivedBytes = xMessageBufferReceive(xAntMsgBuffer,
        (void *) ucRxData,
        sizeof(ucRxData),
        xBlockTime);
    // clear after block allows blink on rx
    HAL_GPIO_WritePin(GLED_GPIO_Port, GLED_Pin, 1);

    if( xReceivedBytes > 0 ) {
      if (DEBUG) {
        printf("ANT+ Rx: ");
        print_hex((char*) ucRxData, xReceivedBytes);
      }
      HAL_GPIO_WritePin(GLED_GPIO_Port, GLED_Pin, 0);


      if (process_ant_message(ucRxData, xReceivedBytes, reply) == MESG_OK) {
        if (DEBUG) {
          printf("ANT+ Tx: ");
          print_hex((char*) reply, ANT_MESSAGE_SIZE(reply));
        }

        // TODO replace with message stream for replies
        while (status != USBD_OK) {
          status = USBD_TEMPLATE_Transmit(&hUsbDeviceFS, reply, ANT_MESSAGE_SIZE(reply));
          osDelay(2);
        }
      } else {
        if (DEBUG) {
          printf("ANT+ Rx Error!");
        }
      }
    }
  }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

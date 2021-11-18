/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "message_buffer.h"
#include "semphr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
  float ke; // system kinetic energy (N)
  float _ke; // last system kinetic energy (N)
  float delta_ke; // change in energy (N)
  float omega; // flywheel angular velocity (rad/s)
  uint16_t vin; // input voltage (mV)
  uint16_t csense; // sense current (mA)
  uint16_t emf; // emf sense (mV)
  uint32_t rps; // revolutions per second of flywheel
} system_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern system_t gsystem;
// semaphore to get freq tick
extern SemaphoreHandle_t xTim2Semaphore;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void thread_printf(uint8_t* pBuffer, size_t len, uint8_t block_tick);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VSENSE_Pin GPIO_PIN_0
#define VSENSE_GPIO_Port GPIOA
#define ISENSE_Pin GPIO_PIN_1
#define ISENSE_GPIO_Port GPIOA
#define EMF_SENSE_Pin GPIO_PIN_2
#define EMF_SENSE_GPIO_Port GPIOA
#define RLED_Pin GPIO_PIN_0
#define RLED_GPIO_Port GPIOB
#define GLED_Pin GPIO_PIN_1
#define GLED_GPIO_Port GPIOB
#define BLED_Pin GPIO_PIN_2
#define BLED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define DEBUG                     1
#define DEBUG_ANT                 DEBUG && 0 // send ant data to uart
#define DEBUG_CH_TX_EVENT         DEBUG && 0
#define DEBUG_TRAINER             DEBUG && 1 // send csv stream to uart
// PA5 is timer counter input (light gate sense) A1 on Feather
// PA6 is PWM output (emf MOSFET) A2 on Feather

#define USBD_RX_LED_PORT          RLED_GPIO_Port
#define USBD_RX_LED               RLED_Pin
#define BLINK_LED_PORT            BLED_GPIO_Port
#define BLINK_LED                 BLED_Pin
#define ANT_CONTROL_LED_PORT      GLED_GPIO_Port
#define ANT_CONTROL_LED           GLED_Pin

#define VSENSE_GAIN               10.0f // potential divider for VIN measurement
#define ISENSE_GAIN               1.0f // current sense gain
#define EMFSENSE_GAIN             1.0f // potential divider for EMF sense

#define UART_TX_BUFFER_SIZE       256 // this is the buffered uart queue used by thread_printf
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

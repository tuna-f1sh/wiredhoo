/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"

/* USER CODE BEGIN 0 */
#define F_CLK                     84000000UL // APB2

// FREQ COUNTER TIM
#define IDLE                      0
#define DONE                      1
#define IC_DIV                    1
#define IC_FREQ                   F_CLK/IC_DIV
#define IC_COUNTER_TOP            UINT16_MAX

#define TIMEOUT_PERIOD            5000 // ms
#define TIMEOUT_OVC_COUNT         ((TIMEOUT_PERIOD * IC_FREQ) / 1000) / IC_COUNTER_TOP

// PWM TIM
#define PWM_FREQ 1000UL // roughly
#define TIMER_PRESCALER 32
#define TIMER_PERIOD_COUNT (uint16_t) (F_CLK / (PWM_FREQ * TIMER_PRESCALER))

volatile static uint8_t sCaptureState = IDLE;
volatile static uint32_t sTick = 0;
volatile static uint32_t sTIM2_OVC = 0;
volatile static uint32_t sTicks = 0;
uint16_t tim3_top = TIMER_PERIOD_COUNT;
/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* TIM2 init function */
void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = IC_DIV;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = IC_COUNTER_TOP;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  // Capture filtering. Will need changing on full encoder trainers. Kickr with 1 per rev I found was picking up noise. See https://www.st.com/content/ccc/resource/technical/document/application_note/group0/91/01/84/3f/7c/67/41/3f/DM00236305/files/DM00236305.pdf/jcr:content/translations/en.DM00236305.pdf 1.4
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  // dicates sampling frequency of filter F_dts = F_clk [IC_FREQ] / pre
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV2;
  /* sConfigIC.ICFilter = 0b1000; // fSAMPLING=fDTS/8, N=6 18/4/8 = 1.1 MHz, 6 samples = 187.5 kHz max signal */
  // max
  sConfigIC.ICFilter = 0b1111; // fSAMPLING=fDTS/32, N=8 18/2/32 = 281 kHz, 8 samples = 35 kHz max signal
  // off
  /* sConfigIC.ICFilter = 0; */
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

}
/* TIM3 init function */
void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = TIMER_PRESCALER;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = TIMER_PERIOD_COUNT;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);

}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* tim_icHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(tim_icHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA5     ------> TIM2_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PA6     ------> TIM3_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }

}

void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef* tim_icHandle)
{

  if(tim_icHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /**TIM2 GPIO Configuration
    PA5     ------> TIM2_CH1
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);

    /* TIM2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
// counter input capture on PA5 (tim2/ch1)
void tim2_capture_setup(void) {
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
}

uint32_t tim2_calc_frequency(void) {
  uint32_t ret = 0;

  // clock of TIM2 divided by the count in one cycle
  xSemaphoreTake(xTim2Semaphore, 5);
  if (sTicks != 0) {
    ret = IC_FREQ / sTicks;
  }
  xSemaphoreGive(xTim2Semaphore);

  return ret;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM2) {

    // first sample
    if(sCaptureState == IDLE) {

      sTick = TIM2->CCR1;
      sTIM2_OVC = 0;
      sCaptureState = DONE;

    // after first, T1 updates during calculation
    } else if(sCaptureState == DONE) {

      if (xSemaphoreTakeFromISR(xTim2Semaphore, NULL) == pdTRUE) {
        // calculate period in ticks since last T1 using current count value as T2
        sTicks = (TIM2->CCR1 + (sTIM2_OVC * (IC_COUNTER_TOP + 1))) - sTick;
        xSemaphoreGiveFromISR(xTim2Semaphore, NULL);
      }

      // set new T1 to current count for next edge
      sTick = TIM2->CCR1;
      // reset over counter
      sTIM2_OVC = 0;
    }
  }
}

void tim2_period_elapsed_callback(void) {
  // if not reached overcount count inc
  if (sTIM2_OVC < TIMEOUT_OVC_COUNT) {
    sTIM2_OVC++;

  // else, timeout so set freq to zero
  } else {

    if (xSemaphoreTakeFromISR(xTim2Semaphore, NULL) == pdTRUE) {
      // clear tick count will read from
      sTicks = 0;

      // reset state for new measurement
      sCaptureState = IDLE;
    }

  }
}

// PWM output on PA6 (tim3/ch1)
void tim3_pwm_init(void) {
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void tim3_pwm_set_value(uint32_t channel, uint16_t value) {
  __HAL_TIM_SET_COMPARE(&htim3, channel, value);
}

void tim3_pwm_set_duty(uint32_t channel, uint8_t duty) {
  // 100% set compare to larger than period so high all time
  if (duty == 100) {
    tim3_pwm_set_value(channel, tim3_top + 1);
  } else {
    uint16_t count = (uint16_t) (((uint32_t) duty * tim3_top) / 100);
    tim3_pwm_set_value(channel, count);
  }
}

void tim3_pwm_set_freq(uint16_t freq) {
  tim3_top = (uint16_t) (F_CLK / (freq * TIMER_PRESCALER));
  __HAL_TIM_SET_AUTORELOAD(&htim3, tim3_top);
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* TIM2 init function */
void MX_TIM2_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 84000000 / ROC_TIMER_PRESCALER_TIM2 - 1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 200 - 1;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* TIM6 init function */
void MX_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig;

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 84000000 / ROC_TIMER_PRESCALER_TIM6 - 1;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = ROC_TIMER_PERIOD_TIM6 - 1;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* TIM7 init function */
void MX_TIM7_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig;

    htim7.Instance = TIM7;
    htim7.Init.Prescaler = 84000000 / ROC_TIMER_PRESCALER_TIM7 - 1;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = ROC_TIMER_PERIOD_TIM7 - 1;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_baseHandle)
{

    if(tim_baseHandle->Instance == TIM2)
    {
        /* USER CODE BEGIN TIM2_MspInit 0 */

        /* USER CODE END TIM2_MspInit 0 */
        /* TIM2 clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();
        /* USER CODE BEGIN TIM2_MspInit 1 */

        /* USER CODE END TIM2_MspInit 1 */
    }
    else if(tim_baseHandle->Instance == TIM6)
    {
        /* USER CODE BEGIN TIM6_MspInit 0 */

        /* USER CODE END TIM6_MspInit 0 */
        /* TIM6 clock enable */
        __HAL_RCC_TIM6_CLK_ENABLE();
        /* USER CODE BEGIN TIM6_MspInit 1 */

        /* USER CODE END TIM6_MspInit 1 */
    }
    else if(tim_baseHandle->Instance == TIM7)
    {
        /* USER CODE BEGIN TIM7_MspInit 0 */

        /* USER CODE END TIM7_MspInit 0 */
        /* TIM7 clock enable */
        __HAL_RCC_TIM7_CLK_ENABLE();
        /* USER CODE BEGIN TIM7_MspInit 1 */

        /* USER CODE END TIM7_MspInit 1 */
    }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *tim_baseHandle)
{

    if(tim_baseHandle->Instance == TIM2)
    {
        /* USER CODE BEGIN TIM2_MspDeInit 0 */

        /* USER CODE END TIM2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM2_CLK_DISABLE();

        /* TIM2 interrupt Deinit */
        HAL_NVIC_DisableIRQ(TIM2_IRQn);
        /* USER CODE BEGIN TIM2_MspDeInit 1 */

        /* USER CODE END TIM2_MspDeInit 1 */
    }
    else if(tim_baseHandle->Instance == TIM6)
    {
        /* USER CODE BEGIN TIM6_MspDeInit 0 */

        /* USER CODE END TIM6_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM6_CLK_DISABLE();

        /* TIM6 interrupt Deinit */
        HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
        /* USER CODE BEGIN TIM6_MspDeInit 1 */

        /* USER CODE END TIM6_MspDeInit 1 */
    }
    else if(tim_baseHandle->Instance == TIM7)
    {
        /* USER CODE BEGIN TIM7_MspDeInit 0 */

        /* USER CODE END TIM7_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM7_CLK_DISABLE();

        /* TIM6 interrupt Deinit */
        HAL_NVIC_DisableIRQ(TIM7_IRQn);
        /* USER CODE BEGIN TIM7_MspDeInit 1 */

        /* USER CODE END TIM7_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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
#include "gpio.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(ROC_LED_1_PORT, ROC_LED_1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ROC_LED_2_PORT, ROC_LED_2_PIN, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(ROC_BEEPER_GPIO_PORT, ROC_BEEPER_CTRL_PIN, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(ROC_OLED_RST_PORT, ROC_OLED_RST_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ROC_OLED_DC_PORT, ROC_OLED_DC_PIN, GPIO_PIN_RESET);

    /*Configure GPIO pins : LED1 Pin */
    GPIO_InitStruct.Pin = ROC_LED_1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ROC_LED_1_PORT, &GPIO_InitStruct);

    /*Configure GPIO pins : LED2 Pin */
    GPIO_InitStruct.Pin = ROC_LED_2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ROC_LED_2_PORT, &GPIO_InitStruct);

    /*Configure GPIO pins : Beeper Pin */
    GPIO_InitStruct.Pin = ROC_BEEPER_CTRL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ROC_BEEPER_GPIO_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : ROC_KEY_1_PIN */
    GPIO_InitStruct.Pin = ROC_KEY_1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ROC_KEY_1_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : ROC_KEY_2_PIN */
    GPIO_InitStruct.Pin = ROC_KEY_2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ROC_KEY_2_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : ROC_KEY_3_PIN */
    GPIO_InitStruct.Pin = ROC_KEY_3_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ROC_KEY_3_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : ROC_KEY_4_PIN */
    GPIO_InitStruct.Pin = ROC_KEY_4_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ROC_KEY_4_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : ROC_KEY_5_PIN */
    GPIO_InitStruct.Pin = ROC_KEY_5_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ROC_KEY_5_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : ROC_KEY_6_PIN */
    GPIO_InitStruct.Pin = ROC_KEY_6_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ROC_KEY_6_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : ROC_KEY_7_PIN */
    GPIO_InitStruct.Pin = ROC_KEY_7_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ROC_KEY_7_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : ROC_KEY_8_PIN */
    GPIO_InitStruct.Pin = ROC_KEY_8_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ROC_KEY_8_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : ROC_KEY_9_PIN */
    GPIO_InitStruct.Pin = ROC_KEY_9_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ROC_KEY_9_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : ROC_KEY_10_PIN */
    GPIO_InitStruct.Pin = ROC_KEY_10_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ROC_KEY_10_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : ROC_KEY_11_PIN */
    GPIO_InitStruct.Pin = ROC_KEY_11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ROC_KEY_11_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : ROC_KEY_12_PIN */
    GPIO_InitStruct.Pin = ROC_KEY_12_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ROC_KEY_12_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : ROC_KEY_13_PIN */
    GPIO_InitStruct.Pin = ROC_KEY_13_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ROC_KEY_13_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : ROC_KEY_14_PIN */
    GPIO_InitStruct.Pin = ROC_KEY_14_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ROC_KEY_14_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : ROC_KEY_15_PIN */
    GPIO_InitStruct.Pin = ROC_KEY_15_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ROC_KEY_15_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : ROC_KEY_LT_PIN */
    GPIO_InitStruct.Pin = ROC_KEY_LT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ROC_KEY_LT_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : ROC_KEY_RT_PIN */
    GPIO_InitStruct.Pin = ROC_KEY_RT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ROC_KEY_RT_PORT, &GPIO_InitStruct);

    /*Configure GPIO pins : OLED Pin */
    GPIO_InitStruct.Pin = ROC_OLED_RST_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ROC_OLED_RST_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ROC_OLED_DC_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ROC_OLED_DC_PORT, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

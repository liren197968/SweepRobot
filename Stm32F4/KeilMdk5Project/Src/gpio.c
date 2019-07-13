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
    HAL_GPIO_WritePin(ROC_PCA9685_A_EN_GPIO_PORT, ROC_PCA9685_A_EN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ROC_PCA9685_B_EN_GPIO_PORT, ROC_PCA9685_B_EN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ROC_PCA9685_C_EN_GPIO_PORT, ROC_PCA9685_C_EN_PIN, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(ROC_DEBUG_LED_GPIO_PORT, ROC_DEBUG_LED_PIN, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(ROC_BEEPER_GPIO_PORT, ROC_BEEPER_CTRL_PIN, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(ROC_TFTLCD_DC_PORT, ROC_TFTLCD_DC_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ROC_TFTLCD_RST_PORT, ROC_TFTLCD_RST_PIN, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(ROC_MOTOR_GPIO_PORT1, ROC_MOTOR_IN1_PIN | ROC_MOTOR_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ROC_MOTOR_GPIO_PORT2, ROC_MOTOR_IN3_PIN | ROC_MOTOR_IN4_PIN, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(ROC_OLED_RST_PORT, ROC_OLED_RST_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ROC_OLED_DC_PORT, ROC_OLED_DC_PIN, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(ROC_ROBOT_RELAY_PORT, ROC_ROBOT_RELAY_PIN, GPIO_PIN_SET);

    /*Configure GPIO pins : PCA9685 ENA Pin */
    GPIO_InitStruct.Pin = ROC_PCA9685_A_EN_PIN ;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ROC_PCA9685_B_EN_GPIO_PORT, &GPIO_InitStruct);

    /*Configure GPIO pins : PCA9685 ENB Pin */
    GPIO_InitStruct.Pin =  ROC_PCA9685_B_EN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ROC_PCA9685_B_EN_GPIO_PORT, &GPIO_InitStruct);

    /*Configure GPIO pins : PCA9685 ENC Pin */
    GPIO_InitStruct.Pin =  ROC_PCA9685_C_EN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ROC_PCA9685_C_EN_GPIO_PORT, &GPIO_InitStruct);

    /*Configure GPIO pins : Beeper Pin */
    GPIO_InitStruct.Pin = ROC_BEEPER_CTRL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ROC_BEEPER_GPIO_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : LED Pin */
    GPIO_InitStruct.Pin = ROC_DEBUG_LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ROC_DEBUG_LED_GPIO_PORT, &GPIO_InitStruct);

    /*Configure GPIO pins : TFTLCD DC Pin */
    GPIO_InitStruct.Pin = ROC_TFTLCD_DC_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ROC_TFTLCD_DC_PORT, &GPIO_InitStruct);

    /*Configure GPIO pins : TFTLCD RST Pin */
    GPIO_InitStruct.Pin = ROC_TFTLCD_RST_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ROC_TFTLCD_RST_PORT, &GPIO_InitStruct);

    /*Configure GPIO pins : Motor Pin */
    GPIO_InitStruct.Pin = ROC_MOTOR_IN1_PIN | ROC_MOTOR_IN2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ROC_MOTOR_GPIO_PORT1, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ROC_MOTOR_IN3_PIN | ROC_MOTOR_IN4_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ROC_MOTOR_GPIO_PORT2, &GPIO_InitStruct);

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

    /*Configure GPIO pins : RELAY Pin */
    GPIO_InitStruct.Pin = ROC_ROBOT_RELAY_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ROC_ROBOT_RELAY_PORT, &GPIO_InitStruct);

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

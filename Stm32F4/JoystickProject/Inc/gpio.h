/**
  ******************************************************************************
  * File Name          : gpio.h
  * Description        : This file contains all the functions prototypes for
  *                      the gpio
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
extern "C" {
#endif

    /* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */
#define ROC_LED_DEBUG_PIN                   GPIO_PIN_8
#define ROC_LED_DEBUG_PORT                  GPIOA
#define ROC_LED_1_PIN                       GPIO_PIN_14
#define ROC_LED_1_PORT                      GPIOB
#define ROC_LED_2_PIN                       GPIO_PIN_15
#define ROC_LED_2_PORT                      GPIOB

#define ROC_ROBOT_BATTERY_ADC_PIN           GPIO_PIN_4
#define ROC_ROBOT_BATTERY_ADC_PORT          GPIOA

#define ROC_BEEPER_CTRL_PIN                 GPIO_PIN_15
#define ROC_BEEPER_GPIO_PORT                GPIOA

#define ROC_KEY_1_PIN                       GPIO_PIN_1
#define ROC_KEY_1_PORT                      GPIOC
#define ROC_KEY_2_PIN                       GPIO_PIN_2
#define ROC_KEY_2_PORT                      GPIOC
#define ROC_KEY_3_PIN                       GPIO_PIN_3
#define ROC_KEY_3_PORT                      GPIOC
#define ROC_KEY_4_PIN                       GPIO_PIN_4
#define ROC_KEY_4_PORT                      GPIOC
#define ROC_KEY_5_PIN                       GPIO_PIN_5
#define ROC_KEY_5_PORT                      GPIOC
#define ROC_KEY_6_PIN                       GPIO_PIN_6
#define ROC_KEY_6_PORT                      GPIOC
#define ROC_KEY_7_PIN                       GPIO_PIN_7
#define ROC_KEY_7_PORT                      GPIOC
#define ROC_KEY_8_PIN                       GPIO_PIN_8
#define ROC_KEY_8_PORT                      GPIOC
#define ROC_KEY_9_PIN                       GPIO_PIN_9
#define ROC_KEY_9_PORT                      GPIOC
#define ROC_KEY_10_PIN                      GPIO_PIN_10
#define ROC_KEY_10_PORT                     GPIOC
#define ROC_KEY_11_PIN                      GPIO_PIN_11
#define ROC_KEY_11_PORT                     GPIOC
#define ROC_KEY_12_PIN                      GPIO_PIN_8
#define ROC_KEY_12_PORT                     GPIOB
#define ROC_KEY_13_PIN                      GPIO_PIN_9
#define ROC_KEY_13_PORT                     GPIOB
#define ROC_KEY_14_PIN                      GPIO_PIN_10
#define ROC_KEY_14_PORT                     GPIOB
#define ROC_KEY_15_PIN                      GPIO_PIN_11
#define ROC_KEY_15_PORT                     GPIOB
#define ROC_KEY_LT_PIN                      GPIO_PIN_11
#define ROC_KEY_LT_PORT                     GPIOA
#define ROC_KEY_RT_PIN                      GPIO_PIN_12
#define ROC_KEY_RT_PORT                     GPIOA
#define ROC_KEY_LX_PIN                      GPIO_PIN_0
#define ROC_KEY_LX_PORT                     GPIOA
#define ROC_KEY_LY_PIN                      GPIO_PIN_1
#define ROC_KEY_LY_PORT                     GPIOA
#define ROC_KEY_RX_PIN                      GPIO_PIN_0
#define ROC_KEY_RX_PORT                     GPIOB
#define ROC_KEY_RY_PIN                      GPIO_PIN_1
#define ROC_KEY_RY_PORT                     GPIOB

#define ROC_OLED_DC_PIN                     GPIO_PIN_4
#define ROC_OLED_DC_PORT                    GPIOB
#define ROC_OLED_RST_PIN                    GPIO_PIN_2
#define ROC_OLED_RST_PORT                   GPIOB

#define ROC_RF_SET_PIN                      GPIO_PIN_8
#define ROC_RF_SET_PORT                     GPIOA

#define RFNSS_Pin                           GPIO_PIN_4
#define RFNSS_GPIO_Port                     GPIOA
#define RFSCK_Pin                           GPIO_PIN_5
#define RFSCK_GPIO_Port                     GPIOA
#define RFMISO_Pin                          GPIO_PIN_6
#define RFMISO_GPIO_Port                    GPIOA
#define RFMOSI_Pin                          GPIO_PIN_7
#define RFMOSI_GPIO_Port                    GPIOA
#define RFDIO_3_Pin                         GPIO_PIN_0
#define RFDIO_3_GPIO_Port                   GPIOB
#define RFDIO_3_EXTI_IRQn                   EXTI0_IRQn
#define RFDIO_2_Pin                         GPIO_PIN_1
#define RFDIO_2_GPIO_Port                   GPIOB
#define RFDIO_2_EXTI_IRQn                   EXTI1_IRQn
#define RFDIO_1_Pin                         GPIO_PIN_7
#define RFDIO_1_GPIO_Port                   GPIOB
#define RFDIO_1_EXTI_IRQn                   EXTI9_5_IRQn
#define RFDIO_0_Pin                         GPIO_PIN_6
#define RFDIO_0_GPIO_Port                   GPIOB
#define RFDIO_0_EXTI_IRQn                   EXTI9_5_IRQn

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
#define RCC_GPIO_CLK_ENABLE( __GPIO_PORT__ )              \
do {                                                    \
    switch( __GPIO_PORT__)                                \
    {                                                     \
      case GPIOA_BASE: __HAL_RCC_GPIOA_CLK_ENABLE(); break;    \
      case GPIOB_BASE: __HAL_RCC_GPIOB_CLK_ENABLE(); break;    \
      case GPIOC_BASE: __HAL_RCC_GPIOC_CLK_ENABLE(); break;    \
      case GPIOD_BASE: __HAL_RCC_GPIOD_CLK_ENABLE(); break;    \
    }                                                    \
  } while(0)

typedef void( GpioIrqHandler )( void );
IRQn_Type MSP_GetIRQn( uint16_t gpioPin);
void HW_GPIO_Init( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_InitTypeDef* initStruct);
void HW_GPIO_SetIrq( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t prio,  GpioIrqHandler *irqHandler );
void HW_GPIO_IrqHandler( uint16_t GPIO_Pin );
void HW_GPIO_Write( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,  uint32_t value );
uint32_t HW_GPIO_Read( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin );

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

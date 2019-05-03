/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2018/12/15      1.0
********************************************************************************/
#ifndef __ROC_PCA9685_H
#define __ROC_PCA9685_H


#include "gpio.h"


#define PCA9685_SUBADR1                 0x02
#define PCA9685_SUBADR2                 0x03
#define PCA9685_SUBADR3                 0x04

#define PCA9685_MODE1                   0x00
#define PCA9685_PRESCALE                0xFE

#define LED0_ON_L                       0x06
#define LED0_ON_H                       0x07
#define LED0_OFF_L                      0x08
#define LED0_OFF_H                      0x09

#define ALLLED_ON_L                     0xFA
#define ALLLED_ON_H                     0xFB
#define ALLLED_OFF_L                    0xFC
#define ALLLED_OFF_H                    0xFD

#define PWM_ADDRESS_L                   0x80
#define PWM_ADDRESS_H                   0xC0

#define ROC_PCA9685_A_EN                GPIO_PIN_2
#define ROC_PCA9685_B_EN                GPIO_PIN_3


#define ROC_PCA9685_DATA_REG_NUM        4U
#define ROC_PCA9685_CHANNEL_MAX_NUM     16U


ROC_RESULT RocPca9685Init(void);
void RocPca9685PwmOutEnable(void);
void RocPca9685PwmOutDisable(void);
HAL_StatusTypeDef RocPca9685OutPwmAll(uint8_t SlaveAddr, uint16_t *pPwmData, uint16_t PwmDataNum);
HAL_StatusTypeDef RocPca9685SetPinOutPwm(uint8_t SlaveAddr, uint8_t NumPin, uint16_t Val, uint8_t Invert);
HAL_StatusTypeDef RocPca9685OutPwm(uint8_t SlaveAddr, uint8_t NumPin, uint16_t LedOnTime, uint16_t LedOffTime);

#endif


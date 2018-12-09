#ifndef _ROC_PCA9685_H
#define _ROC_PCA9685_H

#include "gpio.h"

#define PCA9685_SUBADR1         0x2
#define PCA9685_SUBADR2         0x3
#define PCA9685_SUBADR3         0x4

#define PCA9685_MODE1           0x00
#define PCA9685_PRESCALE        0xFE

#define LED0_ON_L               0x6
#define LED0_ON_H               0x7
#define LED0_OFF_L              0x8
#define LED0_OFF_H              0x9

#define ALLLED_ON_L             0xFA
#define ALLLED_ON_H             0xFB
#define ALLLED_OFF_L            0xFC
#define ALLLED_OFF_H            0xFD

#define PWM_ADDRESS_L           0x80
#define PWM_ADDRESS_H           0xC0


#define ROC_PCA9685_A_EN        GPIO_PIN_2
#define ROC_PCA9685_B_EN        GPIO_PIN_3


void RocPca9685Enable(void);
void RocPca9685Disable(void);
void RocPca9685Init(void);
void RocPca9685SetPin(uint8_t SlaveAddr, uint8_t Num, uint16_t Val, uint8_t Invert);
HAL_StatusTypeDef RocPca9685OutPwm(uint8_t SlaveAddr, uint8_t Num, uint16_t HigBitDat, uint16_t LowBitDat);

#endif


/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2018/12/16      1.0
********************************************************************************/
#ifndef __ROC_LED_H
#define __ROC_LED_H


#define ROC_DEBUG_LED_PIN       GPIO_PIN_7
#define ROC_DEBUG_LED_PORT      GPIOA

#define ROC_DEBUG_LED_ON        GPIO_PIN_RESET
#define ROC_DEBUG_LED_OFF       GPIO_PIN_SET


void RocLedToggle(void);
void RocLedTurnOn(void);
void RocLedTurnOff(void);
ROC_RESULT RocLedInit(void);

#endif

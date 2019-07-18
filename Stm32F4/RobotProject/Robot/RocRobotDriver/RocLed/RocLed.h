/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2018/12/16      1.0
********************************************************************************/
#ifndef __ROC_LED_H
#define __ROC_LED_H


#define ROC_DEBUG_LED_ON        GPIO_PIN_RESET
#define ROC_DEBUG_LED_OFF       GPIO_PIN_SET


typedef enum _ROC_LED_TYPE_e
{
    ROC_LED_DEBUG = 0,
    ROC_LED_1,
    ROC_LED_2,
    ROC_LED_3,
    ROC_LED_4,
    ROC_LED_5,

    ROC_LED_NUM
}ROC_LED_TYPE_e;


void RocLedToggle(ROC_LED_TYPE_e Led);
void RocLedTurnOn(ROC_LED_TYPE_e Led);
void RocLedTurnOff(ROC_LED_TYPE_e Led);
ROC_RESULT RocLedInit(void);

#endif


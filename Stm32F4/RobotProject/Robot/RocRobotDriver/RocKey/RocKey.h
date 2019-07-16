/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/07/14      1.0
********************************************************************************/
#ifndef __ROC_KEY_H
#define __ROC_KEY_H


#define ROC_KEY_DURATION_TIME_MASK      0xFF    /* 8 * timer cysle */

typedef enum _ROC_KEY_TYPE_e
{
    ROC_KEY_1 = 0,
    ROC_KEY_2,
    ROC_KEY_3,
    ROC_KEY_4,
    ROC_KEY_5,
    ROC_KEY_6,
    ROC_KEY_7,
    ROC_KEY_8,
    ROC_KEY_9,
    ROC_KEY_10,
    ROC_KEY_11,
    ROC_KEY_12,
    ROC_KEY_13,
    ROC_KEY_14,
    ROC_KEY_15,
    ROC_KEY_16,
    ROC_KEY_17,

    ROC_KEY_NUM
}ROC_KEY_TYPE_e;

void RocKeyTaskBackground(void);
uint32_t RocPressKeyNumGet(void);
void RocKeyEventClear(ROC_KEY_TYPE_e KeyNum);
ROC_RESULT RocKeyInit(void);


#endif


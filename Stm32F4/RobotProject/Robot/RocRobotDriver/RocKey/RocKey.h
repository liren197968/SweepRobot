/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/07/14      1.0
********************************************************************************/
#ifndef __ROC_KEY_H
#define __ROC_KEY_H

typedef enum _ROC_KEY_TYPE_e
{
    ROC_KEY_0,
    ROC_KEY_1,
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


#define ROC_KEY_DURATION_TIME_MASK      0x0F    /* 4 * timer cysle */

#define ROC_KEY_0_MASK                  (0U << ROC_KEY_1)
#define ROC_KEY_1_MASK                  (1U << ROC_KEY_1)
#define ROC_KEY_2_MASK                  (1U << ROC_KEY_2)
#define ROC_KEY_3_MASK                  (1U << ROC_KEY_3)
#define ROC_KEY_4_MASK                  (1U << ROC_KEY_4)
#define ROC_KEY_5_MASK                  (1U << ROC_KEY_5)
#define ROC_KEY_6_MASK                  (1U << ROC_KEY_6)
#define ROC_KEY_7_MASK                  (1U << ROC_KEY_7)
#define ROC_KEY_8_MASK                  (1U << ROC_KEY_8)
#define ROC_KEY_9_MASK                  (1U << ROC_KEY_9)
#define ROC_KEY_10_MASK                 (1U << ROC_KEY_10)
#define ROC_KEY_11_MASK                 (1U << ROC_KEY_11)
#define ROC_KEY_12_MASK                 (1U << ROC_KEY_12)
#define ROC_KEY_13_MASK                 (1U << ROC_KEY_13)
#define ROC_KEY_14_MASK                 (1U << ROC_KEY_14)
#define ROC_KEY_15_MASK                 (1U << ROC_KEY_15)
#define ROC_KEY_16_MASK                 (1U << ROC_KEY_16)
#define ROC_KEY_17_MASK                 (1U << ROC_KEY_17)


void RocKeyTaskBackground(void);
uint32_t RocPressKeyNumGet(void);
void RocKeyEventClear(ROC_KEY_TYPE_e KeyNum);
ROC_RESULT RocKeyInit(void);


#endif


/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/01/18      1.0
********************************************************************************/
#ifndef _ROC_BEEPER_H
#define _ROC_BEEPER_H


#define ROC_BEEPER_BLINK_FOREVER    0xFFFF


typedef enum _ROC_BEEPER_TYPE_e
{
    ROC_BEEPER_ACTIVE = 0,
    ROC_BEEPER_PASSIVE,

}ROC_BEEPER_TYPE_e;


typedef struct _ROC_BEEPER_CTRL_s
{
    uint16_t            RunTimes;
    uint16_t            RunFreq;
    ROC_BEEPER_TYPE_e   Type;

}ROC_BEEPER_CTRL_s;


void RocBeeperTaskBackground(void);
void RocBeeperBlink(uint16_t PeriodTime, uint16_t BlinkTimes);
ROC_RESULT RocBeeperInit(void);

#endif


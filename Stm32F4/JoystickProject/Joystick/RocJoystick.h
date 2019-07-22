/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/07/14      1.0
********************************************************************************/
#ifndef __ROC_JOYSTICK_H
#define __ROC_JOYSTICK_H


#define ROC_JOYSTICK_CTRL_TIME_TICK         25
#define ROC_BATTERY_CHECK_TIME_TICK         10


void RocJoystickInit(void);
void RocJoystickMain(void);


#endif


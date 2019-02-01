/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2018/12/16      1.0
********************************************************************************/
#ifndef __ROC_ROBOT_DH_H
#define __ROC_ROBOT_DH_H


#include <stdint.h>


extern uint16_t g_RobotStandPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2];
extern uint16_t g_RobotForwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2];
extern uint16_t g_RobotBackwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2];
extern uint16_t g_RobotCirclePwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2];


void RocRobotOpenLoopWalkCalculate(float Step, float Lift, uint16_t *pRobotCtrlPwmVal);
void RocRobotOpenLoopCircleCalculate(float DeltaAngle, float Lift, uint16_t *pRobotCtrlPwmVal);


#endif


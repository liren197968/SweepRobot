#ifndef __ROC_ROBOT_DH_H
#define __ROC_ROBOT_DH_H


#include <stdint.h>


extern uint16_t g_RobotStandPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2];
extern uint16_t g_RobotForwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2];
extern uint16_t g_RobotBackwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2];
extern uint16_t g_RobotCirclePwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2];
extern uint16_t g_RobotLeftForwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2];
extern uint16_t g_RobotLeftBackwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2];
extern uint16_t g_RobotRightForwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2];
extern uint16_t g_RobotRightBackwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2];


void RocRobotOpenLoopWalkCalculate(float Step, float Lift, uint16_t *pRobotCtrlPwmVal);
void RocRobotOpenLoopCircleCalculate(float DeltaAngle, float Lift, uint16_t *pRobotCtrlPwmVal);


#endif


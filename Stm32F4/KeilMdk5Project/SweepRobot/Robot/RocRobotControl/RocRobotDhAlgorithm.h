#ifndef __ROC_ROBOT_DH_H
#define __ROC_ROBOT_DH_H


#include <stdint.h>


#define ROC_ROBOT_D1                            0
#define ROC_ROBOT_A1                            44
#define ROC_ROBOT_A2                            74
#define ROC_ROBOT_A3                            83.24
#define ROC_ROBOT_A4                            19

#define ROC_INIT_ANGLE_THET_1                   60
#define ROC_INIT_ANGLE_THET_2                   0
#define ROC_INIT_ANGLE_THET_3                   60
#define ROC_INIT_ANGLE_BETA_1                   150
#define ROC_INIT_ANGLE_BETA_2                   90
#define ROC_INIT_ANGLE_BETA_3                   30

#define ROC_ROBOT_FIRST_STEP_ERROR              0
#define ROC_ROBOT_SECND_STEP_ERROR              4
#define ROC_ROBOT_LEFT_FIRST_STEP_ERROR         -2
#define ROC_ROBOT_LEFT_SECND_STEP_ERROR         0
#define ROC_ROBOT_RIGHT_FIRST_STEP_ERROR        -2
#define ROC_ROBOT_RIGHT_SECND_STEP_ERROR        0
#define ROC_ROBOT_PID_CONST_P                   1


extern uint16_t g_RobotStandPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2];
extern uint16_t g_RobotForwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2];
extern uint16_t g_RobotBackwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2];
extern uint16_t g_RobotCirclePwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2];
extern uint16_t g_RobotLeftForwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2];
extern uint16_t g_RobotLeftBackwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2];
extern uint16_t g_RobotRightForwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2];
extern uint16_t g_RobotRightBackwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2];


extern void RocAdjustStepCalculate(double g_ExpectedAngle, uint8_t Direction);
extern void RocAdjustLeftStepCalculate(double g_ExpectedAngle, uint8_t Direction);
extern void RocAdjustRightStepCalculate(double g_ExpectedAngle, uint8_t Direction);
extern void RocOpenLoopMoveCalculate(double FirstStepLength, double SecndStepLength);
extern void RocOpenLoopCircleCalculate(uint8_t DeltaAngle);


#endif


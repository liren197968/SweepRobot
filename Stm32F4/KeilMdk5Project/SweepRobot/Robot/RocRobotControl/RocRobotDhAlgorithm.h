#ifndef __ROC_ROBOT_DH_H
#define __ROC_ROBOT_DH_H


#include <stdint.h>


#define ROC_ROBOT_MATH_CONST_PI                     3.141593


#define ROC_ROBOT_DH_CONST_D1                       0
#define ROC_ROBOT_DH_CONST_A1                       44
#define ROC_ROBOT_DH_CONST_A2                       74
#define ROC_ROBOT_DH_CONST_A3                       83.24


#define ROC_ROBOT_INIT_ANGLE_THET_1                 60
#define ROC_ROBOT_INIT_ANGLE_THET_2                 0
#define ROC_ROBOT_INIT_ANGLE_THET_3                 60
#define ROC_ROBOT_INIT_ANGLE_BETA_1                 150
#define ROC_ROBOT_INIT_ANGLE_BETA_2                 90
#define ROC_ROBOT_INIT_ANGLE_BETA_3                 30


#define ROC_ROBOT_FIRST_STEP_ERROR                  0
#define ROC_ROBOT_SECND_STEP_ERROR                  4
#define ROC_ROBOT_LEFT_FIRST_STEP_ERROR             -2
#define ROC_ROBOT_LEFT_SECND_STEP_ERROR             0
#define ROC_ROBOT_RIGHT_FIRST_STEP_ERROR            -2
#define ROC_ROBOT_RIGHT_SECND_STEP_ERROR            0
#define ROC_ROBOT_PID_CONST_P                       1


#define ROC_ROBOT_LEG_WIDTH                         141.06
#define ROC_ROBOT_LEG_HEIGHT                        80
#define ROC_ROBOT_FEET_WIDTH                        23


#define ROC_ROBOT_INIT_DOWN_ANGLE                   0
#define ROC_ROBOT_ANGLE_TO_RADIAN                   (ROC_ROBOT_MATH_CONST_PI / ROC_SERVO_MAX_ROTATE_ANGLE)
#define ROC_ROBOT_ROTATE_ANGLE_TO_PWM               ((ROC_SERVO_MAX_PWM_VAL - ROC_SERVO_MIN_PWM_VAL) / ROC_SERVO_MAX_ROTATE_ANGLE)


#define ROC_ROBOT_WIDTH                             (ROC_ROBOT_DH_CONST_A1 + ROC_ROBOT_DH_CONST_A2 * cos(ROC_ROBOT_INIT_DOWN_ANGLE \
                                                    * ROC_ROBOT_ANGLE_TO_RADIAN) + ROC_ROBOT_FEET_WIDTH)
#define ROC_ROBOT_HEIGHT                            (ROC_ROBOT_DH_CONST_A2 * sin(ROC_ROBOT_INIT_DOWN_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN) \
                                                    + ROC_ROBOT_LEG_HEIGHT)


#define ROC_ROBOT_FRO_HIP_INIT_ANGLE                60
#define ROC_ROBOT_FRO_LEG_INIT_ANGLE                0
#define ROC_ROBOT_FRO_FET_INIT_ANGLE                73.29
#define ROC_ROBOT_MID_HIP_INIT_ANGLE                0
#define ROC_ROBOT_MID_LEG_INIT_ANGLE                0
#define ROC_ROBOT_MID_FET_INIT_ANGLE                73.29
#define ROC_ROBOT_HIN_HIP_INIT_ANGLE                60
#define ROC_ROBOT_HIN_LEG_INIT_ANGLE                0
#define ROC_ROBOT_HIN_FET_INIT_ANGLE                73.29


#define ROC_ROBOT_FRO_INIT_X                        (ROC_ROBOT_WIDTH * cos(ROC_ROBOT_FRO_HIP_INIT_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN))
#define ROC_ROBOT_FRO_INIT_Y                        (ROC_ROBOT_WIDTH * sin(ROC_ROBOT_FRO_HIP_INIT_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN))
#define ROC_ROBOT_FRO_INIT_Z                        -ROC_ROBOT_HEIGHT
#define ROC_ROBOT_MID_INIT_X                        (ROC_ROBOT_WIDTH * cos(ROC_ROBOT_MID_HIP_INIT_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN))
#define ROC_ROBOT_MID_INIT_Y                        (ROC_ROBOT_WIDTH * sin(ROC_ROBOT_MID_HIP_INIT_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN))
#define ROC_ROBOT_MID_INIT_Z                        -ROC_ROBOT_HEIGHT
#define ROC_ROBOT_HIN_INIT_X                        (ROC_ROBOT_WIDTH * cos(ROC_ROBOT_HIN_HIP_INIT_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN))
#define ROC_ROBOT_HIN_INIT_Y                        (ROC_ROBOT_WIDTH * sin(ROC_ROBOT_HIN_HIP_INIT_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN))
#define ROC_ROBOT_HIN_INIT_Z                        -ROC_ROBOT_HEIGHT



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
extern void RocOpenLoopMoveCalculate(double FirstStepLength, double SecndStepLength, uint16_t *RobotCtrlPwmVal);
extern void RocOpenLoopCircleCalculate(uint8_t DeltaAngle);


#endif


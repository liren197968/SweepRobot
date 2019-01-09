#include <math.h>
#include <stdint.h>

#include "RocLog.h"
#include "RocServo.h"
#include "RocRobotControl.h"
#include "RocRobotDhAlgorithm.h"


static double           g_DhAngleBuffer[3];

#ifdef ROC_ROBOT_CLOSED_LOOP_CONTROL
static double           g_FirstAngleError = 0;
static double           g_SecndAngleError = 0;
#endif

uint16_t                g_RobotOpenCtrlPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2] = {0};
uint16_t                g_RobotStandPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2] = {0};
uint16_t                g_RobotForwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2] = {0};
uint16_t                g_RobotBackwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2] = {0};
uint16_t                g_RobotCirclePwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2] = {0};
uint16_t                g_RobotLeftForwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2] = {0};
uint16_t                g_RobotLeftBackwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2] = {0};
uint16_t                g_RobotRightForwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2] = {0};
uint16_t                g_RobotRightBackwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2] = {0};


/*********************************************************************************
 *  Description:
 *              The reverse DH algorithm, which be used to caculate the three joint
 *              rotate angle of the hexapod robot leg.
 *
 *  Parameter:
 *              x: the expected X position of the robot leg tiptoe
 *              y: the expected Y position of the robot leg tiptoe
 *              z: the expected Z position of the robot leg tiptoe
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.16)
**********************************************************************************/
static void RocDhAlgorithmReverse(double x, double y, double z)
{
    double              a = 0;
    double              e = 0;
    double              f = 0;
    double              h = 0;
    double              j = 0;

    g_DhAngleBuffer[0] = (atan(y / x)) * 180 / ROC_ROBOT_MATH_CONST_PI;

    a = x * cos( (g_DhAngleBuffer[0] * ROC_ROBOT_MATH_CONST_PI) / 180) + y * sin( (g_DhAngleBuffer[0] * ROC_ROBOT_MATH_CONST_PI) / 180) - ROC_ROBOT_DH_CONST_A1;

    g_DhAngleBuffer[2] = ( (acos( ( pow( a, 2 ) + pow( ( z - ROC_ROBOT_DH_CONST_D1 ), 2) - pow(ROC_ROBOT_DH_CONST_A3, 2)
                                    - pow(ROC_ROBOT_DH_CONST_A2, 2) ) / ( 2 * ROC_ROBOT_DH_CONST_A2 * ROC_ROBOT_DH_CONST_A3 ) ) ) ) * 180 / ROC_ROBOT_MATH_CONST_PI;

    if( (g_DhAngleBuffer[2] > 0) || (g_DhAngleBuffer[2] < -180) )   // limit the anlge in the range of [0~(-180)]
    {
        if( (g_DhAngleBuffer[2] > 0) && (g_DhAngleBuffer[2] <= 180) )
        {
            g_DhAngleBuffer[2] = -g_DhAngleBuffer[2];
        }

        if( (g_DhAngleBuffer[2] > 180) && (g_DhAngleBuffer[2] <= 360) )
        {
            g_DhAngleBuffer[2] = g_DhAngleBuffer[2] - 360;
        }

        if( (g_DhAngleBuffer[2] < -180) && (g_DhAngleBuffer[2] >= -360) )
        {
            g_DhAngleBuffer[2] = -(g_DhAngleBuffer[2] + 360);
        }
    }

    e = cos(g_DhAngleBuffer[2] * ROC_ROBOT_ANGLE_TO_RADIAN);
    f = sin(g_DhAngleBuffer[2] * ROC_ROBOT_ANGLE_TO_RADIAN);
    h = (e * ROC_ROBOT_DH_CONST_A3 + ROC_ROBOT_DH_CONST_A2 ) * (z - ROC_ROBOT_DH_CONST_D1) - a * f * ROC_ROBOT_DH_CONST_A3;
    j = a * (ROC_ROBOT_DH_CONST_A3 * e + ROC_ROBOT_DH_CONST_A2) + ROC_ROBOT_DH_CONST_A3 * f * (z - ROC_ROBOT_DH_CONST_D1);

    g_DhAngleBuffer[1] = (atan2(h, j) ) * 180 / ROC_ROBOT_MATH_CONST_PI;
}

/*********************************************************************************
 *  Description:
 *              Caculate the servo PWM value when robot straight walking
 *
 *  Parameter:
 *              Step: the move step when robot straight walking
 *              Lift: the lift height of the legs when robot move
 *              pRobotCtrlPwmVal: the point of the result array data
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.16)
**********************************************************************************/
void RocRobotOpenLoopWalkCalculate(float Step, float Lift, uint16_t *pRobotCtrlPwmVal)
{
    double                  x = 0;
    double                  y = 0;
    double                  z = 0;

    /***********the coordinate datas of the first group legs ****************/
    x = ROC_ROBOT_FRO_INIT_X;
    y = ROC_ROBOT_FRO_INIT_Y + Step;
    z = ROC_ROBOT_FRO_INIT_Z + Lift;

    ROC_LOGW("x:%.2f, y:%.2f, z:%.2f", x, y, z);

    RocDhAlgorithmReverse(x, y, z);

    pRobotCtrlPwmVal[0] = (uint16_t)(ROC_ROBOT_RIG_FRO_HIP_CENTER + (ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[1] = (uint16_t)(ROC_ROBOT_RIG_FRO_LEG_CENTER + (ROC_ROBOT_FRO_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[2] = (uint16_t)(ROC_ROBOT_RIG_FRO_FET_CENTER + (ROC_ROBOT_FRO_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[24] = (uint16_t)(ROC_ROBOT_RIG_HIN_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[25] = (uint16_t)(ROC_ROBOT_RIG_HIN_LEG_CENTER + (ROC_ROBOT_HIN_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[26] = (uint16_t)(ROC_ROBOT_RIG_HIN_FET_CENTER + (ROC_ROBOT_HIN_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    x = ROC_ROBOT_MID_INIT_X;
    y = ROC_ROBOT_MID_INIT_Y + Step;
    z = ROC_ROBOT_MID_INIT_Z + Lift;

    RocDhAlgorithmReverse(x, y, z);

    pRobotCtrlPwmVal[12] = (uint16_t)(ROC_ROBOT_LEF_MID_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_MID_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[13] = (uint16_t)(ROC_ROBOT_LEF_MID_LEG_CENTER + (ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[14] = (uint16_t)(ROC_ROBOT_LEF_MID_FET_CENTER + (-ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[30] = (uint16_t)(ROC_ROBOT_LEF_MID_HIP_CENTER + (ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[31] = (uint16_t)(ROC_ROBOT_LEF_MID_LEG_CENTER + (ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[32] = (uint16_t)(ROC_ROBOT_LEF_MID_FET_CENTER + (-ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);

    x = ROC_ROBOT_HIN_INIT_X;
    y = ROC_ROBOT_HIN_INIT_Y - Step;
    z = ROC_ROBOT_HIN_INIT_Z + Lift;

    RocDhAlgorithmReverse(x, y, z);

    pRobotCtrlPwmVal[6] = (uint16_t)(ROC_ROBOT_RIG_HIN_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[7] = (uint16_t)(ROC_ROBOT_RIG_HIN_LEG_CENTER + (ROC_ROBOT_HIN_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[8] = (uint16_t)(ROC_ROBOT_RIG_HIN_FET_CENTER + (ROC_ROBOT_HIN_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[18] = (uint16_t)(ROC_ROBOT_RIG_FRO_HIP_CENTER + (ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[19] = (uint16_t)(ROC_ROBOT_RIG_FRO_LEG_CENTER + (ROC_ROBOT_FRO_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[20] = (uint16_t)(ROC_ROBOT_RIG_FRO_FET_CENTER + (ROC_ROBOT_FRO_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);

    /***********the coordinate datas of the second group legs ***************/
    x = ROC_ROBOT_FRO_INIT_X;
    y = ROC_ROBOT_FRO_INIT_Y + Step;
    z = ROC_ROBOT_FRO_INIT_Z + Lift;

    RocDhAlgorithmReverse(x, y, z);

    pRobotCtrlPwmVal[9] = (uint16_t)(ROC_ROBOT_LEF_FRO_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_FRO_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[10] = (uint16_t)(ROC_ROBOT_LEF_FRO_LEG_CENTER + (g_DhAngleBuffer[1] - ROC_ROBOT_FRO_LEG_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[11] = (uint16_t)(ROC_ROBOT_LEF_FRO_FET_CENTER + (-ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[33] = (uint16_t)(ROC_ROBOT_LEF_HIN_HIP_CENTER + (ROC_ROBOT_HIN_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[34] = (uint16_t)(ROC_ROBOT_LEF_HIN_LEG_CENTER + (g_DhAngleBuffer[1] - ROC_ROBOT_HIN_LEG_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[35] = (uint16_t)(ROC_ROBOT_LEF_HIN_FET_CENTER + (-ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);

    x = ROC_ROBOT_MID_INIT_X;
    y = ROC_ROBOT_MID_INIT_Y + Step;
    z = ROC_ROBOT_MID_INIT_Z + Lift;

    RocDhAlgorithmReverse(x, y, z);

    pRobotCtrlPwmVal[3] = (uint16_t)(ROC_ROBOT_RIG_MID_HIP_CENTER + (ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[4] = (uint16_t)(ROC_ROBOT_RIG_MID_LEG_CENTER + (-ROC_ROBOT_MID_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[5] = (uint16_t)(ROC_ROBOT_RIG_MID_FET_CENTER + (ROC_ROBOT_MID_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[21] = (uint16_t)(ROC_ROBOT_RIG_MID_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_MID_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[22] = (uint16_t)(ROC_ROBOT_RIG_MID_LEG_CENTER + (-ROC_ROBOT_MID_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[23] = (uint16_t)(ROC_ROBOT_RIG_MID_FET_CENTER + (ROC_ROBOT_MID_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);

    x = ROC_ROBOT_HIN_INIT_X;
    y = ROC_ROBOT_HIN_INIT_Y - Step;
    z = ROC_ROBOT_HIN_INIT_Z + Lift;

    RocDhAlgorithmReverse(x, y, z);

    pRobotCtrlPwmVal[15] = (uint16_t)(ROC_ROBOT_LEF_HIN_HIP_CENTER + (ROC_ROBOT_HIN_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[16] = (uint16_t)(ROC_ROBOT_LEF_HIN_LEG_CENTER + (g_DhAngleBuffer[1] - ROC_ROBOT_HIN_LEG_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[17] = (uint16_t)(ROC_ROBOT_LEF_HIN_FET_CENTER + (-ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[27] = (uint16_t)(ROC_ROBOT_LEF_FRO_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_FRO_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[28] = (uint16_t)(ROC_ROBOT_LEF_FRO_LEG_CENTER + (g_DhAngleBuffer[1] - ROC_ROBOT_HIN_LEG_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[29] = (uint16_t)(ROC_ROBOT_LEF_FRO_FET_CENTER + (-ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
}

/*********************************************************************************
 *  Description:
 *              Caculate the servo PWM value when robot circle walking
 *
 *  Parameter:
 *              DeltaAngle: the circle increment angle when robot circle move
 *              Lift: the lift height of the legs when robot move
 *              pRobotCtrlPwmVal: the point of the result array data
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.16)
**********************************************************************************/
void RocRobotOpenLoopCircleCalculate(float DeltaAngle, float Lift, uint16_t *pRobotCtrlPwmVal)
{
    double                  x = 0;
    double                  y = 0;
    double                  z = 0;

    /***********the shifting datas of the first group legs ******************/
    x = ROC_ROBOT_WIDTH * cos( (ROC_ROBOT_FRO_HIP_INIT_ANGLE + DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * sin( (ROC_ROBOT_FRO_HIP_INIT_ANGLE + DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_FRO_INIT_Z + Lift;

    ROC_LOGW("x:%.2f, y:%.2f, z:%.2f", x, y, z);

    RocDhAlgorithmReverse(x, y, z);

    pRobotCtrlPwmVal[0] = (uint16_t)(ROC_ROBOT_RIG_FRO_HIP_CENTER + (ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[1] = (uint16_t)(ROC_ROBOT_RIG_FRO_LEG_CENTER + (ROC_ROBOT_FRO_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[2] = (uint16_t)(ROC_ROBOT_RIG_FRO_FET_CENTER + (ROC_ROBOT_FRO_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[24] = (uint16_t)(ROC_ROBOT_RIG_HIN_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[25] = (uint16_t)(ROC_ROBOT_RIG_HIN_LEG_CENTER + (ROC_ROBOT_HIN_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[26] = (uint16_t)(ROC_ROBOT_RIG_HIN_FET_CENTER + (ROC_ROBOT_HIN_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    x = ROC_ROBOT_WIDTH * cos( (ROC_ROBOT_MID_HIP_INIT_ANGLE - DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * sin( (ROC_ROBOT_MID_HIP_INIT_ANGLE - DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_MID_INIT_Z + Lift;

    RocDhAlgorithmReverse(x, y, z);

    pRobotCtrlPwmVal[12] = (uint16_t)(ROC_ROBOT_LEF_MID_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_MID_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[13] = (uint16_t)(ROC_ROBOT_LEF_MID_LEG_CENTER + (ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[14] = (uint16_t)(ROC_ROBOT_LEF_MID_FET_CENTER + (-ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[30] = (uint16_t)(ROC_ROBOT_LEF_MID_HIP_CENTER + (ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[31] = (uint16_t)(ROC_ROBOT_LEF_MID_LEG_CENTER + (ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[32] = (uint16_t)(ROC_ROBOT_LEF_MID_FET_CENTER + (-ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    x = ROC_ROBOT_WIDTH * cos( (ROC_ROBOT_HIN_HIP_INIT_ANGLE - DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * sin( (ROC_ROBOT_HIN_HIP_INIT_ANGLE - DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_HIN_INIT_Z + Lift;

    RocDhAlgorithmReverse(x, y, z);

    pRobotCtrlPwmVal[6] = (uint16_t)(ROC_ROBOT_RIG_HIN_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[7] = (uint16_t)(ROC_ROBOT_RIG_HIN_LEG_CENTER + (ROC_ROBOT_HIN_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[8] = (uint16_t)(ROC_ROBOT_RIG_HIN_FET_CENTER + (ROC_ROBOT_HIN_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[18] = (uint16_t)(ROC_ROBOT_RIG_FRO_HIP_CENTER + (ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[19] = (uint16_t)(ROC_ROBOT_RIG_FRO_LEG_CENTER + (ROC_ROBOT_FRO_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[20] = (uint16_t)(ROC_ROBOT_RIG_FRO_FET_CENTER + (ROC_ROBOT_FRO_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    /***********the shifting datas of the second group legs *****************/
    x = ROC_ROBOT_WIDTH * cos( (ROC_ROBOT_FRO_HIP_INIT_ANGLE - DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * sin( (ROC_ROBOT_FRO_HIP_INIT_ANGLE - DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_FRO_INIT_Z + Lift;

    RocDhAlgorithmReverse(x, y, z);

    pRobotCtrlPwmVal[9] = (uint16_t)(ROC_ROBOT_LEF_FRO_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_FRO_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[10] = (uint16_t)(ROC_ROBOT_LEF_FRO_LEG_CENTER + (g_DhAngleBuffer[1] - ROC_ROBOT_FRO_LEG_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[11] = (uint16_t)(ROC_ROBOT_LEF_FRO_FET_CENTER + (-ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[33] = (uint16_t)(ROC_ROBOT_LEF_HIN_HIP_CENTER + (ROC_ROBOT_HIN_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[34] = (uint16_t)(ROC_ROBOT_LEF_HIN_LEG_CENTER + (g_DhAngleBuffer[1] - ROC_ROBOT_HIN_LEG_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[35] = (uint16_t)(ROC_ROBOT_LEF_HIN_FET_CENTER + (-ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    x = ROC_ROBOT_WIDTH * cos( (ROC_ROBOT_MID_HIP_INIT_ANGLE + DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * sin( (ROC_ROBOT_MID_HIP_INIT_ANGLE + DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_MID_INIT_Z + Lift;

    RocDhAlgorithmReverse(x, y, z);

    pRobotCtrlPwmVal[3] = (uint16_t)(ROC_ROBOT_RIG_MID_HIP_CENTER + (ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[4] = (uint16_t)(ROC_ROBOT_RIG_MID_LEG_CENTER + (-ROC_ROBOT_MID_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[5] = (uint16_t)(ROC_ROBOT_RIG_MID_FET_CENTER + (ROC_ROBOT_MID_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[21] = (uint16_t)(ROC_ROBOT_RIG_MID_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_MID_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[22] = (uint16_t)(ROC_ROBOT_RIG_MID_LEG_CENTER + (-ROC_ROBOT_MID_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[23] = (uint16_t)(ROC_ROBOT_RIG_MID_FET_CENTER + (ROC_ROBOT_MID_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);

    x = ROC_ROBOT_WIDTH * cos( (ROC_ROBOT_HIN_HIP_INIT_ANGLE + DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * sin( (ROC_ROBOT_HIN_HIP_INIT_ANGLE + DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_HIN_INIT_Z + Lift;

    RocDhAlgorithmReverse(x, y, z);

    pRobotCtrlPwmVal[15] = (uint16_t)(ROC_ROBOT_LEF_HIN_HIP_CENTER + (ROC_ROBOT_HIN_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[16] = (uint16_t)(ROC_ROBOT_LEF_HIN_LEG_CENTER + (g_DhAngleBuffer[1] - ROC_ROBOT_HIN_LEG_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[17] = (uint16_t)(ROC_ROBOT_LEF_HIN_FET_CENTER + (-ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[27] = (uint16_t)(ROC_ROBOT_LEF_FRO_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_FRO_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[28] = (uint16_t)(ROC_ROBOT_LEF_FRO_LEG_CENTER + (g_DhAngleBuffer[1] - ROC_ROBOT_HIN_LEG_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[29] = (uint16_t)(ROC_ROBOT_LEF_FRO_FET_CENTER + (-ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
}

#ifdef ROC_ROBOT_CLOSED_LOOP_CONTROL
/*********************************************************************************
 *  Description:
 *              Correct the robot tiptoe position when use the closed loop control
 *              with the IMU feedback angle.
 *
 *  Parameter:
 *              LegNum: the number of the robot leg which needs correciton
 *              DeltaAlpha: the difference angle measured by IMU sensor
 *              x: the point of the correction X position result
 *              y: the point of the correction Y position result
 *              z: the point of the correction Z position result
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.16)
**********************************************************************************/
static void RocCorrectionPositionCaculate(uint8_t LegNum, double DeltaAlpha, double *x, double *y, double *z)
{
    static double           l = 0, r = 0;
    static double           c1 = 0, alfa1 = 0;
    static double           c2 = 0, alfa2 = 0;
    static double           c3 = 0, alfa3 = 0;
    static double           c4 = 0, alfa4 = 0;
    static double           c5 = 0, alfa5 = 0;
    static double           c6 = 0, alfa6 = 0;
    static double           gamma1 = 0, gamma2 = 0, gamma3 = 0;

    if(LegNum == 1)
    {
        r = ROC_ROBOT_WIDTH;
        l = ROC_ROBOT_DEFAULT_STEP_LENGTH;

        gamma1 = ROC_ROBOT_INIT_ANGLE_BETA_1 - DeltaAlpha;
        gamma2 = ROC_ROBOT_INIT_ANGLE_BETA_2 - DeltaAlpha;
        gamma3 = ROC_ROBOT_INIT_ANGLE_BETA_3 - DeltaAlpha;

        c1 = sqrt( r * r - 2 * l * r * cos(gamma1 * ROC_ROBOT_ANGLE_TO_RADIAN) + l * l);
        c2 = sqrt( r * r - 2 * l * r * cos(gamma2 * ROC_ROBOT_ANGLE_TO_RADIAN) + l * l);
        c3 = sqrt( r * r - 2 * l * r * cos(gamma3 * ROC_ROBOT_ANGLE_TO_RADIAN) + l * l);
        c4 = sqrt( r * r - 2 * l * r * cos( (180 - gamma1) * ROC_ROBOT_ANGLE_TO_RADIAN ) + l * l);
        c5 = sqrt( r * r - 2 * l * r * cos( (180 - gamma2) * ROC_ROBOT_ANGLE_TO_RADIAN ) + l * l);
        c6 = sqrt( r * r - 2 * l * r * cos( (180 - gamma3) * ROC_ROBOT_ANGLE_TO_RADIAN ) + l * l);

        alfa1 = acos( (c1 * c1 + r * r - l * l) / (2 * c1 * r));
        alfa2 = acos( (c2 * c2 + r * r - l * l) / (2 * c2 * r));
        alfa3 = acos( (c3 * c3 + r * r - l * l) / (2 * c3 * r));
        alfa4 = acos( (c4 * c4 + r * r - l * l) / (2 * c4 * r));
        alfa5 = acos( (c5 * c5 + r * r - l * l) / (2 * c5 * r));
        alfa6 = acos( (c6 * c6 + r * r - l * l) / (2 * c6 * r));
    }

    if(ROC_ROBOT_RIGHT_FRONT_LEG == LegNum)
    {
        *x = c1 * cos(ROC_ROBOT_INIT_ANGLE_THET_1 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN + alfa1);
        *y = c1 * sin(ROC_ROBOT_INIT_ANGLE_THET_1 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN + alfa1);
    }
    else if(ROC_ROBOT_RIGHT_MIDDLE_LEG == LegNum)
    {
        *x = c2 * cos(ROC_ROBOT_INIT_ANGLE_THET_2 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN + alfa2);
        *y = c2 * sin(ROC_ROBOT_INIT_ANGLE_THET_2 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN + alfa2);
    }
    else if(ROC_ROBOT_RIGHT_HIND_LEG == LegNum)
    {
        *x = c3 * cos(ROC_ROBOT_INIT_ANGLE_THET_3 * ROC_ROBOT_ANGLE_TO_RADIAN - DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN - alfa3);
        *y = c3 * sin(ROC_ROBOT_INIT_ANGLE_THET_3 * ROC_ROBOT_ANGLE_TO_RADIAN - DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN - alfa3);
    }
    else if(ROC_ROBOT_LEFT_FRONT_LEG == LegNum)
    {
        *x = c4 * cos(ROC_ROBOT_INIT_ANGLE_THET_1 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN - alfa4);
        *y = c4 * sin(ROC_ROBOT_INIT_ANGLE_THET_1 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN - alfa4);
    }
    else if(ROC_ROBOT_LEFT_MIDDLE_LEG == LegNum)
    {
        *x = c5 * cos(ROC_ROBOT_INIT_ANGLE_THET_2 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN - alfa5);
        *y = c5 * sin(ROC_ROBOT_INIT_ANGLE_THET_2 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN - alfa5);
    }
    else if(ROC_ROBOT_LEFT_HIND_LEG == LegNum)
    {
        *x = c6 * cos(ROC_ROBOT_INIT_ANGLE_THET_3 * ROC_ROBOT_ANGLE_TO_RADIAN - DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN + alfa6);
        *y = c6 * sin(ROC_ROBOT_INIT_ANGLE_THET_3 * ROC_ROBOT_ANGLE_TO_RADIAN - DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN + alfa6);
    }

    *z = -ROC_ROBOT_HEIGHT;
}

/*********************************************************************************
 *  Description:
 *              Adjust the robot walk step when using closed loop control.
 *
 *  Parameter:
 *              ExpectedAngle: the expected anlge value when robot straight walking
 *              Direction: the direction of robot move
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.16)
**********************************************************************************/
void RocRobotAdjustRobotStepCalculate(double ExpectedAngle, uint8_t Direction)
{
    double      CurrentAngle = 0;
    double      x = 0, y = 0, z = 0;

    CurrentAngle = g_ImuYawAngle;
    /********************************For the forward direction*************************************************/
    if(Direction == 0)
    {
        g_FirstAngleError = (CurrentAngle - ExpectedAngle) * ROC_ROBOT_PID_CONST_P;
        g_SecndAngleError = (ExpectedAngle - CurrentAngle) * ROC_ROBOT_PID_CONST_P + ROC_ROBOT_FIRST_STEP_ERROR;

        if(g_FirstAngleError >= 15)
        {
            g_FirstAngleError = 15;
        }
        else
        {
            if(g_FirstAngleError < -15)
            {
                g_FirstAngleError = -15;
            }
        }

        if(g_SecndAngleError >= 15)
        {
            g_SecndAngleError = 15;
        }
        else
        {
            if(g_SecndAngleError < -15)
            {
                g_SecndAngleError = -15;
            }
        }
        /*********************************** the first group legs *******************************/
        RocCorrectionPositionCaculate(1, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotForwardPwmVal[0] = (uint16_t)((ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[1] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[2] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(2, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotForwardPwmVal[12] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[13] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[14] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(3, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotForwardPwmVal[6] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[7] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[8] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);

        /************************************* the second group legs ************************************/
        RocCorrectionPositionCaculate(1, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotForwardPwmVal[9] = (uint16_t)((ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[10] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[11] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(2, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotForwardPwmVal[3] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[4] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[5] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(3, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotForwardPwmVal[15] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[16] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[17] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        /*********************************** the first group legs *************************************/
        RocCorrectionPositionCaculate(4, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotForwardPwmVal[18] = (uint16_t)((ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[19] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[20] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(5, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotForwardPwmVal[30] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[31] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[32] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(6, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotForwardPwmVal[24] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[25] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[26] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);

        /******************************** the second group legs ******************************/
        RocCorrectionPositionCaculate(4, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotForwardPwmVal[27] = (uint16_t)((ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[28] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[29] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(5, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotForwardPwmVal[21] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[22] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[23] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(6, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotForwardPwmVal[33] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[34] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotForwardPwmVal[35] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    }

    /******************************** For the backward direction ************************/
    if(Direction == 1)
    {
        g_FirstAngleError = (CurrentAngle - g_ExpectedAngle) * ROC_ROBOT_PID_CONST_P;
        g_SecndAngleError = (g_ExpectedAngle - CurrentAngle) * ROC_ROBOT_PID_CONST_P + ROC_ROBOT_FIRST_STEP_ERROR;

        if(g_FirstAngleError >= 15)
        {
            g_FirstAngleError = 15;
        }
        else
        {
            if(g_FirstAngleError < -15)
            {
                g_FirstAngleError = -15;
            }
        }

        if(g_SecndAngleError >= 15)
        {
            g_SecndAngleError = 15;
        }
        else
        {
            if(g_SecndAngleError < -15)
            {
                g_SecndAngleError = -15;
            }
        }
        /*********************************** the first group legs *************************************/
        RocCorrectionPositionCaculate(3, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotBackwardPwmVal[0] = (uint16_t)((ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[1] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[2] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(5, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotBackwardPwmVal[12] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[13] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[14] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(1, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotBackwardPwmVal[6] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[7] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[8] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);

        /********************************* the second group legs ****************************************/
        RocCorrectionPositionCaculate(3, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotBackwardPwmVal[9] = (uint16_t)((ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[10] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[11] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);

        RocCorrectionPositionCaculate(5, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotBackwardPwmVal[3] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[4] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[5] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(1, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotBackwardPwmVal[15] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[16] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[17] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);

        /******************************** the first group legs *****************************************/
        RocCorrectionPositionCaculate(6, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotBackwardPwmVal[18] = (uint16_t)((ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[19] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[20] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(2, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotBackwardPwmVal[30] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[31] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[32] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(4, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotBackwardPwmVal[24] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[25] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[26] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);

        /******************************** the second group legs ******************************************/
        RocCorrectionPositionCaculate(6, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotBackwardPwmVal[27] = (uint16_t)((ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[28] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[29] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(2, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotBackwardPwmVal[21] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[22] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[23] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(4, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotBackwardPwmVal[33] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[34] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotBackwardPwmVal[35] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    }
}
#endif


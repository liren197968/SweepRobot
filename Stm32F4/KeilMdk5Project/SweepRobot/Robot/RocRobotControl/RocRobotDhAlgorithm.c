#include <math.h>
#include <stdint.h>

#include "RocLog.h"
#include "RocServo.h"
#include "RocRobotControl.h"
#include "RocRobotDhAlgorithm.h"


static double           g_DhAngleBuffer[3];

static double           g_FirstAngleError = 0;
static double           g_SecndAngleError = 0;

uint16_t                g_RobotOpenCtrlPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2] = {0};
uint16_t                g_RobotStandPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2] = {0};
uint16_t                g_RobotForwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2] = {0};
uint16_t                g_RobotBackwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2] = {0};
uint16_t                g_RobotCirclePwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2] = {0};
uint16_t                g_RobotLeftForwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2] = {0};
uint16_t                g_RobotLeftBackwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2] = {0};
uint16_t                g_RobotRightForwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2] = {0};
uint16_t                g_RobotRightBackwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2] = {0};
/*************************D-H参数法反解计算函数*************************/
static void RocDhAlgorithmReverse(double x, double y, double z)
{
    double                      a = 0, e = 0, f = 0, h = 0, j = 0;

    g_DhAngleBuffer[0] = (atan(y / x)) * 180 / 3.141593;

    a = x * cos( (g_DhAngleBuffer[0] * 3.141593) / 180) + y * sin( (g_DhAngleBuffer[0] * 3.141593) / 180) - ROC_ROBOT_DH_CONST_A1;

    g_DhAngleBuffer[2] = ( (acos( ( pow( a, 2 ) + pow ( ( z - ROC_ROBOT_DH_CONST_D1 ), 2) - pow (ROC_ROBOT_DH_CONST_A3, 2)
                                    - pow(ROC_ROBOT_DH_CONST_A2, 2) ) / ( 2 * ROC_ROBOT_DH_CONST_A2 * ROC_ROBOT_DH_CONST_A3 ) ) ) ) * 180 / 3.141593;

    if( (g_DhAngleBuffer[2] > 0) || (g_DhAngleBuffer[2] < -180) )   //多解的转换到预定范围内 [0~(-180)]
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

    g_DhAngleBuffer[1] = (atan2(h, j) ) * 180 / 3.141593;
}
/*********************************************************************/
/************************计算补偿修正后的坐标值*************************/
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
/**********************************************************************/
/***************************开环运动控制********************************/
void RocOpenLoopMoveCalculate(double Step, double Left, uint16_t *pRobotCtrlPwmVal)
{
    double                  x = 0, y = 0, z = 0;

    /***********the coordinate datas of the first group legs ****************/
    x = ROC_ROBOT_FRO_INIT_X;
    y = ROC_ROBOT_FRO_INIT_Y + Step;
    z = ROC_ROBOT_FRO_INIT_Z + Left;

    ROC_LOGW("x:%.2f, y:%.2f, z:%.2f, data:%.4f", x, y, z, ROC_ROBOT_FRO_FET_INIT_ANGLE);

    RocDhAlgorithmReverse(x, y, z);

    pRobotCtrlPwmVal[0] = (uint16_t)(ROC_ROBOT_RIG_FRO_HIP_CENTER + (ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[1] = (uint16_t)(ROC_ROBOT_RIG_FRO_LEG_CENTER + (ROC_ROBOT_FRO_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[2] = (uint16_t)(ROC_ROBOT_RIG_FRO_FET_CENTER + (ROC_ROBOT_FRO_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[24] = (uint16_t)(ROC_ROBOT_RIG_HIN_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[25] = (uint16_t)(ROC_ROBOT_RIG_HIN_LEG_CENTER + (ROC_ROBOT_HIN_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[26] = (uint16_t)(ROC_ROBOT_RIG_HIN_FET_CENTER + (ROC_ROBOT_HIN_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    x = ROC_ROBOT_MID_INIT_X;
    y = ROC_ROBOT_MID_INIT_Y + Step;
    z = ROC_ROBOT_MID_INIT_Z + Left;

    RocDhAlgorithmReverse(x, y, z);

    pRobotCtrlPwmVal[12] = (uint16_t)(ROC_ROBOT_LEF_MID_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_MID_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[13] = (uint16_t)(ROC_ROBOT_LEF_MID_LEG_CENTER + (ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[14] = (uint16_t)(ROC_ROBOT_LEF_MID_FET_CENTER + (-ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[30] = (uint16_t)(ROC_ROBOT_LEF_MID_HIP_CENTER + (ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[31] = (uint16_t)(ROC_ROBOT_LEF_MID_LEG_CENTER + (ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[32] = (uint16_t)(ROC_ROBOT_LEF_MID_FET_CENTER + (-ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);

    x = ROC_ROBOT_HIN_INIT_X;
    y = ROC_ROBOT_HIN_INIT_Y - Step;
    z = ROC_ROBOT_HIN_INIT_Z + Left;

    RocDhAlgorithmReverse(x, y, z);

    pRobotCtrlPwmVal[6] = (uint16_t)(ROC_ROBOT_RIG_HIN_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[7] = (uint16_t)(ROC_ROBOT_RIG_HIN_LEG_CENTER + (ROC_ROBOT_HIN_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[8] = (uint16_t)(ROC_ROBOT_RIG_HIN_FET_CENTER + (ROC_ROBOT_HIN_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[18] = (uint16_t)(ROC_ROBOT_RIG_FRO_HIP_CENTER + (ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[19] = (uint16_t)(ROC_ROBOT_RIG_FRO_LEG_CENTER + (ROC_ROBOT_FRO_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[20] = (uint16_t)(ROC_ROBOT_RIG_FRO_FET_CENTER + (ROC_ROBOT_FRO_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    /************************************************************************/

    /***********the coordinate datas of the second group legs ***************/
    x = ROC_ROBOT_FRO_INIT_X;
    y = ROC_ROBOT_FRO_INIT_Y + Step;
    z = ROC_ROBOT_FRO_INIT_Z + Left;

    RocDhAlgorithmReverse(x, y, z);

    pRobotCtrlPwmVal[9] = (uint16_t)(ROC_ROBOT_LEF_FRO_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_FRO_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[10] = (uint16_t)(ROC_ROBOT_LEF_FRO_LEG_CENTER + (g_DhAngleBuffer[1] - ROC_ROBOT_FRO_LEG_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[11] = (uint16_t)(ROC_ROBOT_LEF_FRO_FET_CENTER + (-ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[33] = (uint16_t)(ROC_ROBOT_LEF_HIN_HIP_CENTER + (ROC_ROBOT_HIN_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[34] = (uint16_t)(ROC_ROBOT_LEF_HIN_LEG_CENTER + (g_DhAngleBuffer[1] - ROC_ROBOT_HIN_LEG_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[35] = (uint16_t)(ROC_ROBOT_LEF_HIN_FET_CENTER + (-ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);

    x = ROC_ROBOT_MID_INIT_X;
    y = ROC_ROBOT_MID_INIT_Y + Step;
    z = ROC_ROBOT_MID_INIT_Z + Left;

    RocDhAlgorithmReverse(x, y, z);

    pRobotCtrlPwmVal[3] = (uint16_t)(ROC_ROBOT_RIG_MID_HIP_CENTER + (ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[4] = (uint16_t)(ROC_ROBOT_RIG_MID_LEG_CENTER + (-ROC_ROBOT_MID_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[5] = (uint16_t)(ROC_ROBOT_RIG_MID_FET_CENTER + (ROC_ROBOT_MID_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[21] = (uint16_t)(ROC_ROBOT_RIG_MID_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_MID_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[22] = (uint16_t)(ROC_ROBOT_RIG_MID_LEG_CENTER + (-ROC_ROBOT_MID_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[23] = (uint16_t)(ROC_ROBOT_RIG_MID_FET_CENTER + (ROC_ROBOT_MID_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);

    x = ROC_ROBOT_HIN_INIT_X;
    y = ROC_ROBOT_HIN_INIT_Y - Step;
    z = ROC_ROBOT_HIN_INIT_Z + Left;

    RocDhAlgorithmReverse(x, y, z);

    pRobotCtrlPwmVal[15] = (uint16_t)(ROC_ROBOT_LEF_HIN_HIP_CENTER + (ROC_ROBOT_HIN_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[16] = (uint16_t)(ROC_ROBOT_LEF_HIN_LEG_CENTER + (g_DhAngleBuffer[1] - ROC_ROBOT_HIN_LEG_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[17] = (uint16_t)(ROC_ROBOT_LEF_HIN_FET_CENTER + (-ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[27] = (uint16_t)(ROC_ROBOT_LEF_FRO_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_FRO_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[28] = (uint16_t)(ROC_ROBOT_LEF_FRO_LEG_CENTER + (g_DhAngleBuffer[1] - ROC_ROBOT_HIN_LEG_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotCtrlPwmVal[29] = (uint16_t)(ROC_ROBOT_LEF_FRO_FET_CENTER + (-ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    /***********************************************************************/
}
/**********************************************************************/
/***********************转圈运动计算函数********************************/
void RocOpenLoopCircleCalculate(uint8_t DeltaAngle)
{
    double          x = 0, y = 0, z = 0;

    /***********the shifting datas of the first group legs ******************/
    x = ROC_ROBOT_WIDTH * cos( (ROC_ROBOT_FRO_HIP_INIT_ANGLE + DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * sin( (ROC_ROBOT_FRO_HIP_INIT_ANGLE + DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_FRO_INIT_Z;

    RocDhAlgorithmReverse(x, y, z);

    g_RobotCirclePwmVal[0] = (uint16_t)((ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[1] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[2] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[24] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[25] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[26] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    x = ROC_ROBOT_WIDTH * cos( (ROC_ROBOT_MID_HIP_INIT_ANGLE - DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * sin( (ROC_ROBOT_MID_HIP_INIT_ANGLE - DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_MID_INIT_Z;

    RocDhAlgorithmReverse(x, y, z);

    g_RobotCirclePwmVal[12] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[13] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[14] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[30] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_MID_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[31] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[32] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    x = ROC_ROBOT_WIDTH * cos( (ROC_ROBOT_HIN_HIP_INIT_ANGLE - DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * sin( (ROC_ROBOT_HIN_HIP_INIT_ANGLE - DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_HIN_INIT_Z;

    RocDhAlgorithmReverse(x, y, z);

    g_RobotCirclePwmVal[6] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[7] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[8] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[18] = (uint16_t)((ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[19] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[20] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    /************************************************************************/

    /***********the shifting datas of the second group legs *****************/
    x = ROC_ROBOT_WIDTH * cos( (ROC_ROBOT_FRO_HIP_INIT_ANGLE - DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * sin( (ROC_ROBOT_FRO_HIP_INIT_ANGLE - DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_FRO_INIT_Z;

    RocDhAlgorithmReverse(x, y, z);

    g_RobotCirclePwmVal[9] = (uint16_t)((ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[10] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[11] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[33] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[34] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[35] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    x = ROC_ROBOT_WIDTH * cos( (ROC_ROBOT_MID_HIP_INIT_ANGLE + DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * sin( (ROC_ROBOT_MID_HIP_INIT_ANGLE + DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_MID_INIT_Z;

    RocDhAlgorithmReverse(x, y, z);

    g_RobotCirclePwmVal[3] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[4] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[5] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[21] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_MID_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[22] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[23] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);

    x = ROC_ROBOT_WIDTH * cos( (ROC_ROBOT_HIN_HIP_INIT_ANGLE + DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * sin( (ROC_ROBOT_HIN_HIP_INIT_ANGLE + DeltaAngle) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_HIN_INIT_Z;

    RocDhAlgorithmReverse(x, y, z);

    g_RobotCirclePwmVal[15] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[16] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[17] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[27] = (uint16_t)((ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[28] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    g_RobotCirclePwmVal[29] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
}
/**********************************************************************/
/*************************步长修正计算函数******************************/
void RocAdjustStepCalculate(double g_ExpectedAngle, uint8_t Direction)
{
    double      CurrentAngle = 0;
    double      x = 0, y = 0, z = 0;

    //CurrentAngle = yaw;
    /********************************前进功能***************************************************************/
    if(Direction == 0)
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
        /*****************************************第一组腿***********************************************/
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
        /*****************************************第二组腿***********************************************/
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

        /*****************************************第一组腿************操作与前一组操作相同***************/
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
        /*****************************************第二组腿***********************************************/
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
    /********************************后退功能*****************后退操作与前进操作相同************************/
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
        /*****************************************第一组腿***********************************************/
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
        /*****************************************第二组腿***********************************************/
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

        /*****************************************第一组腿************操作与前一组操作相同***************/
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
        /*****************************************第二组腿***********************************************/
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
/**********************************************************************/
/************************右步长调整函数*********************************/
void RocAdjustRightStepCalculate(double g_ExpectedAngle, uint8_t Direction)
{
    double      CurrentAngle = 0;
    double      x = 0, y = 0, z = 0;

    //CurrentAngle = yaw;
    /********************************前进功能**************************************************************/
    if(Direction == 0)
    {
        g_FirstAngleError = (CurrentAngle - g_ExpectedAngle) * ROC_ROBOT_PID_CONST_P;
        g_SecndAngleError = (g_ExpectedAngle - CurrentAngle) * ROC_ROBOT_PID_CONST_P + ROC_ROBOT_LEFT_FIRST_STEP_ERROR;

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
        /*****************************************第一组腿***********************************************/
        RocCorrectionPositionCaculate(1, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftForwardPwmVal[9] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_FRO_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[10] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[11] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(2, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftForwardPwmVal[15] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[16] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[17] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(3, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftForwardPwmVal[3] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[4] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[5] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        RocCorrectionPositionCaculate(1, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftForwardPwmVal[12] = (uint16_t)((ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[13] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[14] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(2, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftForwardPwmVal[0] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[1] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[2] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(3, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftForwardPwmVal[6] = (uint16_t)((ROC_ROBOT_HIN_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[7] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[8] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);

        /*****************************************第一组腿**********************操作与前一组操作相同*****/
        RocCorrectionPositionCaculate(4, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftForwardPwmVal[27] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_FRO_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[28] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[29] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(5, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftForwardPwmVal[33] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[34] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[35] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(6, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftForwardPwmVal[21] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[22] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[23] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        RocCorrectionPositionCaculate(4, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftForwardPwmVal[30] = (uint16_t)((ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[31] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[32] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(5, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftForwardPwmVal[18] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[19] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[20] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(6, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftForwardPwmVal[24] = (uint16_t)((ROC_ROBOT_HIN_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[25] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftForwardPwmVal[26] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    }
    /********************************后退功能*****************后退操作与前进操作相同**************************/
    if(Direction == 1)
    {
        g_FirstAngleError = (CurrentAngle - g_ExpectedAngle) * ROC_ROBOT_PID_CONST_P;
        g_SecndAngleError = (g_ExpectedAngle - CurrentAngle) * ROC_ROBOT_PID_CONST_P + ROC_ROBOT_LEFT_SECND_STEP_ERROR;

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

        if(g_SecndAngleError >= 20)
        {
            g_SecndAngleError = 20;
        }
        else
        {
            if(g_SecndAngleError < -20)
            {
                g_SecndAngleError = -20;
            }
        }
        /*****************************************第一组腿***********************************************/
        RocCorrectionPositionCaculate(3, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftBackwardPwmVal[9] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_FRO_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[10] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[11] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(5, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftBackwardPwmVal[15] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[16] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[17] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(1, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftBackwardPwmVal[3] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[4] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[5] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        RocCorrectionPositionCaculate(3, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftBackwardPwmVal[12] = (uint16_t)((ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[13] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[14] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(5, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftBackwardPwmVal[0] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[1] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[2] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(1, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftBackwardPwmVal[6] = (uint16_t)((ROC_ROBOT_HIN_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[7] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[8] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);

        /*****************************************第一组腿**********************操作与前一组操作相同*****/
        RocCorrectionPositionCaculate(6, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftBackwardPwmVal[27] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_FRO_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[28] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[29] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(2, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftBackwardPwmVal[33] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[34] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[35] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(4, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftBackwardPwmVal[21] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[22] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[23] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        RocCorrectionPositionCaculate(6, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftBackwardPwmVal[30] = (uint16_t)((ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[31] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[32] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(2, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftBackwardPwmVal[18] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[19] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[20] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(4, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotLeftBackwardPwmVal[24] = (uint16_t)((ROC_ROBOT_HIN_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[25] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotLeftBackwardPwmVal[26] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    }
}
/**********************************************************************/
/***********************左步长调整函数**********************************/
void RocAdjustLeftStepCalculate(double g_ExpectedAngle, uint8_t Direction)
{
    double      CurrentAngle = 0;
    double      x = 0, y = 0, z = 0;

    //CurrentAngle = yaw;
    /********************************前进功能***************************************************************/
    if(Direction == 0)
    {
        g_FirstAngleError = (CurrentAngle - g_ExpectedAngle) * ROC_ROBOT_PID_CONST_P;
        g_SecndAngleError = (g_ExpectedAngle - CurrentAngle) * ROC_ROBOT_PID_CONST_P + ROC_ROBOT_RIGHT_FIRST_STEP_ERROR;

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
        /*****************************************第一组腿***********************************************/
        RocCorrectionPositionCaculate(1, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightForwardPwmVal[3] = (uint16_t)((ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[4] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[5] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(2, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightForwardPwmVal[9] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[10] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[11] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(3, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightForwardPwmVal[15] = (uint16_t)((ROC_ROBOT_HIN_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[16] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[17] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        RocCorrectionPositionCaculate(1, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightForwardPwmVal[0] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_FRO_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[1] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[2] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(2, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightForwardPwmVal[6] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[7] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[8] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(3, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightForwardPwmVal[12] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[13] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[14] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        /*****************************************第一组腿**********************操作与前一组操作相同*****/
        RocCorrectionPositionCaculate(4, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightForwardPwmVal[21] = (uint16_t)((ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[22] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[23] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(5, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightForwardPwmVal[27] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[28] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[29] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(6, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightForwardPwmVal[33] = (uint16_t)((ROC_ROBOT_HIN_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[34] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[35] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        RocCorrectionPositionCaculate(4, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightForwardPwmVal[18] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_FRO_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[19] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[20] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(5, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightForwardPwmVal[24] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[25] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[26] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(6, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightForwardPwmVal[30] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[31] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightForwardPwmVal[32] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    }
    /********************************后退功能*****************后退操作与前进操作相同************************/
    if(Direction == 1)
    {
        g_FirstAngleError = (CurrentAngle - g_ExpectedAngle) * ROC_ROBOT_PID_CONST_P + ROC_ROBOT_RIGHT_SECND_STEP_ERROR;
        g_SecndAngleError = (g_ExpectedAngle - CurrentAngle) * ROC_ROBOT_PID_CONST_P;

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
        /*****************************************第一组腿***********************************************/
        RocCorrectionPositionCaculate(3, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightBackwardPwmVal[3] = (uint16_t)((ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[4] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[5] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(5, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightBackwardPwmVal[9] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[10] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[11] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(1, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightBackwardPwmVal[15] = (uint16_t)((ROC_ROBOT_HIN_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[16] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[17] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        RocCorrectionPositionCaculate(3, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightBackwardPwmVal[0] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_FRO_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[1] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[2] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(5, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightBackwardPwmVal[6] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[7] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[8] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(1, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightBackwardPwmVal[12] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[13] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[14] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        /*****************************************第一组腿**********************操作与前一组操作相同*****/
        RocCorrectionPositionCaculate(6, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightBackwardPwmVal[21] = (uint16_t)((ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[22] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[23] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(2, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightBackwardPwmVal[27] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[28] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[29] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(4, g_SecndAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightBackwardPwmVal[33] = (uint16_t)((ROC_ROBOT_HIN_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[34] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[35] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        RocCorrectionPositionCaculate(6, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightBackwardPwmVal[18] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_FRO_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[19] = (uint16_t)((ROC_ROBOT_FRO_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[20] = (uint16_t)((ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(2, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightBackwardPwmVal[24] = (uint16_t)((ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[25] = (uint16_t)((ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[26] = (uint16_t)((ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


        RocCorrectionPositionCaculate(4, g_FirstAngleError, &x, &y, &z);
        RocDhAlgorithmReverse(x, y, z);

        g_RobotRightBackwardPwmVal[30] = (uint16_t)((g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[31] = (uint16_t)((ROC_ROBOT_HIN_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
        g_RobotRightBackwardPwmVal[32] = (uint16_t)((ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    }
}


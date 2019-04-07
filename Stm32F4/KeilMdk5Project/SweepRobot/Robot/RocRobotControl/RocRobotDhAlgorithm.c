/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2018/12/16      1.0
********************************************************************************/
#include <stdint.h>

#include "RocLog.h"
#include "RocRobotDhAlgorithm.h"


static double           g_DhAngleBuffer[3];

#ifdef ROC_ROBOT_CLOSED_LOOP_CONTROL
static double           g_FirstAngleError = 0;
static double           g_SecndAngleError = 0;
#endif


uint16_t                g_RobotStandPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2] = {0};
uint16_t                g_RobotForwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2] = {0};
uint16_t                g_RobotBackwardPwmVal[ROC_SERVO_MAX_SUPPORT_NUM * 2] = {0};


#ifdef ROC_ROBOT_GAIT_QUADMODE
static ROC_PHOENIX_GAIT_s g_RobotGait[] =
{
    {ROC_ROBOT_RUN_SPEED_DEFAULT, 8, 2, 1, 2, 6, 1, 0, 0,0, 1, {7, 1, 3, 5}},   // ripple
    {ROC_ROBOT_RUN_SPEED_DEFAULT, 12, 2, 1, 2, 10, 1, 0, 0,0, 1, {7, 1, 4, 10}},   // ripple
    {ROC_ROBOT_RUN_SPEED_DEFAULT, 4, 2, 1, 2, 2, 1, 0, 0, 0, 1,{3, 1, 1, 3}},  // Amble
    {ROC_ROBOT_RUN_SPEED_DEFAULT, 6, 3, 2, 2, 3, 2, 0, 0,0, 1, {1, 4, 4, 1}} }; // Smooth Amble 
};
#else
static ROC_PHOENIX_GAIT_s g_RobotGait[] =
{
    [ROC_ROBOT_GAIT_RIPPLE_12]  =   {ROC_ROBOT_RUN_SPEED_DEFAULT,   12, 3,  2,  2,  8,  3, {7,  11, 3,  1,  5,  9}, "Ripple 12"},   // Ripple 12
    [ROC_ROBOT_GAIT_TRIPOD_8]   =   {ROC_ROBOT_RUN_SPEED_DEFAULT,   8,  3,  2,  2,  4,  3, {1,  5,  1,  5,  1,  5}, "Tripod 8"},    // Tripod 8 steps
    [ROC_ROBOT_GAIT_TRIPLE_12]  =   {ROC_ROBOT_RUN_SPEED_DEFAULT,   12, 3,  2,  2,  8,  3, {5,  10, 3,  11, 4,  9}, "Tripple 12"},  // Triple Tripod 12 step
    [ROC_ROBOT_GAIT_TRIPLE_16]  =   {ROC_ROBOT_RUN_SPEED_DEFAULT,   16, 5,  3,  4,  10, 1, {6,  13, 4,  14, 5,  12},"Tripple 16"},  // Triple Tripod 16 steps, use 5 lifted positions
    [ROC_ROBOT_GAIT_WAVE_24]    =   {ROC_ROBOT_RUN_SPEED_DEFAULT,   24, 3,  2,  2,  20, 3, {13, 17, 21, 1,  5,  9}, "Wave 24"},     // Wave 24 steps
    [ROC_ROBOT_GAIT_TRIPOD_6]   =   {ROC_ROBOT_RUN_SPEED_DEFAULT,   6,  2,  1,  2,  4,  1, {1,  4,  1,  4,  1,  4}, "Tripod 6"},    // Tripod 6 steps
    [ROC_ROBOT_GAIT_CIRCLE_6]   =   {ROC_ROBOT_RUN_SPEED_DEFAULT,   6,  2,  1,  2,  4,  1, {1,  4,  1,  4,  1,  4}, "Circle 6"},    // In-situ circle 6 steps
};
#endif


static ROC_ROBOT_CONTROL_s g_RobotCtrl = {0};


/*********************************************************************************
 *  Description:
 *              Select the robot gait type
 *
 *  Parameter:
 *              CurLegNum: the selected leg to control
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.03.30)
**********************************************************************************/
static ROC_RESULT RocRobotGaitSelect(void)
{
    g_RobotCtrl.CurGait = g_RobotGait[g_RobotCtrl.CurState.GaitType];

    return RET_OK;
}

/*********************************************************************************
 *  Description:
 *              Update the robot leg position
 *
 *  Parameter:
 *              CurLegNum: the selected leg to control
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.03.30)
**********************************************************************************/
static void RocRobotGaitPosUpdate(uint8_t CurLegNum)
{
    // Try to reduce the number of time we look at g_RobotCtrl.CurGait.GaitLegNr and g_RobotCtrl.CurState.GaitStep
    int16_t LegStep = g_RobotCtrl.CurState.GaitStep - g_RobotCtrl.CurGait.GaitLegNr[CurLegNum];

    //Gait in motion
    if ((g_RobotCtrl.CurState.TravelRequest
            && (g_RobotCtrl.CurGait.NrLiftedPos == 1
            || g_RobotCtrl.CurGait.NrLiftedPos == 3
            || g_RobotCtrl.CurGait.NrLiftedPos == 5)
            && g_RobotCtrl.CurState.GaitStep == g_RobotCtrl.CurGait.GaitLegNr[CurLegNum])
            || (!g_RobotCtrl.CurState.TravelRequest
            && g_RobotCtrl.CurState.GaitStep == g_RobotCtrl.CurGait.GaitLegNr[CurLegNum]
            && ((abs(g_RobotCtrl.CurState.LegCurPos[CurLegNum].X) > 2)
            || (abs(g_RobotCtrl.CurState.LegCurPos[CurLegNum].Y) > 2)
            || (abs(g_RobotCtrl.CurState.GaitRot[CurLegNum]) > 2))))
    {
        g_RobotCtrl.CurState.LegCurPos[CurLegNum].X = 0;
        g_RobotCtrl.CurState.LegCurPos[CurLegNum].Y = 0;
        g_RobotCtrl.CurState.LegCurPos[CurLegNum].Z = g_RobotCtrl.CurState.LegLiftHeight;
        g_RobotCtrl.CurState.GaitRot[CurLegNum] = 0;

        LegStep = 1;
    }
    else if (((g_RobotCtrl.CurGait.NrLiftedPos == 2
            && g_RobotCtrl.CurState.GaitStep == g_RobotCtrl.CurGait.GaitLegNr[CurLegNum])
            || (g_RobotCtrl.CurGait.NrLiftedPos >= 3
            && (g_RobotCtrl.CurState.GaitStep == g_RobotCtrl.CurGait.GaitLegNr[CurLegNum] - 1
            || g_RobotCtrl.CurState.GaitStep == g_RobotCtrl.CurGait.GaitLegNr[CurLegNum] + (g_RobotCtrl.CurGait.StepsInGait - 1))))
            && g_RobotCtrl.CurState.TravelRequest)
    {    //Optional Half heigth Rear (2, 3, 5 lifted positions)
        if(ROC_ROBOT_GAIT_CIRCLE_6 != g_RobotCtrl.CurState.GaitType)
        {
            g_RobotCtrl.CurState.LegCurPos[CurLegNum].X = -g_RobotCtrl.CurState.TravelLength.X / g_RobotCtrl.CurGait.LiftDivFactor;
            g_RobotCtrl.CurState.LegCurPos[CurLegNum].Y = -g_RobotCtrl.CurState.TravelLength.Y / g_RobotCtrl.CurGait.LiftDivFactor;
            g_RobotCtrl.CurState.LegCurPos[CurLegNum].Z = 3 * g_RobotCtrl.CurState.LegLiftHeight / (3 + g_RobotCtrl.CurGait.HalfLiftHeight);     //Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
            g_RobotCtrl.CurState.GaitRot[CurLegNum] = -g_RobotCtrl.CurState.TravelLength.Z / g_RobotCtrl.CurGait.LiftDivFactor;
        }
        else
        {
            g_RobotCtrl.CurState.LegCurPos[CurLegNum].X = 0;
            g_RobotCtrl.CurState.LegCurPos[CurLegNum].Y = 0;
            g_RobotCtrl.CurState.LegCurPos[CurLegNum].Z = 3 * g_RobotCtrl.CurState.LegLiftHeight / (3 + g_RobotCtrl.CurGait.HalfLiftHeight);
            g_RobotCtrl.CurState.LegCurPos[CurLegNum].A = -g_RobotCtrl.CurState.TravelLength.A / g_RobotCtrl.CurGait.LiftDivFactor;
            g_RobotCtrl.CurState.GaitRot[CurLegNum] = -g_RobotCtrl.CurState.TravelLength.Z / g_RobotCtrl.CurGait.LiftDivFactor;
        }

        LegStep = 2;
    }
    else if ((g_RobotCtrl.CurGait.NrLiftedPos >= 2)
            && (g_RobotCtrl.CurState.GaitStep == g_RobotCtrl.CurGait.GaitLegNr[CurLegNum] + 1
            || g_RobotCtrl.CurState.GaitStep == g_RobotCtrl.CurGait.GaitLegNr[CurLegNum] - (g_RobotCtrl.CurGait.StepsInGait-1))
            && g_RobotCtrl.CurState.TravelRequest)
    {    // Optional Half heigth front (2, 3, 5 lifted positions)
        if(ROC_ROBOT_GAIT_CIRCLE_6 != g_RobotCtrl.CurState.GaitType)
        {
            g_RobotCtrl.CurState.LegCurPos[CurLegNum].X = g_RobotCtrl.CurState.TravelLength.X / g_RobotCtrl.CurGait.LiftDivFactor;
            g_RobotCtrl.CurState.LegCurPos[CurLegNum].Y = g_RobotCtrl.CurState.TravelLength.Y / g_RobotCtrl.CurGait.LiftDivFactor;
            g_RobotCtrl.CurState.LegCurPos[CurLegNum].Z = 3 * g_RobotCtrl.CurState.LegLiftHeight / (3 + g_RobotCtrl.CurGait.HalfLiftHeight); // Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
            g_RobotCtrl.CurState.GaitRot[CurLegNum] = g_RobotCtrl.CurState.TravelLength.Z / g_RobotCtrl.CurGait.LiftDivFactor;
        }
        else
        {
            g_RobotCtrl.CurState.LegCurPos[CurLegNum].X = 0;
            g_RobotCtrl.CurState.LegCurPos[CurLegNum].Y = 0;
            g_RobotCtrl.CurState.LegCurPos[CurLegNum].Z = 3 * g_RobotCtrl.CurState.LegLiftHeight / (3 + g_RobotCtrl.CurGait.HalfLiftHeight);
            g_RobotCtrl.CurState.LegCurPos[CurLegNum].A = g_RobotCtrl.CurState.TravelLength.A / g_RobotCtrl.CurGait.LiftDivFactor;
            g_RobotCtrl.CurState.GaitRot[CurLegNum] = -g_RobotCtrl.CurState.TravelLength.Z / g_RobotCtrl.CurGait.LiftDivFactor;
        }

        LegStep = 3;
    }
    else if (((g_RobotCtrl.CurGait.NrLiftedPos == 5
            && (g_RobotCtrl.CurState.GaitStep == g_RobotCtrl.CurGait.GaitLegNr[CurLegNum] - 2)))
            && g_RobotCtrl.CurState.TravelRequest)
    {    //Optional Half heigth Rear 5 LiftedPos (5 lifted positions)
        g_RobotCtrl.CurState.LegCurPos[CurLegNum].X = -g_RobotCtrl.CurState.TravelLength.X / 2;
        g_RobotCtrl.CurState.LegCurPos[CurLegNum].Y = -g_RobotCtrl.CurState.TravelLength.Y / 2;
        g_RobotCtrl.CurState.LegCurPos[CurLegNum].Z = g_RobotCtrl.CurState.LegLiftHeight / 2;
        g_RobotCtrl.CurState.GaitRot[CurLegNum] = -g_RobotCtrl.CurState.TravelLength.Z / 2;

        LegStep = 4;
    }
    else if ((g_RobotCtrl.CurGait.NrLiftedPos == 5)
            && (g_RobotCtrl.CurState.GaitStep == g_RobotCtrl.CurGait.GaitLegNr[CurLegNum] + 2
            || g_RobotCtrl.CurState.GaitStep == g_RobotCtrl.CurGait.GaitLegNr[CurLegNum] - (g_RobotCtrl.CurGait.StepsInGait-2))
            && g_RobotCtrl.CurState.TravelRequest)
    {   //Optional Half heigth Front 5 LiftedPos(5 lifted positions)
        g_RobotCtrl.CurState.LegCurPos[CurLegNum].X = g_RobotCtrl.CurState.TravelLength.X / 2;
        g_RobotCtrl.CurState.LegCurPos[CurLegNum].Y = g_RobotCtrl.CurState.TravelLength.Y / 2;
        g_RobotCtrl.CurState.LegCurPos[CurLegNum].Z = g_RobotCtrl.CurState.LegLiftHeight / 2;
        g_RobotCtrl.CurState.GaitRot[CurLegNum] = g_RobotCtrl.CurState.TravelLength.Z / 2;

        LegStep = 5;
    }
    else if((g_RobotCtrl.CurState.GaitStep == g_RobotCtrl.CurGait.GaitLegNr[CurLegNum] + g_RobotCtrl.CurGait.FrontDownPos
            || g_RobotCtrl.CurState.GaitStep == g_RobotCtrl.CurGait.GaitLegNr[CurLegNum] - (g_RobotCtrl.CurGait.StepsInGait - g_RobotCtrl.CurGait.FrontDownPos))
            && g_RobotCtrl.CurState.LegCurPos[CurLegNum].Y < 0)
    {   //Leg front down position
        g_RobotCtrl.CurState.LegCurPos[CurLegNum].X = g_RobotCtrl.CurState.TravelLength.X / 2;
        g_RobotCtrl.CurState.LegCurPos[CurLegNum].Y = g_RobotCtrl.CurState.TravelLength.Y / 2;
        g_RobotCtrl.CurState.LegCurPos[CurLegNum].Z = 0;
        g_RobotCtrl.CurState.GaitRot[CurLegNum] = g_RobotCtrl.CurState.TravelLength.Z / 2;

        LegStep = 6;
    }
    else
    {   //Move body forward
        if(ROC_ROBOT_GAIT_CIRCLE_6 != g_RobotCtrl.CurState.GaitType)
        {
            g_RobotCtrl.CurState.LegCurPos[CurLegNum].X = g_RobotCtrl.CurState.LegCurPos[CurLegNum].X - (g_RobotCtrl.CurState.TravelLength.X / g_RobotCtrl.CurGait.SlidDivFactor);
            g_RobotCtrl.CurState.LegCurPos[CurLegNum].Y = g_RobotCtrl.CurState.LegCurPos[CurLegNum].Y - (g_RobotCtrl.CurState.TravelLength.Y / g_RobotCtrl.CurGait.SlidDivFactor);
            g_RobotCtrl.CurState.LegCurPos[CurLegNum].Z = 0;
            g_RobotCtrl.CurState.GaitRot[CurLegNum] = g_RobotCtrl.CurState.GaitRot[CurLegNum] - (g_RobotCtrl.CurState.TravelLength.Y / g_RobotCtrl.CurGait.SlidDivFactor);
        }
        else
        {
            g_RobotCtrl.CurState.LegCurPos[CurLegNum].X = 0;
            g_RobotCtrl.CurState.LegCurPos[CurLegNum].Y = 0;
            g_RobotCtrl.CurState.LegCurPos[CurLegNum].Z = 0;
            g_RobotCtrl.CurState.LegCurPos[CurLegNum].A =  g_RobotCtrl.CurState.LegCurPos[CurLegNum].A - (g_RobotCtrl.CurState.TravelLength.A / g_RobotCtrl.CurGait.SlidDivFactor);
            g_RobotCtrl.CurState.GaitRot[CurLegNum] = g_RobotCtrl.CurState.GaitRot[CurLegNum] - (g_RobotCtrl.CurState.TravelLength.Y / g_RobotCtrl.CurGait.SlidDivFactor);
        }

        LegStep = 7;
    }

    if((ROC_ROBOT_CNT_LEGS - 1) == CurLegNum)
    {
        g_RobotCtrl.CurState.GaitStep++;                                    //Advance to the next step

        if (g_RobotCtrl.CurState.GaitStep > g_RobotCtrl.CurGait.StepsInGait) //The last leg in this step
        {
            g_RobotCtrl.CurState.GaitStep = 1;
        }
    }

#ifdef ROC_ROBOT_GAIT_DEBUG
    ROC_LOGI("LegNum: %d, LegStep: %d", CurLegNum, LegStep);
    ROC_LOGI("x:%.2f, y:%.2f, z:%.2f, a: %.2f", g_RobotCtrl.CurState.LegCurPos[CurLegNum].X, g_RobotCtrl.CurState.LegCurPos[CurLegNum].Y,
                                                g_RobotCtrl.CurState.LegCurPos[CurLegNum].Z, g_RobotCtrl.CurState.LegCurPos[CurLegNum].A);

#endif
}  

/*********************************************************************************
 *  Description:
 *              Update the robot gait sequence
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.03.30)
**********************************************************************************/
void RocRobotGaitSeqUpdate(void)
{
    uint8_t            LegIndex = 0;    //Index used for leg Index Number

    //Check if the Gait is in motion

    if(g_RobotCtrl.CurState.ForceGaitStepCnt != 0)
    {
        g_RobotCtrl.CurState.TravelRequest = ROC_ENABLE;
    }
    else
    {
        g_RobotCtrl.CurState.TravelRequest =    (abs(g_RobotCtrl.CurState.TravelLength.X) > ROC_ROBOT_TRAVEL_DEAD_ZONE)
                                             || (abs(g_RobotCtrl.CurState.TravelLength.Z) > ROC_ROBOT_TRAVEL_DEAD_ZONE)
                                             || (abs(g_RobotCtrl.CurState.TravelLength.Y) > ROC_ROBOT_TRAVEL_DEAD_ZONE);
    }

    g_RobotCtrl.CurState.TravelRequest = ROC_ENABLE;

    for(LegIndex = 0; LegIndex < ROC_ROBOT_CNT_LEGS; LegIndex++)
    {
        RocRobotGaitPosUpdate(LegIndex);
    }

    // If we have a force count decrement it now
    if (g_RobotCtrl.CurState.ForceGaitStepCnt)
    {
        g_RobotCtrl.CurState.ForceGaitStepCnt--;
    }
}

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
void RocRobotOpenLoopWalkCalculate(ROC_ROBOT_SERVO_s *pRobotServo)
{
    double                  x = 0;
    double                  y = 0;
    double                  z = 0;

    /***********the coordinate datas of the first group legs ****************/
    x = ROC_ROBOT_FRO_INIT_X + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_FRO_LEG].X;
    y = ROC_ROBOT_FRO_INIT_Y + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_FRO_LEG].Y;
    z = ROC_ROBOT_FRO_INIT_Z + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_FRO_LEG].Z;

#ifdef ROC_ROBOT_GAIT_DEBUG
    ROC_LOGW("x:%.2f, y:%.2f, z:%.2f", x, y, z);
#endif
    RocDhAlgorithmReverse(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_RIG_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_RIG_FRO_HIP_CENTER + (ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_RIG_FRO_LEG_CENTER + (ROC_ROBOT_FRO_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_RIG_FRO_FET_CENTER + (ROC_ROBOT_FRO_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    x = ROC_ROBOT_MID_INIT_X - g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_MID_LEG].X;
    y = ROC_ROBOT_MID_INIT_Y + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_MID_LEG].Y;
    z = ROC_ROBOT_MID_INIT_Z + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_MID_LEG].Z;

    RocDhAlgorithmReverse(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_LEF_MID_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_LEF_MID_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_MID_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_MID_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_LEF_MID_LEG_CENTER + (ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_MID_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_LEF_MID_FET_CENTER + (-ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    x = ROC_ROBOT_HIN_INIT_X + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_HIN_LEG].X;
    y = ROC_ROBOT_HIN_INIT_Y - g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_HIN_LEG].Y;
    z = ROC_ROBOT_HIN_INIT_Z + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_HIN_LEG].Z;

    RocDhAlgorithmReverse(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_RIG_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_RIG_HIN_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_RIG_HIN_LEG_CENTER + (ROC_ROBOT_HIN_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_RIG_HIN_FET_CENTER + (ROC_ROBOT_HIN_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    /***********the coordinate datas of the second group legs ***************/
    x = ROC_ROBOT_FRO_INIT_X - g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_FRO_LEG].X;
    y = ROC_ROBOT_FRO_INIT_Y + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_FRO_LEG].Y;
    z = ROC_ROBOT_FRO_INIT_Z + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_FRO_LEG].Z;

    RocDhAlgorithmReverse(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_LEF_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_LEF_FRO_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_FRO_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_LEF_FRO_LEG_CENTER + (g_DhAngleBuffer[1] - ROC_ROBOT_FRO_LEG_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_LEF_FRO_FET_CENTER + (-ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    x = ROC_ROBOT_MID_INIT_X + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_MID_LEG].X;
    y = ROC_ROBOT_MID_INIT_Y + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_MID_LEG].Y;
    z = ROC_ROBOT_MID_INIT_Z + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_MID_LEG].Z;

    RocDhAlgorithmReverse(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_RIG_MID_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_RIG_MID_HIP_CENTER + (ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_MID_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_RIG_MID_LEG_CENTER + (-ROC_ROBOT_MID_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_MID_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_RIG_MID_FET_CENTER + (ROC_ROBOT_MID_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    x = ROC_ROBOT_HIN_INIT_X - g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_HIN_LEG].X;
    y = ROC_ROBOT_HIN_INIT_Y - g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_HIN_LEG].Y;
    z = ROC_ROBOT_HIN_INIT_Z + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_HIN_LEG].Z;

    RocDhAlgorithmReverse(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_LEF_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_LEF_HIN_HIP_CENTER + (ROC_ROBOT_HIN_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_LEF_HIN_LEG_CENTER + (g_DhAngleBuffer[1] - ROC_ROBOT_HIN_LEG_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_LEF_HIN_FET_CENTER + (-ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
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
void RocRobotOpenLoopCircleCalculate(ROC_ROBOT_SERVO_s *pRobotServo)
{
    double                  x = 0;
    double                  y = 0;
    double                  z = 0;

    /***********the coordinate datas of the first group legs ****************/
    x = ROC_ROBOT_WIDTH * cos( (ROC_ROBOT_FRO_HIP_INIT_ANGLE + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_FRO_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * sin( (ROC_ROBOT_FRO_HIP_INIT_ANGLE + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_FRO_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_FRO_INIT_Z + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_MID_LEG].Z;

#ifdef ROC_ROBOT_GAIT_DEBUG
    ROC_LOGW("x:%.2f, y:%.2f, z:%.2f", x, y, z);
#endif
    RocDhAlgorithmReverse(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_RIG_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_RIG_FRO_HIP_CENTER + (ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_RIG_FRO_LEG_CENTER + (ROC_ROBOT_FRO_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_RIG_FRO_FET_CENTER + (ROC_ROBOT_FRO_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    x = ROC_ROBOT_WIDTH * cos( (ROC_ROBOT_MID_HIP_INIT_ANGLE - g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_MID_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * sin( (ROC_ROBOT_MID_HIP_INIT_ANGLE - g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_MID_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_MID_INIT_Z + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_MID_LEG].Z;

    RocDhAlgorithmReverse(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_LEF_MID_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_LEF_MID_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_MID_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_MID_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_LEF_MID_LEG_CENTER + (ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_MID_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_LEF_MID_FET_CENTER + (-ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    x = ROC_ROBOT_WIDTH * cos( (ROC_ROBOT_HIN_HIP_INIT_ANGLE - g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_HIN_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * sin( (ROC_ROBOT_HIN_HIP_INIT_ANGLE - g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_HIN_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_HIN_INIT_Z + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_HIN_LEG].Z;

    RocDhAlgorithmReverse(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_RIG_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_RIG_HIN_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_RIG_HIN_LEG_CENTER + (ROC_ROBOT_HIN_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_RIG_HIN_FET_CENTER + (ROC_ROBOT_HIN_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    /***********the coordinate datas of the second group legs ***************/
    x = ROC_ROBOT_WIDTH * cos( (ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_FRO_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * sin( (ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_FRO_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_FRO_INIT_Z + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_FRO_LEG].Z;

    RocDhAlgorithmReverse(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_LEF_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_LEF_FRO_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_FRO_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_LEF_FRO_LEG_CENTER + (g_DhAngleBuffer[1] - ROC_ROBOT_FRO_LEG_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_LEF_FRO_FET_CENTER + (-ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    x = ROC_ROBOT_WIDTH * cos( (ROC_ROBOT_MID_HIP_INIT_ANGLE + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_MID_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * sin( (ROC_ROBOT_MID_HIP_INIT_ANGLE + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_MID_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_MID_INIT_Z + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_MID_LEG].Z;

    RocDhAlgorithmReverse(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_RIG_MID_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_RIG_MID_HIP_CENTER + (ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_MID_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_RIG_MID_LEG_CENTER + (-ROC_ROBOT_MID_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_MID_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_RIG_MID_FET_CENTER + (ROC_ROBOT_MID_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    x = ROC_ROBOT_WIDTH * cos( (ROC_ROBOT_HIN_HIP_INIT_ANGLE + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_HIN_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * sin( (ROC_ROBOT_HIN_HIP_INIT_ANGLE + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_HIN_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_HIN_INIT_Z + g_RobotCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_HIN_LEG].Z;

    RocDhAlgorithmReverse(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_LEF_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_LEF_HIN_HIP_CENTER + (ROC_ROBOT_HIN_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_LEF_HIN_LEG_CENTER + (g_DhAngleBuffer[1] - ROC_ROBOT_HIN_LEG_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_LEF_HIN_FET_CENTER + (-ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
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
        l = ROC_ROBOT_DEFAULT_LEG_STEP;

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

    if(ROC_ROBOT_RIG_FRO_LEG == LegNum)
    {
        *x = c1 * cos(ROC_ROBOT_INIT_ANGLE_THET_1 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN + alfa1);
        *y = c1 * sin(ROC_ROBOT_INIT_ANGLE_THET_1 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN + alfa1);
    }
    else if(ROC_ROBOT_RIG_MID_LEG == LegNum)
    {
        *x = c2 * cos(ROC_ROBOT_INIT_ANGLE_THET_2 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN + alfa2);
        *y = c2 * sin(ROC_ROBOT_INIT_ANGLE_THET_2 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN + alfa2);
    }
    else if(ROC_ROBOT_RIG_HIN_LEG == LegNum)
    {
        *x = c3 * cos(ROC_ROBOT_INIT_ANGLE_THET_3 * ROC_ROBOT_ANGLE_TO_RADIAN - DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN - alfa3);
        *y = c3 * sin(ROC_ROBOT_INIT_ANGLE_THET_3 * ROC_ROBOT_ANGLE_TO_RADIAN - DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN - alfa3);
    }
    else if(ROC_ROBOT_LEF_FRO_LEG == LegNum)
    {
        *x = c4 * cos(ROC_ROBOT_INIT_ANGLE_THET_1 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN - alfa4);
        *y = c4 * sin(ROC_ROBOT_INIT_ANGLE_THET_1 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN - alfa4);
    }
    else if(ROC_ROBOT_LEF_MID_LEG == LegNum)
    {
        *x = c5 * cos(ROC_ROBOT_INIT_ANGLE_THET_2 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN - alfa5);
        *y = c5 * sin(ROC_ROBOT_INIT_ANGLE_THET_2 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN - alfa5);
    }
    else if(ROC_ROBOT_LEF_HIN_LEG == LegNum)
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

/*********************************************************************************
 *  Description:
 *              Get robot current control information
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              The control parameter
 *
 *  Author:
 *              ROC LiRen(2019.03.30)
**********************************************************************************/
ROC_ROBOT_CONTROL_s *RocRobotCtrlInfoGet(void)
{
    return &g_RobotCtrl;
}

/*********************************************************************************
 *  Description:
 *              Set the robot move status
 *
 *  Parameter:
 *              MoveStatus: the robot current move status
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.06)
**********************************************************************************/
void RocRobotMoveStatus_Set(ROC_ROBOT_MOVE_STATUS_e MoveStatus)
{
    g_RobotCtrl.CurState.MoveStatus = MoveStatus;
}

/*********************************************************************************
 *  Description:
 *              Get the robot move status
 *
 *  Parameter:
 *              MoveStatus: the robot current move status
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.06)
**********************************************************************************/
ROC_ROBOT_MOVE_STATUS_e RocRobotMoveStatus_Get(void)
{
    return g_RobotCtrl.CurState.MoveStatus;
}

/*********************************************************************************
 *  Description:
 *              Input the delta move coordinate
 *              For example: move (5, 5) every cycle
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              x: the delta x coordinate
                y: the delta y coordinate
                z: the delta z coordinate
                h: the lift height of robot leg when mov
 *
 *  Author:
 *              ROC LiRen(2019.03.30)
**********************************************************************************/
void RocRobotCtrlDeltaMoveCoorInput(double x, double y, double z, double a, double h)
{
    g_RobotCtrl.CurState.TravelLength.X = x;
    g_RobotCtrl.CurState.TravelLength.Y = y;
    g_RobotCtrl.CurState.TravelLength.Z = z;
    g_RobotCtrl.CurState.TravelLength.A = a;
    g_RobotCtrl.CurState.LegLiftHeight  = h;
}

/*********************************************************************************
 *  Description:
 *              Update robot current leg position
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.06)
**********************************************************************************/
static void RocRobotCurLegPosUpdate(void)
{
    uint8_t i = 0;

    for(i = 0; i < ROC_ROBOT_CNT_LEGS; i++)
    {
        g_RobotCtrl.CurState.LegCurPos[i].X = g_RobotCtrl.CurState.TravelLength.X;
        g_RobotCtrl.CurState.LegCurPos[i].Y = g_RobotCtrl.CurState.TravelLength.Y;
        g_RobotCtrl.CurState.LegCurPos[i].Z = g_RobotCtrl.CurState.TravelLength.Z;
        g_RobotCtrl.CurState.LegCurPos[i].A = g_RobotCtrl.CurState.TravelLength.A;
    }
}

/*********************************************************************************
 *  Description:
 *              Update the robot single position
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.06)
**********************************************************************************/
void RocRobotSingleLegPosUpdate(ROC_ROBOT_SERVO_s *pRobotServo)
{
    uint8_t i = 0;

    g_RobotCtrl.CurState.SelectLegIsAllDown = ROC_TRUE;

    RocRobotCurLegPosUpdate();

    for(i = 0; i < ROC_ROBOT_CNT_LEGS; i++)
    {
        if(0 != g_RobotCtrl.CurState.LegCurPos[i].Z)
        {
            g_RobotCtrl.CurState.SelectLegIsAllDown = ROC_FALSE;
        }
    }

    if(ROC_FALSE == g_RobotCtrl.CurState.SelectLegIsAllDown)
    {
        RocRobotOpenLoopWalkCalculate(pRobotServo);
    }
    else if(ROC_TRUE == g_RobotCtrl.CurState.SelectLegIsAllDown)
    {
        g_RobotCtrl.CurState.LegCurPos[g_RobotCtrl.CurState.SelectLegNum].Z = 20;
    }

}

/*********************************************************************************
 *  Description:
 *              Robot algorithm parameter init
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.03.30)
**********************************************************************************/
ROC_RESULT RocRobotAlgoCtrlInit(void)
{
    uint8_t     i = 0;
    ROC_RESULT  Ret = RET_OK;

    for(i = 0; i < ROC_ROBOT_CNT_LEGS; i++)
    {
        g_RobotCtrl.CurState.LegCurPos[i].X = 0;
        g_RobotCtrl.CurState.LegCurPos[i].Y = 0;
        g_RobotCtrl.CurState.LegCurPos[i].Z = 0;
        g_RobotCtrl.CurState.LegCurPos[i].A = 0;
    }

    g_RobotCtrl.CurState.GaitStep = 1;
    g_RobotCtrl.CurState.GaitType = ROC_ROBOT_GAIT_TRIPOD_6;

    Ret = RocRobotGaitSelect();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Robot algorithm is in error!");
    }

    return Ret;
}



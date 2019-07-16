/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2018/12/16      1.0
********************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "RocLog.h"
#include "RocRobotMath.h"
#include "RocTftLcd.h"
#include "RocMpu6050.h"
#include "RocRobotDhAlgorithm.h"


#ifdef ROC_ROBOT_GAIT_QUAD_MODE_ENABLE
static ROC_PHOENIX_GAIT_s g_RobotGait[] =
{
    [ROC_ROBOT_GAIT_HEXP_MODE_RIPPLE_12]    = {ROC_ROBOT_RUN_SPEED_DEFAULT, 12, 3,  2,  2,  8,  3,  0,  0,  0,  1,  {7, 11, 3,  1,  5,  9}, "Ripple 12"},   // Ripple 12
    [ROC_ROBOT_GAIT_HEXP_MODE_TRIPOD_8]     = {ROC_ROBOT_RUN_SPEED_DEFAULT, 8,  3,  2,  2,  4,  3,  0,  0,  0,  1,  {1, 5,  1,  5,  1,  5}, "Tripod 8"},    // Tripod 8 steps
    [ROC_ROBOT_GAIT_HEXP_MODE_TRIPLE_12]    = {ROC_ROBOT_RUN_SPEED_DEFAULT, 12, 3,  2,  2,  8,  3,  0,  0,  0,  1,  {5, 10, 3,  11, 4,  9}, "Tripple 12"},  // Triple Tripod 12 step
    [ROC_ROBOT_GAIT_HEXP_MODE_TRIPLE_16]    = {ROC_ROBOT_RUN_SPEED_DEFAULT, 16, 5,  3,  4,  10, 1,  0,  0,  0,  1,  {6, 13, 4,  14, 5,  12},"Tripple 16"},  // Triple Tripod 16 steps, use 5 lifted positions
    [ROC_ROBOT_GAIT_HEXP_MODE_WAVE_24]      = {ROC_ROBOT_RUN_SPEED_DEFAULT, 24, 3,  2,  2,  20, 3,  0,  0,  0,  1,  {13,17, 21, 1,  5,  9}, "Wave 24"},     // Wave 24 steps
    [ROC_ROBOT_GAIT_HEXP_MODE_TRIPOD_6]     = {ROC_ROBOT_RUN_SPEED_DEFAULT, 6,  2,  1,  2,  4,  1,  0,  0,  0,  1,  {1, 4,  1,  4,  1,  4}, "Tripod 6"},    // Tripod 6 steps
    [ROC_ROBOT_GAIT_HEXP_MODE_CIRCLE_6]     = {ROC_ROBOT_RUN_SPEED_DEFAULT, 6,  2,  1,  2,  4,  1,  0,  0,  0,  1,  {1, 4,  1,  4,  1,  4}, "Circle 6"},    // In-situ circle 6 steps

    [ROC_ROBOT_GAIT_QUAD_MODE_RIPPLE_4]     = {ROC_ROBOT_RUN_SPEED_DEFAULT, 8,  2,  1,  2,  6,  1,  0,  0,  0,  1,  {7, 0,  1,  3,  0,  5}, "Ripple 4"},        // Ripple
    [ROC_ROBOT_GAIT_QUAD_MODE_SM_RIPPLE_4]  = {ROC_ROBOT_RUN_SPEED_DEFAULT, 12, 2,  1,  2,  10, 1,  0,  0,  0,  1,  {7, 0,  1,  4,  0,  10},"Smooth Ripple 4"}, // Smooth Ripple
    [ROC_ROBOT_GAIT_QUAD_MODE_AMBLE_4]      = {ROC_ROBOT_RUN_SPEED_DEFAULT, 4,  2,  1,  2,  2,  1,  0,  0,  0,  1,  {3, 0,  1,  1,  0,  3}, "Amble 4"},         // Amble
    [ROC_ROBOT_GAIT_QUAD_MODE_SM_AMBLE_4]   = {ROC_ROBOT_RUN_SPEED_DEFAULT, 6,  3,  2,  2,  3,  2,  0,  0,  0,  1,  {1, 0,  4,  4,  0,  1}, "Smooth Amble 4"},  // Smooth Amble
};
#else
static ROC_PHOENIX_GAIT_s g_RobotGait[] =
{
    [ROC_ROBOT_GAIT_HEXP_MODE_RIPPLE_12]    = {ROC_ROBOT_RUN_SPEED_DEFAULT, 12, 3,  2,  2,  8,  3, {7,  11, 3,  1,  5,  9}, "Ripple 12"},   // Ripple 12
    [ROC_ROBOT_GAIT_HEXP_MODE_TRIPOD_8]     = {ROC_ROBOT_RUN_SPEED_DEFAULT, 8,  3,  2,  2,  4,  3, {1,  5,  1,  5,  1,  5}, "Tripod 8"},    // Tripod 8 steps
    [ROC_ROBOT_GAIT_HEXP_MODE_TRIPLE_12]    = {ROC_ROBOT_RUN_SPEED_DEFAULT, 12, 3,  2,  2,  8,  3, {5,  10, 3,  11, 4,  9}, "Tripple 12"},  // Triple Tripod 12 step
    [ROC_ROBOT_GAIT_HEXP_MODE_TRIPLE_16]    = {ROC_ROBOT_RUN_SPEED_DEFAULT, 16, 5,  3,  4,  10, 1, {6,  13, 4,  14, 5,  12},"Tripple 16"},  // Triple Tripod 16 steps, use 5 lifted positions
    [ROC_ROBOT_GAIT_HEXP_MODE_WAVE_24]      = {ROC_ROBOT_RUN_SPEED_DEFAULT, 24, 3,  2,  2,  20, 3, {13, 17, 21, 1,  5,  9}, "Wave 24"},     // Wave 24 steps
    [ROC_ROBOT_GAIT_HEXP_MODE_TRIPOD_6]     = {ROC_ROBOT_RUN_SPEED_DEFAULT, 6,  2,  1,  2,  4,  1, {1,  4,  1,  4,  1,  4}, "Tripod 6"},    // Tripod 6 steps
    [ROC_ROBOT_GAIT_HEXP_MODE_CIRCLE_6]     = {ROC_ROBOT_RUN_SPEED_DEFAULT, 6,  2,  1,  2,  4,  1, {1,  4,  1,  4,  1,  4}, "Circle 6"},    // In-situ circle 6 steps
};
#endif


static float                    g_DhAngleBuffer[3];
static float                    g_BodyIkPos[3] = {0};
static ROC_ROBOT_MOVE_CTRL_s    g_RobotMoveCtrl = {0};

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
    g_RobotMoveCtrl.CurGait = g_RobotGait[g_RobotMoveCtrl.CurState.GaitType];

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
    // Try to reduce the number of time we look at g_RobotMoveCtrl.CurGait.GaitLegNr and g_RobotMoveCtrl.CurState.GaitStep
    int16_t LegStep = g_RobotMoveCtrl.CurState.GaitStep - g_RobotMoveCtrl.CurGait.GaitLegNr[CurLegNum];
    uint8_t GaitPos = 0;

#if 0
    //Gait in motion
    if ((g_RobotMoveCtrl.CurState.TravelRequest
            && (g_RobotMoveCtrl.CurGait.NrLiftedPos == 1
            || g_RobotMoveCtrl.CurGait.NrLiftedPos == 3
            || g_RobotMoveCtrl.CurGait.NrLiftedPos == 5)
            && g_RobotMoveCtrl.CurState.GaitStep == g_RobotMoveCtrl.CurGait.GaitLegNr[CurLegNum])
            || (!g_RobotMoveCtrl.CurState.TravelRequest
            && g_RobotMoveCtrl.CurState.GaitStep == g_RobotMoveCtrl.CurGait.GaitLegNr[CurLegNum]
            && ((fabs(g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X) > 2)
            || (fabs(g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y) > 2)
            || (fabs(g_RobotMoveCtrl.CurState.GaitRot[CurLegNum]) > 2))))
    {
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X = 0;
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y = 0;
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Z = g_RobotMoveCtrl.CurState.LegLiftHeight;
        g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] = 0;

        GaitPos = 1;
    }
    else if (((g_RobotMoveCtrl.CurGait.NrLiftedPos == 2
            && g_RobotMoveCtrl.CurState.GaitStep == g_RobotMoveCtrl.CurGait.GaitLegNr[CurLegNum])
            || (g_RobotMoveCtrl.CurGait.NrLiftedPos >= 3
            && (g_RobotMoveCtrl.CurState.GaitStep == g_RobotMoveCtrl.CurGait.GaitLegNr[CurLegNum] - 1
            || g_RobotMoveCtrl.CurState.GaitStep == g_RobotMoveCtrl.CurGait.GaitLegNr[CurLegNum] + (g_RobotMoveCtrl.CurGait.StepsInGait - 1))))
            && g_RobotMoveCtrl.CurState.TravelRequest)
    {    //Optional Half heigth Rear (2, 3, 5 lifted positions)
        if(ROC_ROBOT_MOVE_STATUS_CIRCLING != g_RobotMoveCtrl.CurState.MoveStatus)
        {
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X = -g_RobotMoveCtrl.CurState.TravelLength.X / g_RobotMoveCtrl.CurGait.LiftDivFactor;
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y = -g_RobotMoveCtrl.CurState.TravelLength.Y / g_RobotMoveCtrl.CurGait.LiftDivFactor;
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Z = 3 * g_RobotMoveCtrl.CurState.LegLiftHeight / (3 + g_RobotMoveCtrl.CurGait.HalfLiftHeight);     //Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
            g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] = -g_RobotMoveCtrl.CurState.TravelLength.Z / g_RobotMoveCtrl.CurGait.LiftDivFactor;
        }
        else
        {
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X = 0;
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y = 0;
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Z = 3 * g_RobotMoveCtrl.CurState.LegLiftHeight / (3 + g_RobotMoveCtrl.CurGait.HalfLiftHeight);
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].A = -g_RobotMoveCtrl.CurState.TravelLength.A / g_RobotMoveCtrl.CurGait.LiftDivFactor;
            g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] = -g_RobotMoveCtrl.CurState.TravelLength.Z / g_RobotMoveCtrl.CurGait.LiftDivFactor;
        }

        GaitPos = 2;
    }
    else if ((g_RobotMoveCtrl.CurGait.NrLiftedPos >= 2)
            && (g_RobotMoveCtrl.CurState.GaitStep == g_RobotMoveCtrl.CurGait.GaitLegNr[CurLegNum] + 1
            || g_RobotMoveCtrl.CurState.GaitStep == g_RobotMoveCtrl.CurGait.GaitLegNr[CurLegNum] - (g_RobotMoveCtrl.CurGait.StepsInGait-1))
            && g_RobotMoveCtrl.CurState.TravelRequest)
    {    // Optional Half heigth front (2, 3, 5 lifted positions)
        if(ROC_ROBOT_MOVE_STATUS_CIRCLING != g_RobotMoveCtrl.CurState.MoveStatus)
        {
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X = g_RobotMoveCtrl.CurState.TravelLength.X / g_RobotMoveCtrl.CurGait.LiftDivFactor;
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y = g_RobotMoveCtrl.CurState.TravelLength.Y / g_RobotMoveCtrl.CurGait.LiftDivFactor;
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Z = 3 * g_RobotMoveCtrl.CurState.LegLiftHeight / (3 + g_RobotMoveCtrl.CurGait.HalfLiftHeight); // Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
            g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] = g_RobotMoveCtrl.CurState.TravelLength.Z / g_RobotMoveCtrl.CurGait.LiftDivFactor;
        }
        else
        {
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X = 0;
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y = 0;
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Z = 3 * g_RobotMoveCtrl.CurState.LegLiftHeight / (3 + g_RobotMoveCtrl.CurGait.HalfLiftHeight);
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].A = g_RobotMoveCtrl.CurState.TravelLength.A / g_RobotMoveCtrl.CurGait.LiftDivFactor;
            g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] = -g_RobotMoveCtrl.CurState.TravelLength.Z / g_RobotMoveCtrl.CurGait.LiftDivFactor;
        }

        GaitPos = 3;
    }
    else if (((g_RobotMoveCtrl.CurGait.NrLiftedPos == 5
            && (g_RobotMoveCtrl.CurState.GaitStep == g_RobotMoveCtrl.CurGait.GaitLegNr[CurLegNum] - 2)))
            && g_RobotMoveCtrl.CurState.TravelRequest)
    {    //Optional Half heigth Rear 5 LiftedPos (5 lifted positions)
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X = -g_RobotMoveCtrl.CurState.TravelLength.X / 2;
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y = -g_RobotMoveCtrl.CurState.TravelLength.Y / 2;
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Z = g_RobotMoveCtrl.CurState.LegLiftHeight / 2;
        g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] = -g_RobotMoveCtrl.CurState.TravelLength.Z / 2;

        GaitPos = 4;
    }
    else if ((g_RobotMoveCtrl.CurGait.NrLiftedPos == 5)
            && (g_RobotMoveCtrl.CurState.GaitStep == g_RobotMoveCtrl.CurGait.GaitLegNr[CurLegNum] + 2
            || g_RobotMoveCtrl.CurState.GaitStep == g_RobotMoveCtrl.CurGait.GaitLegNr[CurLegNum] - (g_RobotMoveCtrl.CurGait.StepsInGait-2))
            && g_RobotMoveCtrl.CurState.TravelRequest)
    {   //Optional Half heigth Front 5 LiftedPos(5 lifted positions)
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X = g_RobotMoveCtrl.CurState.TravelLength.X / 2;
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y = g_RobotMoveCtrl.CurState.TravelLength.Y / 2;
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Z = g_RobotMoveCtrl.CurState.LegLiftHeight / 2;
        g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] = g_RobotMoveCtrl.CurState.TravelLength.Z / 2;

        GaitPos = 5;
    }
    else if((g_RobotMoveCtrl.CurState.GaitStep == g_RobotMoveCtrl.CurGait.GaitLegNr[CurLegNum] + g_RobotMoveCtrl.CurGait.FrontDownPos
            || g_RobotMoveCtrl.CurState.GaitStep == g_RobotMoveCtrl.CurGait.GaitLegNr[CurLegNum] - (g_RobotMoveCtrl.CurGait.StepsInGait - g_RobotMoveCtrl.CurGait.FrontDownPos))
            && g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y < 0)
    {   //Leg front down position
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X = g_RobotMoveCtrl.CurState.TravelLength.X / 2;
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y = g_RobotMoveCtrl.CurState.TravelLength.Y / 2;
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Z = 0;
        g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] = g_RobotMoveCtrl.CurState.TravelLength.Z / 2;

        GaitPos = 6;
    }
    else
    {   //Move body forward
        if(ROC_ROBOT_MOVE_STATUS_CIRCLING != g_RobotMoveCtrl.CurState.MoveStatus)
        {
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X = g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X - (g_RobotMoveCtrl.CurState.TravelLength.X / g_RobotMoveCtrl.CurGait.SlidDivFactor);
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y = g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y - (g_RobotMoveCtrl.CurState.TravelLength.Y / g_RobotMoveCtrl.CurGait.SlidDivFactor);
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Z = 0;
            g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] = g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] - (g_RobotMoveCtrl.CurState.TravelLength.Y / g_RobotMoveCtrl.CurGait.SlidDivFactor);
        }
        else
        {
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X = 0;
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y = 0;
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Z = 0;
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].A =  g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].A - (g_RobotMoveCtrl.CurState.TravelLength.A / g_RobotMoveCtrl.CurGait.SlidDivFactor);
            g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] = g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] - (g_RobotMoveCtrl.CurState.TravelLength.Y / g_RobotMoveCtrl.CurGait.SlidDivFactor);
        }

        GaitPos = 7;
    }

    if((ROC_ROBOT_CNT_LEGS - 1) == CurLegNum)
    {
        g_RobotMoveCtrl.CurState.GaitStep++;                                    //Advance to the next step

        if (g_RobotMoveCtrl.CurState.GaitStep > g_RobotMoveCtrl.CurGait.StepsInGait) //The last leg in this step
        {
            g_RobotMoveCtrl.CurState.GaitStep = 1;
        }
    }
#endif

    //Gait in motion
    if ((g_RobotMoveCtrl.CurState.TravelRequest
        && (g_RobotMoveCtrl.CurGait.NrLiftedPos == 1) && (LegStep == 0))
        || (!g_RobotMoveCtrl.CurState.TravelRequest && (LegStep == 0)
        && ((fabs(g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X) > 2)
        || (fabs(g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y) > 2)
        || (fabs(g_RobotMoveCtrl.CurState.GaitRot[CurLegNum]) > 2))))
    {
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X = 0;
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y = 0;
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Z = g_RobotMoveCtrl.CurState.LegLiftHeight;
        g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] = 0;

        GaitPos = 1;
    }
    else if ((((g_RobotMoveCtrl.CurGait.NrLiftedPos == 2)&& (LegStep == 0))
        || ((g_RobotMoveCtrl.CurGait.NrLiftedPos >= 3)
        && ((LegStep == -1) || (LegStep == g_RobotMoveCtrl.CurGait.StepsInGait - 1))))
        && g_RobotMoveCtrl.CurState.TravelRequest)
    {    //Optional Half heigth Rear (2, 3, 5 lifted positions)
        if(ROC_ROBOT_MOVE_STATUS_CIRCLING != g_RobotMoveCtrl.CurState.MoveStatus)
        {
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X = -g_RobotMoveCtrl.CurState.TravelLength.X / g_RobotMoveCtrl.CurGait.LiftDivFactor;
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y = -g_RobotMoveCtrl.CurState.TravelLength.Y / g_RobotMoveCtrl.CurGait.LiftDivFactor;
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Z = 3 * g_RobotMoveCtrl.CurState.LegLiftHeight / (3 + g_RobotMoveCtrl.CurGait.HalfLiftHeight);     //Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
            g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] = -g_RobotMoveCtrl.CurState.TravelLength.Z / g_RobotMoveCtrl.CurGait.LiftDivFactor;
        }
        else
        {
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X = 0;
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y = 0;
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Z = 3 * g_RobotMoveCtrl.CurState.LegLiftHeight / (3 + g_RobotMoveCtrl.CurGait.HalfLiftHeight);
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].A = -g_RobotMoveCtrl.CurState.TravelLength.A / g_RobotMoveCtrl.CurGait.LiftDivFactor;
            g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] = -g_RobotMoveCtrl.CurState.TravelLength.Z / g_RobotMoveCtrl.CurGait.LiftDivFactor;
        }

        GaitPos = 2;
    }
    else if ((g_RobotMoveCtrl.CurGait.NrLiftedPos >= 2)
        && ((LegStep == 1) || LegStep == - (g_RobotMoveCtrl.CurGait.StepsInGait-1))
        && g_RobotMoveCtrl.CurState.TravelRequest)
    {    // Optional Half heigth front (2, 3, 5 lifted positions)
        if(ROC_ROBOT_MOVE_STATUS_CIRCLING != g_RobotMoveCtrl.CurState.MoveStatus)
        {
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X = g_RobotMoveCtrl.CurState.TravelLength.X / g_RobotMoveCtrl.CurGait.LiftDivFactor;
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y = g_RobotMoveCtrl.CurState.TravelLength.Y / g_RobotMoveCtrl.CurGait.LiftDivFactor;
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Z = 3 * g_RobotMoveCtrl.CurState.LegLiftHeight / (3 + g_RobotMoveCtrl.CurGait.HalfLiftHeight); // Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
            g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] = g_RobotMoveCtrl.CurState.TravelLength.Z / g_RobotMoveCtrl.CurGait.LiftDivFactor;
        }
        else
        {
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X = 0;
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y = 0;
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Z = 3 * g_RobotMoveCtrl.CurState.LegLiftHeight / (3 + g_RobotMoveCtrl.CurGait.HalfLiftHeight);
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].A = g_RobotMoveCtrl.CurState.TravelLength.A / g_RobotMoveCtrl.CurGait.LiftDivFactor;
            g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] = -g_RobotMoveCtrl.CurState.TravelLength.Z / g_RobotMoveCtrl.CurGait.LiftDivFactor;
        }

        GaitPos = 3;
    }
    else if (((g_RobotMoveCtrl.CurGait.NrLiftedPos == 5) && (LegStep == -2))
        && g_RobotMoveCtrl.CurState.TravelRequest)
    {    //Optional Half heigth Rear 5 LiftedPos (5 lifted positions)
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X = -g_RobotMoveCtrl.CurState.TravelLength.X / 2;
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y = -g_RobotMoveCtrl.CurState.TravelLength.Y / 2;
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Z = g_RobotMoveCtrl.CurState.LegLiftHeight / 2;
        g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] = -g_RobotMoveCtrl.CurState.TravelLength.Z / 2;

        GaitPos = 4;
    }
    else if ((g_RobotMoveCtrl.CurGait.NrLiftedPos == 5)
        && ((LegStep == 2) || (LegStep == -(g_RobotMoveCtrl.CurGait.StepsInGait - 2)))
        && g_RobotMoveCtrl.CurState.TravelRequest)
    {   //Optional Half heigth Front 5 LiftedPos(5 lifted positions)
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X = g_RobotMoveCtrl.CurState.TravelLength.X / 2;
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y = g_RobotMoveCtrl.CurState.TravelLength.Y / 2;
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Z = g_RobotMoveCtrl.CurState.LegLiftHeight / 2;
        g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] = g_RobotMoveCtrl.CurState.TravelLength.Z / 2;

        GaitPos = 5;
    }
    else if(((LegStep == g_RobotMoveCtrl.CurGait.FrontDownPos)
        || (LegStep == -(g_RobotMoveCtrl.CurGait.StepsInGait - g_RobotMoveCtrl.CurGait.FrontDownPos)))
        && g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y < 0)
    {   //Leg front down position
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X = g_RobotMoveCtrl.CurState.TravelLength.X / 2;
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y = g_RobotMoveCtrl.CurState.TravelLength.Y / 2;
        g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Z = 0;
        g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] = g_RobotMoveCtrl.CurState.TravelLength.Z / 2;

        GaitPos = 6;
    }
    else
    {   //Move body forward
        if(ROC_ROBOT_MOVE_STATUS_CIRCLING != g_RobotMoveCtrl.CurState.MoveStatus)
        {
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X = g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X - (g_RobotMoveCtrl.CurState.TravelLength.X / g_RobotMoveCtrl.CurGait.SlidDivFactor);
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y = g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y - (g_RobotMoveCtrl.CurState.TravelLength.Y / g_RobotMoveCtrl.CurGait.SlidDivFactor);
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Z = 0;
            g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] = g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] - (g_RobotMoveCtrl.CurState.TravelLength.Y / g_RobotMoveCtrl.CurGait.SlidDivFactor);
        }
        else
        {
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X = 0;
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y = 0;
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Z = 0;
            g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].A =  g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].A - (g_RobotMoveCtrl.CurState.TravelLength.A / g_RobotMoveCtrl.CurGait.SlidDivFactor);
            g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] = g_RobotMoveCtrl.CurState.GaitRot[CurLegNum] - (g_RobotMoveCtrl.CurState.TravelLength.Y / g_RobotMoveCtrl.CurGait.SlidDivFactor);
        }

        GaitPos = 7;
    }

    GaitPos = GaitPos;
#ifdef ROC_ROBOT_GAIT_DEBUG
//    ROC_LOGI("LegNum: %d, LegStep: %d, GaitPos: %d", CurLegNum, LegStep, GaitPos);
//    ROC_LOGI("x:%.2f, y:%.2f, z:%.2f, a: %.2f", g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].X, g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Y,
//                                                g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].Z, g_RobotMoveCtrl.CurState.LegCurPos[CurLegNum].A);
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
    if(g_RobotMoveCtrl.CurState.ForceGaitStepCnt != 0)
    {
        g_RobotMoveCtrl.CurState.TravelRequest = ROC_ENABLE;
    }
    else
    {
        g_RobotMoveCtrl.CurState.TravelRequest =    (fabs(g_RobotMoveCtrl.CurState.TravelLength.X) > ROC_ROBOT_TRAVEL_DEAD_ZONE)
                                             || (fabs(g_RobotMoveCtrl.CurState.TravelLength.Z) > ROC_ROBOT_TRAVEL_DEAD_ZONE)
                                             || (fabs(g_RobotMoveCtrl.CurState.TravelLength.Y) > ROC_ROBOT_TRAVEL_DEAD_ZONE);

        if (g_RobotMoveCtrl.CurState.TravelRequest)
        {
#if 0
            if (g_RobotMoveCtrl.CurState.TravelLength.Y < 0)    // just start walking - Try to guess a good foot to start off on
            {
                g_RobotMoveCtrl.CurState.GaitStep = ((g_RobotMoveCtrl.CurState.TravelLength.X < 0) ? g_RobotMoveCtrl.CurGait.GaitLegNr[ROC_ROBOT_LEF_HIN_LEG] : g_RobotMoveCtrl.CurGait.GaitLegNr[ROC_ROBOT_RIG_HIN_LEG]);
            }
            else
            {
                 g_RobotMoveCtrl.CurState.GaitStep = ((g_RobotMoveCtrl.CurState.TravelLength.X < 0) ? g_RobotMoveCtrl.CurGait.GaitLegNr[ROC_ROBOT_LEF_FRO_LEG] : g_RobotMoveCtrl.CurGait.GaitLegNr[ROC_ROBOT_RIG_FRO_LEG]);
            }

            // And lets backup a few Gaitsteps before this to allow it to start the up swing
            g_RobotMoveCtrl.CurState.GaitStep = ((g_RobotMoveCtrl.CurState.GaitStep > g_RobotMoveCtrl.CurGait.FrontDownPos)
                                            ? (g_RobotMoveCtrl.CurState.GaitStep - g_RobotMoveCtrl.CurGait.FrontDownPos)
                                            : (g_RobotMoveCtrl.CurState.GaitStep + g_RobotMoveCtrl.CurGait.StepsInGait - g_RobotMoveCtrl.CurGait.FrontDownPos));
#endif
        }
        else
        {
            g_RobotMoveCtrl.CurState.TravelLength.X = 0;        // Clear values under the cTravelDeadZone
            g_RobotMoveCtrl.CurState.TravelLength.Y = 0;        // Gait NOT in motion, return to home position
            g_RobotMoveCtrl.CurState.TravelLength.Z = 0;
        }
    }

    g_RobotMoveCtrl.CurState.TravelRequest = ROC_ENABLE;

    for(LegIndex = 0; LegIndex < ROC_ROBOT_CNT_LEGS; LegIndex++)
    {
        RocRobotGaitPosUpdate(LegIndex);
    }

    g_RobotMoveCtrl.CurState.GaitStep++;
    if (g_RobotMoveCtrl.CurState.GaitStep > g_RobotMoveCtrl.CurGait.StepsInGait)
    {
      g_RobotMoveCtrl.CurState.GaitStep = 1;
    }

    // If we have a force count decrement it now
    if (g_RobotMoveCtrl.CurState.ForceGaitStepCnt)
    {
        g_RobotMoveCtrl.CurState.ForceGaitStepCnt--;
    }
}

/*********************************************************************************
 *  Description:
 *              Check the body rotate angle range
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.06.02)
**********************************************************************************/
static void RocBodyRotateRangeCheck(void)
{
    if(ROC_ROBOT_BODY_ROTATE_MIN_PITCH > g_RobotMoveCtrl.CurState.BodyRot.X)
    {
        g_RobotMoveCtrl.CurState.BodyRot.X = ROC_ROBOT_BODY_ROTATE_MIN_PITCH;
    }
    else if(ROC_ROBOT_BODY_ROTATE_MAX_PITCH < g_RobotMoveCtrl.CurState.BodyRot.X)
    {
        g_RobotMoveCtrl.CurState.BodyRot.X = ROC_ROBOT_BODY_ROTATE_MAX_PITCH;
    }

    if(ROC_ROBOT_BODY_ROTATE_MIN_ROLL > g_RobotMoveCtrl.CurState.BodyRot.Y)
    {
        g_RobotMoveCtrl.CurState.BodyRot.Y = ROC_ROBOT_BODY_ROTATE_MIN_ROLL;
    }
    else if(ROC_ROBOT_BODY_ROTATE_MAX_ROLL < g_RobotMoveCtrl.CurState.BodyRot.Y)
    {
        g_RobotMoveCtrl.CurState.BodyRot.Y = ROC_ROBOT_BODY_ROTATE_MAX_ROLL;
    }

    if(ROC_ROBOT_BODY_ROTATE_MIN_YAW > g_RobotMoveCtrl.CurState.BodyRot.Z)
    {
        g_RobotMoveCtrl.CurState.BodyRot.Z = ROC_ROBOT_BODY_ROTATE_MIN_YAW;
    }
    else if(ROC_ROBOT_BODY_ROTATE_MAX_YAW < g_RobotMoveCtrl.CurState.BodyRot.Z)
    {
        g_RobotMoveCtrl.CurState.BodyRot.Z = ROC_ROBOT_BODY_ROTATE_MAX_YAW;
    }
}

/*********************************************************************************
 *  Description:
 *              The robot body inverse kinematic, which be used to caculate the
 *              the coordinate of robot feet after rotating from pitch, roll, yaw
 *              axis.
 *
 *  Parameter:
 *              x: the expected X position of the robot leg tiptoe
 *              y: the expected Y position of the robot leg tiptoe
 *              z: the expected Z position of the robot leg tiptoe
 *
 *              BodyRotX: Global Input pitch of the body
 *              BodyRotY: Global Input roll of the body
 *              BodyRotZ: Global Input yaw of the body
 *              RotationY: Input Rotation for the gait
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.16)
**********************************************************************************/
static void RocBodyInverseKinematic(float x, float y, float z, ROC_ROBOT_LEG_e LegNum)
{
    float   SinA;   //Sin buffer for BodyRotX calculations
    float   CosA;   //Cos buffer for BodyRotX calculations
    float   SinB;   //Sin buffer for BodyRotX calculations
    float   CosB;   //Cos buffer for BodyRotX calculations
    float   SinG;   //Sin buffer for BodyRotZ calculations
    float   CosG;   //Cos buffer for BodyRotZ calculations
    float   CprX;   //Final X value for centerpoint of rotation
    float   CprY;   //Final Y value for centerpoint of rotation
    float   CprZ;   //Final Z value for centerpoint of rotation

    RocBodyRotateRangeCheck();

    //Calculating totals from center of the body to the feet
    switch(LegNum)
    {
        case ROC_ROBOT_RIG_FRO_LEG:
        {
            CprX = ROC_ROBOT_RIG_FRO_OFFSET_X + ROC_ROBOT_FRO_INIT_X + x - g_RobotMoveCtrl.CurState.BodyOffset.X;
            CprY = ROC_ROBOT_RIG_FRO_OFFSET_Y + ROC_ROBOT_FRO_INIT_Y + y - g_RobotMoveCtrl.CurState.BodyOffset.Y;
            CprZ = ROC_ROBOT_RIG_FRO_OFFSET_Z + ROC_ROBOT_FRO_INIT_Z + z - g_RobotMoveCtrl.CurState.BodyOffset.Z;

            break;
        }

        case ROC_ROBOT_RIG_MID_LEG:
        {
            CprX = ROC_ROBOT_RIG_MID_OFFSET_X + ROC_ROBOT_MID_INIT_X + x - g_RobotMoveCtrl.CurState.BodyOffset.X;
            CprY = ROC_ROBOT_RIG_MID_OFFSET_Y + ROC_ROBOT_MID_INIT_Y + y - g_RobotMoveCtrl.CurState.BodyOffset.Y;
            CprZ = ROC_ROBOT_RIG_MID_OFFSET_Z + ROC_ROBOT_MID_INIT_Z + z - g_RobotMoveCtrl.CurState.BodyOffset.Z;

            break;
        }

        case ROC_ROBOT_RIG_HIN_LEG:
        {
            CprX = ROC_ROBOT_RIG_HIN_OFFSET_X + ROC_ROBOT_HIN_INIT_X + x - g_RobotMoveCtrl.CurState.BodyOffset.X;
            CprY = ROC_ROBOT_RIG_HIN_OFFSET_Y - ROC_ROBOT_HIN_INIT_Y + y - g_RobotMoveCtrl.CurState.BodyOffset.Y;
            CprZ = ROC_ROBOT_RIG_HIN_OFFSET_Z + ROC_ROBOT_HIN_INIT_Z + z - g_RobotMoveCtrl.CurState.BodyOffset.Z;

            break;
        }

        case ROC_ROBOT_LEF_FRO_LEG:
        {
            CprX = ROC_ROBOT_LEF_FRO_OFFSET_X - ROC_ROBOT_FRO_INIT_X + x - g_RobotMoveCtrl.CurState.BodyOffset.X;
            CprY = ROC_ROBOT_LEF_FRO_OFFSET_Y + ROC_ROBOT_FRO_INIT_Y + y - g_RobotMoveCtrl.CurState.BodyOffset.Y;
            CprZ = ROC_ROBOT_LEF_FRO_OFFSET_Z + ROC_ROBOT_FRO_INIT_Z + z - g_RobotMoveCtrl.CurState.BodyOffset.Z;

            break;
        }

        case ROC_ROBOT_LEF_MID_LEG:
        {
            CprX = ROC_ROBOT_LEF_MID_OFFSET_X - ROC_ROBOT_MID_INIT_X + x - g_RobotMoveCtrl.CurState.BodyOffset.X;
            CprY = ROC_ROBOT_LEF_MID_OFFSET_Y + ROC_ROBOT_MID_INIT_Y + y - g_RobotMoveCtrl.CurState.BodyOffset.Y;
            CprZ = ROC_ROBOT_LEF_MID_OFFSET_Z + ROC_ROBOT_MID_INIT_Z + z - g_RobotMoveCtrl.CurState.BodyOffset.Z;

            break;
        }

        case ROC_ROBOT_LEF_HIN_LEG:
        {
            CprX = ROC_ROBOT_LEF_HIN_OFFSET_X - ROC_ROBOT_HIN_INIT_X + x - g_RobotMoveCtrl.CurState.BodyOffset.X;
            CprY = ROC_ROBOT_LEF_HIN_OFFSET_Y - ROC_ROBOT_HIN_INIT_Y + y - g_RobotMoveCtrl.CurState.BodyOffset.Y;
            CprZ = ROC_ROBOT_LEF_HIN_OFFSET_Z + ROC_ROBOT_HIN_INIT_Z + z - g_RobotMoveCtrl.CurState.BodyOffset.Z;

            break;
        }

//      CprX = x + g_RobotMoveCtrl.CurState.BodyOffset.X;
//      CprY = y + g_RobotMoveCtrl.CurState.BodyOffset.Y;
//      CprZ = z + g_RobotMoveCtrl.CurState.BodyOffset.Z;  //Define centerpoint for rotation along the Y-axis

        default:
        {
            break;
        }
    }

    //ROC_LOGI("BodyRot.X: %.2f", g_RobotMoveCtrl.CurState.BodyRot.X);
    //ROC_LOGW("CprX: %.2f, CprY: %.2f, CprZ: %.2f", CprX, CprY, CprZ);

    /*Successive global rotation matrix:
    Math shorts for rotation: Alfa [A] = Zrotate, Beta [B] = Yrotate, Gamma [G] = Xrotate
    Sinus Alfa = SinA, cosinus Alfa = cosA, and so on. */
    //First calculate sinus and cosinus for each rotation:
    SinG = Sin(g_RobotMoveCtrl.CurState.BodyRot.X * ROC_ROBOT_ANGLE_TO_RADIAN);
    CosG = Cos(g_RobotMoveCtrl.CurState.BodyRot.X * ROC_ROBOT_ANGLE_TO_RADIAN);

    SinB = Sin(g_RobotMoveCtrl.CurState.BodyRot.Y * ROC_ROBOT_ANGLE_TO_RADIAN);
    CosB = Cos(g_RobotMoveCtrl.CurState.BodyRot.Y * ROC_ROBOT_ANGLE_TO_RADIAN);

    SinA = Sin(g_RobotMoveCtrl.CurState.BodyRot.Z * ROC_ROBOT_ANGLE_TO_RADIAN);
    CosA = Cos(g_RobotMoveCtrl.CurState.BodyRot.Z * ROC_ROBOT_ANGLE_TO_RADIAN);

    //Calcualtion of rotation matrix:
    //Increment for the feet coordinate
    g_BodyIkPos[0] = (CprX - (CprX * CosA * CosB - CprY * CosB * SinA + CprZ * SinB));

    g_BodyIkPos[1] = (CprY - (CprX * CosG * SinA + CprX * CosA * SinB * SinG + CprY * CosA * CosG
                    - CprY * SinA * SinB * SinG - CprZ * CosB * SinG));

    g_BodyIkPos[2] = (CprZ - (CprX * SinA * SinG - CprX * CosA * CosG * SinB + CprY * CosA * SinG
                    + CprY * CosG * SinA * SinB + CprZ * CosB * CosG));

#ifdef ROC_ROBOT_GAIT_DEBUG
    ROC_LOGI("BodyIkPosX: %.2f, BodyIkPosY: %.2f, BodyIkPosZ: %.2f, LegNum: %d", g_BodyIkPos[0], g_BodyIkPos[1], g_BodyIkPos[2], LegNum);
#endif
}

/*********************************************************************************
 *  Description:
 *              The reverse DH algorithm, which be used to caculate the three joint
 *              rotate angle of the robot leg for the given position of the feet.
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
static void RocLegInverseKinematic(float x, float y, float z)
{
    double  a = 0;
    double  e = 0;
    double  f = 0;
    double  h = 0;
    double  j = 0;

    g_DhAngleBuffer[0] = (float)ATan(y / x) / ROC_ROBOT_ANGLE_TO_RADIAN;

    a = x * Cos(g_DhAngleBuffer[0] * ROC_ROBOT_ANGLE_TO_RADIAN) + y * Sin(g_DhAngleBuffer[0] * ROC_ROBOT_ANGLE_TO_RADIAN) - ROC_ROBOT_DH_CONST_A1;

    g_DhAngleBuffer[2] = (float)ACos((a * a + (z - ROC_ROBOT_DH_CONST_D1) * (z - ROC_ROBOT_DH_CONST_D1) - ROC_ROBOT_DH_CONST_A3 * ROC_ROBOT_DH_CONST_A3
                        - ROC_ROBOT_DH_CONST_A2 * ROC_ROBOT_DH_CONST_A2) / (2 * ROC_ROBOT_DH_CONST_A2 * ROC_ROBOT_DH_CONST_A3)) / ROC_ROBOT_ANGLE_TO_RADIAN;

    if((g_DhAngleBuffer[2] > 0) || (g_DhAngleBuffer[2] < -180))     // limit the anlge in the range of [0~(-180)]
    {
        if((g_DhAngleBuffer[2] > 0) && (g_DhAngleBuffer[2] <= 180))
        {
            g_DhAngleBuffer[2] = -g_DhAngleBuffer[2];
        }
        else if((g_DhAngleBuffer[2] > 180) && (g_DhAngleBuffer[2] <= 360))
        {
            g_DhAngleBuffer[2] = g_DhAngleBuffer[2] - 360;
        }
        else if((g_DhAngleBuffer[2] < -180) && (g_DhAngleBuffer[2] >= -360))
        {
            g_DhAngleBuffer[2] = -(g_DhAngleBuffer[2] + 360);
        }
    }

    e = Cos(g_DhAngleBuffer[2] * ROC_ROBOT_ANGLE_TO_RADIAN);
    f = Sin(g_DhAngleBuffer[2] * ROC_ROBOT_ANGLE_TO_RADIAN);
    h = (e * ROC_ROBOT_DH_CONST_A3 + ROC_ROBOT_DH_CONST_A2 ) * (z - ROC_ROBOT_DH_CONST_D1) - a * f * ROC_ROBOT_DH_CONST_A3;
    j = a * (ROC_ROBOT_DH_CONST_A3 * e + ROC_ROBOT_DH_CONST_A2) + ROC_ROBOT_DH_CONST_A3 * f * (z - ROC_ROBOT_DH_CONST_D1);

    g_DhAngleBuffer[1] = (float)ATan2(h, j) / ROC_ROBOT_ANGLE_TO_RADIAN;
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
    float   x = 0;
    float   y = 0;
    float   z = 0;

    /***********the coordinate datas of the first group legs ****************/
    x = ROC_ROBOT_FRO_INIT_X + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_FRO_LEG].X;
    y = ROC_ROBOT_FRO_INIT_Y + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_FRO_LEG].Y;
    z = ROC_ROBOT_FRO_INIT_Z + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_FRO_LEG].Z;

#ifdef ROC_ROBOT_GAIT_DEBUG
    ROC_LOGW("x:%.2f, y:%.2f, z:%.2f", x, y, z);
#endif
    RocLegInverseKinematic(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_RIG_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_RIG_FRO_HIP_CENTER + (ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_RIG_FRO_LEG_CENTER + (ROC_ROBOT_FRO_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_RIG_FRO_FET_CENTER + (ROC_ROBOT_FRO_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    if(ROC_ROBOT_WALK_MODE_QUADRUPED == g_RobotMoveCtrl.CurState.WalkMode)
    {
        x = ROC_ROBOT_MID_INIT_X;
        y = ROC_ROBOT_MID_INIT_Y;
        z = ROC_ROBOT_MID_INIT_Z + ROC_ROBOT_QUAD_MODE_FEET_LIFT;
    }
    else if(ROC_ROBOT_WALK_MODE_HEXAPOD == g_RobotMoveCtrl.CurState.WalkMode)
    {
        x = ROC_ROBOT_MID_INIT_X - g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_MID_LEG].X;
        y = ROC_ROBOT_MID_INIT_Y + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_MID_LEG].Y;
        z = ROC_ROBOT_MID_INIT_Z + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_MID_LEG].Z;
    }

    RocLegInverseKinematic(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_LEF_MID_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_LEF_MID_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_MID_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_MID_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_LEF_MID_LEG_CENTER + (ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_MID_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_LEF_MID_FET_CENTER + (-ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    x = ROC_ROBOT_HIN_INIT_X + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_HIN_LEG].X;
    y = ROC_ROBOT_HIN_INIT_Y - g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_HIN_LEG].Y;
    z = ROC_ROBOT_HIN_INIT_Z + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_HIN_LEG].Z;

    RocLegInverseKinematic(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_RIG_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_RIG_HIN_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_RIG_HIN_LEG_CENTER + (ROC_ROBOT_HIN_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_RIG_HIN_FET_CENTER + (ROC_ROBOT_HIN_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    /***********the coordinate datas of the second group legs ***************/
    x = ROC_ROBOT_FRO_INIT_X - g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_FRO_LEG].X;
    y = ROC_ROBOT_FRO_INIT_Y + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_FRO_LEG].Y;
    z = ROC_ROBOT_FRO_INIT_Z + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_FRO_LEG].Z;

    RocLegInverseKinematic(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_LEF_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_LEF_FRO_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_FRO_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_LEF_FRO_LEG_CENTER + (g_DhAngleBuffer[1] - ROC_ROBOT_FRO_LEG_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_LEF_FRO_FET_CENTER + (-ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    if(ROC_ROBOT_WALK_MODE_QUADRUPED == g_RobotMoveCtrl.CurState.WalkMode)
    {
        x = ROC_ROBOT_MID_INIT_X;
        y = ROC_ROBOT_MID_INIT_Y;
        z = ROC_ROBOT_MID_INIT_Z + ROC_ROBOT_QUAD_MODE_FEET_LIFT;
    }
    else if(ROC_ROBOT_WALK_MODE_HEXAPOD == g_RobotMoveCtrl.CurState.WalkMode)
    {
        x = ROC_ROBOT_MID_INIT_X + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_MID_LEG].X;
        y = ROC_ROBOT_MID_INIT_Y + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_MID_LEG].Y;
        z = ROC_ROBOT_MID_INIT_Z + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_MID_LEG].Z;
    }

    RocLegInverseKinematic(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_RIG_MID_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_RIG_MID_HIP_CENTER + (ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_MID_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_RIG_MID_LEG_CENTER + (-ROC_ROBOT_MID_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_MID_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_RIG_MID_FET_CENTER + (ROC_ROBOT_MID_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    x = ROC_ROBOT_HIN_INIT_X - g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_HIN_LEG].X;
    y = ROC_ROBOT_HIN_INIT_Y - g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_HIN_LEG].Y;
    z = ROC_ROBOT_HIN_INIT_Z + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_HIN_LEG].Z;

    RocLegInverseKinematic(x, y, z);

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
    float   x = 0;
    float   y = 0;
    float   z = 0;

    /***********the coordinate datas of the first group legs ****************/
    x = ROC_ROBOT_WIDTH * Cos((ROC_ROBOT_FRO_HIP_INIT_ANGLE + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_FRO_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * Sin((ROC_ROBOT_FRO_HIP_INIT_ANGLE + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_FRO_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_FRO_INIT_Z + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_MID_LEG].Z;

#ifdef ROC_ROBOT_GAIT_DEBUG
    ROC_LOGW("x:%.2f, y:%.2f, z:%.2f", x, y, z);
#endif
    RocLegInverseKinematic(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_RIG_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_RIG_FRO_HIP_CENTER + (ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_RIG_FRO_LEG_CENTER + (ROC_ROBOT_FRO_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_RIG_FRO_FET_CENTER + (ROC_ROBOT_FRO_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    x = ROC_ROBOT_WIDTH * Cos( (ROC_ROBOT_MID_HIP_INIT_ANGLE - g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_MID_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * Sin( (ROC_ROBOT_MID_HIP_INIT_ANGLE - g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_MID_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_MID_INIT_Z + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_MID_LEG].Z;

    RocLegInverseKinematic(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_LEF_MID_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_LEF_MID_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_MID_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_MID_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_LEF_MID_LEG_CENTER + (ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_MID_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_LEF_MID_FET_CENTER + (-ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    x = ROC_ROBOT_WIDTH * Cos( (ROC_ROBOT_HIN_HIP_INIT_ANGLE - g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_HIN_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * Sin( (ROC_ROBOT_HIN_HIP_INIT_ANGLE - g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_HIN_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_HIN_INIT_Z + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_HIN_LEG].Z;

    RocLegInverseKinematic(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_RIG_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_RIG_HIN_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_RIG_HIN_LEG_CENTER + (ROC_ROBOT_HIN_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_RIG_HIN_FET_CENTER + (ROC_ROBOT_HIN_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    /***********the coordinate datas of the second group legs ***************/
    x = ROC_ROBOT_WIDTH * Cos( (ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_FRO_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * Sin( (ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_FRO_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_FRO_INIT_Z + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_FRO_LEG].Z;

    RocLegInverseKinematic(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_LEF_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_LEF_FRO_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_FRO_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_LEF_FRO_LEG_CENTER + (g_DhAngleBuffer[1] - ROC_ROBOT_FRO_LEG_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_LEF_FRO_FET_CENTER + (-ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    x = ROC_ROBOT_WIDTH * Cos( (ROC_ROBOT_MID_HIP_INIT_ANGLE + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_MID_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * Sin( (ROC_ROBOT_MID_HIP_INIT_ANGLE + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_MID_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_MID_INIT_Z + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_MID_LEG].Z;

    RocLegInverseKinematic(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_RIG_MID_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_RIG_MID_HIP_CENTER + (ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_MID_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_RIG_MID_LEG_CENTER + (-ROC_ROBOT_MID_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_MID_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_RIG_MID_FET_CENTER + (ROC_ROBOT_MID_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    x = ROC_ROBOT_WIDTH * Cos( (ROC_ROBOT_HIN_HIP_INIT_ANGLE + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_HIN_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    y = ROC_ROBOT_WIDTH * Sin( (ROC_ROBOT_HIN_HIP_INIT_ANGLE + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_HIN_LEG].A) * ROC_ROBOT_ANGLE_TO_RADIAN);
    z = ROC_ROBOT_HIN_INIT_Z + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_HIN_LEG].Z;

    RocLegInverseKinematic(x, y, z);

    pRobotServo->RobotLeg[ROC_ROBOT_LEF_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_LEF_HIN_HIP_CENTER + (ROC_ROBOT_HIN_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_LEF_HIN_LEG_CENTER + (g_DhAngleBuffer[1] - ROC_ROBOT_HIN_LEG_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_LEF_HIN_FET_CENTER + (-ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
}

#ifdef ROC_ROBOT_CLOSED_LOOP_CONTROL
/*********************************************************************************
 *  Description:
 *              Check the step error value
 *
 *  Parameter:
 *              *x: the pointer to the step error value
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.16)
**********************************************************************************/
static void RocRobotStepErrorCheck(float *x)
{
    if(*x > ROC_ROBOT_STEP_ERROR_HIGH_LIMIT)
    {
        *x = ROC_ROBOT_STEP_ERROR_HIGH_LIMIT;
    }
    else if(*x < ROC_ROBOT_STEP_ERROR_LOW_LIMIT)
    {
        *x = ROC_ROBOT_STEP_ERROR_LOW_LIMIT;
    }
}

#ifdef ROC_ROBOT_CIRCLE_CORRECT_ALG
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
    float   L = 0, R = 0;
    float   C1 = 0, Alfa1 = 0;
    float   C2 = 0, Alfa2 = 0;
    float   C3 = 0, Alfa3 = 0;
    float   C4 = 0, Alfa4 = 0;
    float   C5 = 0, Alfa5 = 0;
    float   C6 = 0, Alfa6 = 0;
    float   Gamma1 = 0, Gamma2 = 0, Gamma3 = 0;

    if(LegNum == 1)
    {
        R = ROC_ROBOT_WIDTH;
        L = ROC_ROBOT_DEFAULT_LEG_STEP;

        Gamma1 = ROC_ROBOT_INIT_ANGLE_BETA_1 - DeltaAlpha;
        Gamma2 = ROC_ROBOT_INIT_ANGLE_BETA_2 - DeltaAlpha;
        Gamma3 = ROC_ROBOT_INIT_ANGLE_BETA_3 - DeltaAlpha;

        Sqrt((R * R - 2 * L * R * Cos(Gamma1 * ROC_ROBOT_ANGLE_TO_RADIAN) + L * L), &C1);
        Sqrt((R * R - 2 * L * R * Cos(Gamma2 * ROC_ROBOT_ANGLE_TO_RADIAN) + L * L), &C1);
        Sqrt((R * R - 2 * L * R * Cos(Gamma3 * ROC_ROBOT_ANGLE_TO_RADIAN) + L * L), &C1);
        Sqrt((R * R - 2 * L * R * Cos( (180 - Gamma1) * ROC_ROBOT_ANGLE_TO_RADIAN ) + L * L), &C1);
        Sqrt((R * R - 2 * L * R * Cos( (180 - Gamma2) * ROC_ROBOT_ANGLE_TO_RADIAN ) + L * L), &C1);
        Sqrt((R * R - 2 * L * R * Cos( (180 - Gamma3) * ROC_ROBOT_ANGLE_TO_RADIAN ) + L * L), &C1);

        Alfa1 = ACos((C1 * C1 + R * R - L * L) / (2 * C1 * R));
        Alfa2 = ACos((C2 * C2 + R * R - L * L) / (2 * C2 * R));
        Alfa3 = ACos((C3 * C3 + R * R - L * L) / (2 * C3 * R));
        Alfa4 = ACos((C4 * C4 + R * R - L * L) / (2 * C4 * R));
        Alfa5 = ACos((C5 * C5 + R * R - L * L) / (2 * C5 * R));
        Alfa6 = ACos((C6 * C6 + R * R - L * L) / (2 * C6 * R));
    }

    if(ROC_ROBOT_RIG_FRO_LEG == LegNum)
    {
        *x = C1 * Cos(ROC_ROBOT_INIT_ANGLE_THET_1 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN + Alfa1);
        *y = C1 * Sin(ROC_ROBOT_INIT_ANGLE_THET_1 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN + Alfa1);
    }
    else if(ROC_ROBOT_RIG_MID_LEG == LegNum)
    {
        *x = C2 * Cos(ROC_ROBOT_INIT_ANGLE_THET_2 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN + Alfa2);
        *y = C2 * Sin(ROC_ROBOT_INIT_ANGLE_THET_2 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN + Alfa2);
    }
    else if(ROC_ROBOT_RIG_HIN_LEG == LegNum)
    {
        *x = C3 * Cos(ROC_ROBOT_INIT_ANGLE_THET_3 * ROC_ROBOT_ANGLE_TO_RADIAN - DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN - Alfa3);
        *y = C3 * Sin(ROC_ROBOT_INIT_ANGLE_THET_3 * ROC_ROBOT_ANGLE_TO_RADIAN - DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN - Alfa3);
    }
    else if(ROC_ROBOT_LEF_FRO_LEG == LegNum)
    {
        *x = C4 * Cos(ROC_ROBOT_INIT_ANGLE_THET_1 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN - Alfa4);
        *y = C4 * Sin(ROC_ROBOT_INIT_ANGLE_THET_1 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN - Alfa4);
    }
    else if(ROC_ROBOT_LEF_MID_LEG == LegNum)
    {
        *x = C5 * Cos(ROC_ROBOT_INIT_ANGLE_THET_2 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN - Alfa5);
        *y = C5 * Sin(ROC_ROBOT_INIT_ANGLE_THET_2 * ROC_ROBOT_ANGLE_TO_RADIAN + DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN - Alfa5);
    }
    else if(ROC_ROBOT_LEF_HIN_LEG == LegNum)
    {
        *x = C6 * Cos(ROC_ROBOT_INIT_ANGLE_THET_3 * ROC_ROBOT_ANGLE_TO_RADIAN - DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN + Alfa6);
        *y = C6 * Sin(ROC_ROBOT_INIT_ANGLE_THET_3 * ROC_ROBOT_ANGLE_TO_RADIAN - DeltaAlpha * ROC_ROBOT_ANGLE_TO_RADIAN + Alfa6);
    }

    *z = -ROC_ROBOT_HEIGHT;
}
#endif

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
void RocRobotClosedLoopWalkCalculate(ROC_ROBOT_SERVO_s *pRobotServo)
{
    float   x = 0;
    float   y = 0;
    float   z = 0;
    float   XStepError = 0;
    static  float MaxError = 0;

    XStepError = (g_RobotMoveCtrl.CurState.CurImuAngle.Yaw - g_RobotMoveCtrl.CurState.RefImuAngle.Yaw) * ROC_ROBOT_PID_CONST_P;

    if(ROC_ROBOT_MOVE_STATUS_BAKWALKING == g_RobotMoveCtrl.CurState.MoveStatus)
    {
        XStepError = -XStepError;
    }

    if(fabs(MaxError) < fabs(XStepError))
    {
        MaxError = XStepError;
    }

    XStepError = 0;

    //RocTftLcdDrawGbk24Num(120, 200, ROC_TFT_LCD_COLOR_DEFAULT_FOR, ROC_TFT_LCD_COLOR_DEFAULT_BAK, fabs(MaxError));

    RocRobotStepErrorCheck(&XStepError);

    RocBodyInverseKinematic(g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_FRO_LEG].X,
                            g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_FRO_LEG].Y,
                            g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_FRO_LEG].Z,
                            ROC_ROBOT_RIG_FRO_LEG);

    /***********the coordinate datas of the first group legs ****************/
    x = ROC_ROBOT_FRO_INIT_X + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_FRO_LEG].X - g_BodyIkPos[0] + XStepError;
    y = ROC_ROBOT_FRO_INIT_Y + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_FRO_LEG].Y - g_BodyIkPos[1];
    z = ROC_ROBOT_FRO_INIT_Z + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_FRO_LEG].Z - g_BodyIkPos[2];

    RocLegInverseKinematic(x, y, z);

#ifdef ROC_ROBOT_GAIT_DEBUG
    ROC_LOGW("FeetInPosX: %.2f, FeetInPosY: %.2f, FeetInPosZ: %.2f", x, y, z);
#endif

    pRobotServo->RobotLeg[ROC_ROBOT_RIG_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_RIG_FRO_HIP_CENTER + (ROC_ROBOT_FRO_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_RIG_FRO_LEG_CENTER + (ROC_ROBOT_FRO_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_RIG_FRO_FET_CENTER + (ROC_ROBOT_FRO_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    RocBodyInverseKinematic(g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_MID_LEG].X,
                            g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_MID_LEG].Y,
                            g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_MID_LEG].Z,
                            ROC_ROBOT_LEF_MID_LEG);

    if(ROC_ROBOT_WALK_MODE_QUADRUPED == g_RobotMoveCtrl.CurState.WalkMode)
    {
        x = ROC_ROBOT_MID_INIT_X;
        y = ROC_ROBOT_MID_INIT_Y;
        z = ROC_ROBOT_MID_INIT_Z + ROC_ROBOT_QUAD_MODE_FEET_LIFT;
    }
    else if(ROC_ROBOT_WALK_MODE_HEXAPOD == g_RobotMoveCtrl.CurState.WalkMode)
    {
        x = ROC_ROBOT_MID_INIT_X - g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_MID_LEG].X + g_BodyIkPos[0];
        y = ROC_ROBOT_MID_INIT_Y + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_MID_LEG].Y - g_BodyIkPos[1];
        z = ROC_ROBOT_MID_INIT_Z + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_MID_LEG].Z - g_BodyIkPos[2];
    }

    RocLegInverseKinematic(x, y, z);

#ifdef ROC_ROBOT_GAIT_DEBUG
    ROC_LOGW("FeetInPosX: %.2f, FeetInPosY: %.2f, FeetInPosZ: %.2f", x, y, z);
#endif

    pRobotServo->RobotLeg[ROC_ROBOT_LEF_MID_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_LEF_MID_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_MID_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_MID_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_LEF_MID_LEG_CENTER + (ROC_ROBOT_MID_LEG_INIT_ANGLE + g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_MID_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_LEF_MID_FET_CENTER + (-ROC_ROBOT_MID_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    RocBodyInverseKinematic(g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_HIN_LEG].X,
                            g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_HIN_LEG].Y,
                            g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_HIN_LEG].Z,
                            ROC_ROBOT_RIG_HIN_LEG);

    x = ROC_ROBOT_HIN_INIT_X + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_HIN_LEG].X - g_BodyIkPos[0] + XStepError;
    y = ROC_ROBOT_HIN_INIT_Y - g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_HIN_LEG].Y + g_BodyIkPos[1];
    z = ROC_ROBOT_HIN_INIT_Z + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_HIN_LEG].Z - g_BodyIkPos[2];

    RocLegInverseKinematic(x, y, z);

#ifdef ROC_ROBOT_GAIT_DEBUG
    ROC_LOGW("FeetInPosX: %.2f, FeetInPosY: %.2f, FeetInPosZ: %.2f", x, y, z);
#endif

    pRobotServo->RobotLeg[ROC_ROBOT_RIG_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_RIG_HIN_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_HIN_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_RIG_HIN_LEG_CENTER + (ROC_ROBOT_HIN_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_RIG_HIN_FET_CENTER + (ROC_ROBOT_HIN_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    /***********the coordinate datas of the second group legs ***************/
    RocBodyInverseKinematic(g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_FRO_LEG].X,
                            g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_FRO_LEG].Y,
                            g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_FRO_LEG].Z,
                            ROC_ROBOT_LEF_FRO_LEG);

    x = ROC_ROBOT_FRO_INIT_X - g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_FRO_LEG].X + g_BodyIkPos[0] - XStepError;
    y = ROC_ROBOT_FRO_INIT_Y + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_FRO_LEG].Y - g_BodyIkPos[1];
    z = ROC_ROBOT_FRO_INIT_Z + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_FRO_LEG].Z - g_BodyIkPos[2];

    RocLegInverseKinematic(x, y, z);

#ifdef ROC_ROBOT_GAIT_DEBUG
    ROC_LOGW("FeetInPosX: %.2f, FeetInPosY: %.2f, FeetInPosZ: %.2f", x, y, z);
#endif

    pRobotServo->RobotLeg[ROC_ROBOT_LEF_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_LEF_FRO_HIP_CENTER + (g_DhAngleBuffer[0] - ROC_ROBOT_FRO_HIP_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_LEF_FRO_LEG_CENTER + (g_DhAngleBuffer[1] - ROC_ROBOT_FRO_LEG_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_FRO_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_LEF_FRO_FET_CENTER + (-ROC_ROBOT_FRO_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    RocBodyInverseKinematic(g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_MID_LEG].X,
                            g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_MID_LEG].Y,
                            g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_MID_LEG].Z,
                            ROC_ROBOT_RIG_MID_LEG);

    if(ROC_ROBOT_WALK_MODE_QUADRUPED == g_RobotMoveCtrl.CurState.WalkMode)
    {
        x = ROC_ROBOT_MID_INIT_X;
        y = ROC_ROBOT_MID_INIT_Y;
        z = ROC_ROBOT_MID_INIT_Z + ROC_ROBOT_QUAD_MODE_FEET_LIFT;
    }
    else if(ROC_ROBOT_WALK_MODE_HEXAPOD == g_RobotMoveCtrl.CurState.WalkMode)
    {
        x = ROC_ROBOT_MID_INIT_X + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_MID_LEG].X - g_BodyIkPos[0];
        y = ROC_ROBOT_MID_INIT_Y + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_MID_LEG].Y - g_BodyIkPos[1];
        z = ROC_ROBOT_MID_INIT_Z + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_RIG_MID_LEG].Z - g_BodyIkPos[2];
    }

    RocLegInverseKinematic(x, y, z);

#ifdef ROC_ROBOT_GAIT_DEBUG
    ROC_LOGW("FeetInPosX: %.2f, FeetInPosY: %.2f, FeetInPosZ: %.2f", x, y, z);
#endif

    pRobotServo->RobotLeg[ROC_ROBOT_RIG_MID_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_RIG_MID_HIP_CENTER + (ROC_ROBOT_MID_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_MID_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_RIG_MID_LEG_CENTER + (-ROC_ROBOT_MID_LEG_INIT_ANGLE - g_DhAngleBuffer[1]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_RIG_MID_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_RIG_MID_FET_CENTER + (ROC_ROBOT_MID_FET_INIT_ANGLE + g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);


    RocBodyInverseKinematic(g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_HIN_LEG].X,
                            g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_HIN_LEG].Y,
                            g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_HIN_LEG].Z,
                            ROC_ROBOT_LEF_HIN_LEG);

    x = ROC_ROBOT_HIN_INIT_X - g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_HIN_LEG].X + g_BodyIkPos[0] - XStepError;
    y = ROC_ROBOT_HIN_INIT_Y - g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_HIN_LEG].Y + g_BodyIkPos[1];
    z = ROC_ROBOT_HIN_INIT_Z + g_RobotMoveCtrl.CurState.LegCurPos[ROC_ROBOT_LEF_HIN_LEG].Z - g_BodyIkPos[2];

    RocLegInverseKinematic(x, y, z);

#ifdef ROC_ROBOT_GAIT_DEBUG
    ROC_LOGW("FeetInPosX: %.2f, FeetInPosY: %.2f, FeetInPosZ: %.2f", x, y, z);
#endif

    pRobotServo->RobotLeg[ROC_ROBOT_LEF_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_HIP_JOINT] = (int16_t)(ROC_ROBOT_LEF_HIN_HIP_CENTER + (ROC_ROBOT_HIN_HIP_INIT_ANGLE - g_DhAngleBuffer[0]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_KNEE_JOINT] = (int16_t)(ROC_ROBOT_LEF_HIN_LEG_CENTER + (g_DhAngleBuffer[1] - ROC_ROBOT_HIN_LEG_INIT_ANGLE) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
    pRobotServo->RobotLeg[ROC_ROBOT_LEF_HIN_LEG].RobotJoint[ROC_ROBOT_LEG_ANKLE_JOINT] = (int16_t)(ROC_ROBOT_LEF_HIN_FET_CENTER + (-ROC_ROBOT_HIN_FET_INIT_ANGLE - g_DhAngleBuffer[2]) * ROC_ROBOT_ROTATE_ANGLE_TO_PWM);
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
ROC_ROBOT_MOVE_CTRL_s *RocRobotCtrlInfoGet(void)
{
    return &g_RobotMoveCtrl;
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
    g_RobotMoveCtrl.CurState.MoveStatus = MoveStatus;
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
    return g_RobotMoveCtrl.CurState.MoveStatus;
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
    g_RobotMoveCtrl.CurState.TravelLength.X = x;
    g_RobotMoveCtrl.CurState.TravelLength.Y = y;
    g_RobotMoveCtrl.CurState.TravelLength.Z = z;
    g_RobotMoveCtrl.CurState.TravelLength.A = a;
    g_RobotMoveCtrl.CurState.LegLiftHeight  = h;
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
        g_RobotMoveCtrl.CurState.LegCurPos[i].X = g_RobotMoveCtrl.CurState.TravelLength.X;
        g_RobotMoveCtrl.CurState.LegCurPos[i].Y = g_RobotMoveCtrl.CurState.TravelLength.Y;
        g_RobotMoveCtrl.CurState.LegCurPos[i].Z = g_RobotMoveCtrl.CurState.TravelLength.Z;
        g_RobotMoveCtrl.CurState.LegCurPos[i].A = g_RobotMoveCtrl.CurState.TravelLength.A;
    }
}

/*********************************************************************************
 *  Description:
 *              Select which leg to control
 *
 *  Parameter:
 *              SlecetLegNum: the number of the selected leg
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.06)
**********************************************************************************/
void RocRobotSingleLegSelect(ROC_ROBOT_LEG_e SlecetLegNum)
{
    g_RobotMoveCtrl.CurState.SelectLegNum = SlecetLegNum;
}

/*********************************************************************************
 *  Description:
 *              Control the robot single leg
 *
 *  Parameter:
 *              pRobotCtrl: the pointer to the robot control structure
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.06)
**********************************************************************************/
void RocRobotSingleLegCtrl(ROC_ROBOT_SERVO_s *pRobotServo)
{
    uint8_t                 i = 0;
    static ROC_ROBOT_LEG_e  PrevSelectedLeg = ROC_ROBOT_RIG_FRO_LEG;

    if(ROC_ROBOT_CNT_LEGS == g_RobotMoveCtrl.CurState.SelectLegNum)
    {
        RocRobotCurLegPosUpdate();  /* control all the legs */
    }
    else
    {
        if(ROC_FALSE == g_RobotMoveCtrl.CurState.SelectLegIsAllDown)
        {
            g_RobotMoveCtrl.CurState.LegCurPos[PrevSelectedLeg].Z = 0;

            g_RobotMoveCtrl.CurState.LegCurPos[g_RobotMoveCtrl.CurState.SelectLegNum].Z = ROC_ROBOT_DEFAULT_FEET_LIFT * 1.5;

#ifdef ROC_ROBOT_GAIT_DEBUG
            ROC_LOGN("SelectLegNum: %u, PrevSelectedLeg: %u", g_RobotMoveCtrl.CurState.SelectLegNum, PrevSelectedLeg);
#endif
            PrevSelectedLeg = g_RobotMoveCtrl.CurState.SelectLegNum;
        }
        else if(ROC_TRUE == g_RobotMoveCtrl.CurState.SelectLegIsAllDown)
        {
            g_RobotMoveCtrl.CurState.LegCurPos[g_RobotMoveCtrl.CurState.SelectLegNum].Z = ROC_ROBOT_DEFAULT_FEET_LIFT * 1.5;
        }

#ifdef ROC_ROBOT_GAIT_DEBUG
        ROC_LOGN("LegCurPos.Z: %.2f ", g_RobotMoveCtrl.CurState.LegCurPos[g_RobotMoveCtrl.CurState.SelectLegNum].Z);
#endif
    }

    RocRobotOpenLoopWalkCalculate(pRobotServo);

    g_RobotMoveCtrl.CurState.SelectLegIsAllDown = ROC_TRUE;

    for(i = 0; i < ROC_ROBOT_CNT_LEGS; i++)
    {
        if(0 != g_RobotMoveCtrl.CurState.LegCurPos[i].Z)
        {
            g_RobotMoveCtrl.CurState.SelectLegIsAllDown = ROC_FALSE;
        }
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
        g_RobotMoveCtrl.CurState.LegCurPos[i].X = 0;
        g_RobotMoveCtrl.CurState.LegCurPos[i].Y = 0;
        g_RobotMoveCtrl.CurState.LegCurPos[i].Z = 0;
        g_RobotMoveCtrl.CurState.LegCurPos[i].A = 0;
    }

    g_RobotMoveCtrl.CurState.GaitStep = 1;
    g_RobotMoveCtrl.CurState.GaitType = ROC_ROBOT_GAIT_HEXP_MODE_TRIPOD_6;
    g_RobotMoveCtrl.CurState.WalkMode = ROC_ROBOT_WALK_MODE_HEXAPOD;
    //g_RobotMoveCtrl.CurState.GaitType = ROC_ROBOT_GAIT_QUAD_MODE_AMBLE_4;
    //g_RobotMoveCtrl.CurState.WalkMode = ROC_ROBOT_WALK_MODE_QUADRUPED;

    Ret = RocRobotGaitSelect();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Robot algorithm is in error!");
    }

    return Ret;
}


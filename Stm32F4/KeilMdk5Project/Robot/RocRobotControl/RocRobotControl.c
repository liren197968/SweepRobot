/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2018/12/16      1.0
********************************************************************************/
#include <string.h>

#include "tim.h"
#include "usart.h"
#include "inv_mpu.h"

#include "RocLog.h"
#include "RocLed.h"
#include "RocServo.h"
#include "RocMotor.h"
#include "RocBeeper.h"
#include "RocTftLcd.h"
#include "RocBattery.h"
#include "RocPca9685.h"
#include "RocMpu6050.h"
#include "RocBluetooth.h"
#include "RocRobotControl.h"


static ROC_ROBOT_CTRL_s    g_RobotCtrl = {{0}, {0}, ROC_ROBOT_RUN_MODE_HEXAPOD, {0}, NULL};

/*********************************************************************************
 *  Description:
 *              Robot init success beeper aciton
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.05.11)
**********************************************************************************/
static void RocRobotInitEndBeeperAction(void)
{
    RocBeeperBlink(4, 100);
}

/*********************************************************************************
 *  Description:
 *              Robot battery need charge beeper aciton
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.05.11)
**********************************************************************************/
static void RocRobotBatteryChargeBeeperAction(void)
{
    RocBeeperBlink(ROC_BEEPER_BLINK_FOREVER, 1000);
}

/*********************************************************************************
 *  Description:
 *              Set the robot walk mode
 *
 *  Parameter:
 *              WalkMode: the expected robot walk mode
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.16)
**********************************************************************************/
static void RocRobotRunModeSet(ROC_ROBOT_RUN_MODE_e WalkMode)
{
    g_RobotCtrl.RunMode = WalkMode;
}

/*********************************************************************************
 *  Description:
 *              Get the robot walk mode
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              The current robot walk mode
 *
 *  Author:
 *              ROC LiRen(2018.12.16)
**********************************************************************************/
static uint32_t RocRobotRunModeGet(void)
{
    return g_RobotCtrl.RunMode;
}

/*********************************************************************************
 *  Description:
 *              Set the robot control flag
 *
 *  Parameter:
 *              FlagNum: the control flag number to be setted
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.17)
**********************************************************************************/
static void RocRobotCtrlFlagSet(uint8_t FlagNum)
{
    uint8_t i = 0;

    for(i = 0; i < ROC_ROBOT_CTRL_CMD_NUM; i++)
    {
        if(FlagNum == i)
        {
            g_RobotCtrl.CtrlFlag.FlagStatus[i] = ROC_TRUE;
        }
        else
        {
            g_RobotCtrl.CtrlFlag.FlagStatus[i] = ROC_FALSE;
        }
    }
}

/*********************************************************************************
 *  Description:
 *              Get the robot control flag
 *
 *  Parameter:
 *              FlagNum: the control flag number to be getted
 *
 *  Return:
 *              The value of the control flag
 *
 *  Author:
 *              ROC LiRen(2019.04.17)
**********************************************************************************/
static uint8_t RocRobotCtrlFlagGet(uint8_t FlagNum)
{
    return g_RobotCtrl.CtrlFlag.FlagStatus[FlagNum];
}

/*********************************************************************************
 *  Description:
 *              Get current robot IMU euler angle
 *
 *  Parameter:
 *              *ImuDat: the pointer to the robot IMU data structure
 *
 *  Return:
 *              The IMU sensor get status
 *
 *  Author:
 *              ROC LiRen(2019.04.15)
**********************************************************************************/
static ROC_RESULT RocRobotImuEulerAngleGet(ROC_ROBOT_IMU_DATA_s *ImuDat)
{
    ROC_RESULT Ret = RET_OK;

    Ret = RocMpu6050EulerAngleGet(&ImuDat->Pitch, &ImuDat->Roll, &ImuDat->Yaw);
    if(RET_OK != Ret)
    {
        //ROC_LOGE("Robot get the IMU angle is in error(%d)! Be careful!", Ret); /* For the FIFO overflow problem */
    }

    return Ret;
}

/*********************************************************************************
 *  Description:
 *              Transmit the robot walk information
 *
 *  Parameter:
 *              *ImuDat: the pointer to the robot IMU data structure
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.15)
**********************************************************************************/
static void RocRemoteWaklInfoTransmit(ROC_ROBOT_IMU_DATA_s *ImuDat)
{
    uint8_t     i = 0;
    uint8_t     SendBuf[ROC_REMOTE_MAX_NUM_LEN_SEND] = {ROC_NONE};

    SendBuf[0] = 0x55;
    SendBuf[1] = 0x53;
    SendBuf[2] = (uint8_t)(ImuDat->Roll * 32768 / 180);
    SendBuf[3] = (uint8_t)(ImuDat->Roll * 32768 / 180) >> 8;
    SendBuf[4] = (uint8_t)(ImuDat->Pitch * 32768 / 180);
    SendBuf[5] = (uint8_t)(ImuDat->Pitch * 32768 / 180) >> 8;
    SendBuf[6] = (uint8_t)(ImuDat->Yaw * 32768 / 180);
    SendBuf[7] = (uint8_t)(ImuDat->Yaw * 32768 / 180) >> 8;
    SendBuf[8] = 0;
    SendBuf[9] = 0;

    for(i = 0; i < ROC_REMOTE_MAX_NUM_LEN_SEND -2; i++)
    {
        SendBuf[ROC_REMOTE_MAX_NUM_LEN_SEND - 1] |= SendBuf[i];
    }

    RocRemoteDataTransmit(SendBuf, ROC_REMOTE_MAX_NUM_LEN_SEND);
}

/*********************************************************************************
 *  Description:
 *              Draw robot motion track on LCD on real time
 *
 *  Parameter:
 *              *pRobotCurstate: the pointer to the robot current status
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
static void RocRobotMotionTrackOnLcdDraw(ROC_PHOENIX_STATE_s *pRobotCurstate)
{
    static uint16_t DrawXCor = 0;

    if(DrawXCor == (ROC_TFT_LCD_X_MAX_PIXEL - 1))
    {
        DrawXCor = 0;

        RocTftLcdAllClear(ROC_TFT_LCD_COLOR_DEFAULT_BAK);
    }

    RocTftLcdDrawPoint(DrawXCor, (uint16_t)(pRobotCurstate->CurImuAngle.Yaw + 100), ROC_TFT_LCD_COLOR_DEFAULT_FOR);

    DrawXCor++;
}
/*********************************************************************************
 *  Description:
 *              Start the measure of robot sensor
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
**********************************************************************************/
static void RocRobotSensorMeasure(void)
{
    if(ROC_ROBOT_CTRL_MEASURE_START == RocBluetoothCtrlCmd_Get())
    {

    }
    else if(ROC_ROBOT_CTRL_MEASURE_STOP == RocBluetoothCtrlCmd_Get())
    {

    }
}

/*********************************************************************************
 *  Description:
 *              Robot remote control function
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.16)
**********************************************************************************/
static void RocRobotRemoteControl(void)
{
    uint8_t RobotCtrlCmd;

    RobotCtrlCmd = RocBluetoothCtrlCmd_Get();

    switch(RobotCtrlCmd)
    {
        case ROC_ROBOT_CTRL_CMD_MOSTAND:
        {
            if(ROC_ROBOT_RUN_MODE_HEXAPOD == RocRobotRunModeGet())
            {
                g_RobotCtrl.RemoteCtrl.X = 0;
                g_RobotCtrl.RemoteCtrl.Y = 0;
                g_RobotCtrl.RemoteCtrl.Z = 0;
                g_RobotCtrl.RemoteCtrl.A = 0;
                g_RobotCtrl.RemoteCtrl.H = 0;

                RocRobotMoveStatus_Set(ROC_ROBOT_MOVE_STATUS_STANDING);

                if(ROC_FALSE == RocRobotCtrlFlagGet(0))
                {
                    RocRobotCtrlFlagSet(0);
                }
            }

            break;
        }

        case ROC_ROBOT_CTRL_CMD_FORWARD:
        {
            if(ROC_ROBOT_RUN_MODE_HEXAPOD == RocRobotRunModeGet())
            {
                g_RobotCtrl.RemoteCtrl.X = 0;
                g_RobotCtrl.RemoteCtrl.Y = ROC_ROBOT_DEFAULT_LEG_STEP;
                g_RobotCtrl.RemoteCtrl.Z = 0;
                g_RobotCtrl.RemoteCtrl.A = 0;
                g_RobotCtrl.RemoteCtrl.H = ROC_ROBOT_DEFAULT_FEET_LIFT;

                RocRobotMoveStatus_Set(ROC_ROBOT_MOVE_STATUS_FORWALKING);

                if(ROC_FALSE == RocRobotCtrlFlagGet(1))
                {
                    RocRobotCtrlFlagSet(1);

                    g_RobotCtrl.MoveCtrl->CurState.RefImuAngle = g_RobotCtrl.MoveCtrl->CurState.CurImuAngle;

                    g_RobotCtrl.MoveCtrl->CurState.BodyRot.X = -g_RobotCtrl.MoveCtrl->CurState.CurImuAngle.Pitch;
                    g_RobotCtrl.MoveCtrl->CurState.BodyRot.Y = g_RobotCtrl.MoveCtrl->CurState.CurImuAngle.Roll;
                    g_RobotCtrl.MoveCtrl->CurState.BodyRot.Z = g_RobotCtrl.MoveCtrl->CurState.CurImuAngle.Yaw;
                }
            }

            break;
        }

        case ROC_ROBOT_CTRL_CMD_BAKWARD:
        {
            if(ROC_ROBOT_RUN_MODE_HEXAPOD == RocRobotRunModeGet())
            {
                g_RobotCtrl.RemoteCtrl.X = 0;
                g_RobotCtrl.RemoteCtrl.Y = -ROC_ROBOT_DEFAULT_LEG_STEP;
                g_RobotCtrl.RemoteCtrl.Z = 0;
                g_RobotCtrl.RemoteCtrl.A = 0;
                g_RobotCtrl.RemoteCtrl.H = ROC_ROBOT_DEFAULT_FEET_LIFT;

                RocRobotMoveStatus_Set(ROC_ROBOT_MOVE_STATUS_BAKWALKING);

                if(ROC_FALSE == RocRobotCtrlFlagGet(2))
                {
                    RocRobotCtrlFlagSet(2);

                    g_RobotCtrl.MoveCtrl->CurState.RefImuAngle = g_RobotCtrl.MoveCtrl->CurState.CurImuAngle;

                    g_RobotCtrl.MoveCtrl->CurState.BodyRot.X = -g_RobotCtrl.MoveCtrl->CurState.CurImuAngle.Pitch;
                    g_RobotCtrl.MoveCtrl->CurState.BodyRot.Y = -g_RobotCtrl.MoveCtrl->CurState.CurImuAngle.Roll;
                    //g_RobotCtrl.MoveCtrl->CurState.BodyRot.Z = -g_RobotCtrl.MoveCtrl->CurState.CurImuAngle.Yaw;
                }
            }

            break;
        }

        case ROC_ROBOT_CTRL_CMD_LFCLOCK:
        {
            if(ROC_ROBOT_RUN_MODE_HEXAPOD == RocRobotRunModeGet())
            {
                g_RobotCtrl.RemoteCtrl.X = 0;
                g_RobotCtrl.RemoteCtrl.Y = 0;
                g_RobotCtrl.RemoteCtrl.Z = 0;
                g_RobotCtrl.RemoteCtrl.A = ROC_ROBOT_DEFAULT_TURN_ANGLE;
                g_RobotCtrl.RemoteCtrl.H = ROC_ROBOT_DEFAULT_FEET_LIFT;

                RocRobotMoveStatus_Set(ROC_ROBOT_MOVE_STATUS_CIRCLING);

                if(ROC_FALSE == RocRobotCtrlFlagGet(3))
                {
                    RocRobotCtrlFlagSet(3);
                }
            }

            break;
        }

        case ROC_ROBOT_CTRL_CMD_RGCLOCK:
        {
            if(ROC_ROBOT_RUN_MODE_HEXAPOD == RocRobotRunModeGet())
            {
                g_RobotCtrl.RemoteCtrl.X = 0;
                g_RobotCtrl.RemoteCtrl.Y = 0;
                g_RobotCtrl.RemoteCtrl.Z = 0;
                g_RobotCtrl.RemoteCtrl.A = -ROC_ROBOT_DEFAULT_TURN_ANGLE;
                g_RobotCtrl.RemoteCtrl.H = ROC_ROBOT_DEFAULT_FEET_LIFT;

                RocRobotMoveStatus_Set(ROC_ROBOT_MOVE_STATUS_CIRCLING);

                if(ROC_FALSE == RocRobotCtrlFlagGet(4))
                {
                    RocRobotCtrlFlagSet(4);
                }
            }

            break;
        }

        case ROC_ROBOT_CTRL_CMD_LFRWARD:
        {
            if(ROC_ROBOT_RUN_MODE_HEXAPOD == RocRobotRunModeGet())
            {
                g_RobotCtrl.RemoteCtrl.X = -ROC_ROBOT_DEFAULT_LEG_STEP;
                g_RobotCtrl.RemoteCtrl.Y = ROC_ROBOT_DEFAULT_LEG_STEP;
                g_RobotCtrl.RemoteCtrl.Z = 0;
                g_RobotCtrl.RemoteCtrl.A = 0;
                g_RobotCtrl.RemoteCtrl.H = ROC_ROBOT_DEFAULT_FEET_LIFT;

                RocRobotMoveStatus_Set(ROC_ROBOT_MOVE_STATUS_FORWALKING);

                if(ROC_FALSE == RocRobotCtrlFlagGet(5))
                {
                    RocRobotCtrlFlagSet(5);

                    g_RobotCtrl.MoveCtrl->CurState.RefImuAngle = g_RobotCtrl.MoveCtrl->CurState.CurImuAngle;

                    g_RobotCtrl.MoveCtrl->CurState.BodyRot.X = -g_RobotCtrl.MoveCtrl->CurState.CurImuAngle.Pitch;
                    g_RobotCtrl.MoveCtrl->CurState.BodyRot.Y = -g_RobotCtrl.MoveCtrl->CurState.CurImuAngle.Roll;
                    //g_RobotCtrl.MoveCtrl->CurState.BodyRot.Z = -g_RobotCtrl.MoveCtrl->CurState.CurImuAngle.Yaw;
                }
            }

            break;
        }

        case ROC_ROBOT_CTRL_CMD_RFRWARD:
        {
            if(ROC_ROBOT_RUN_MODE_HEXAPOD == RocRobotRunModeGet())
            {
                g_RobotCtrl.RemoteCtrl.X = ROC_ROBOT_DEFAULT_LEG_STEP;
                g_RobotCtrl.RemoteCtrl.Y = ROC_ROBOT_DEFAULT_LEG_STEP;
                g_RobotCtrl.RemoteCtrl.Z = 0;
                g_RobotCtrl.RemoteCtrl.A = 0;
                g_RobotCtrl.RemoteCtrl.H = ROC_ROBOT_DEFAULT_FEET_LIFT;

                RocRobotMoveStatus_Set(ROC_ROBOT_MOVE_STATUS_FORWALKING);

                if(ROC_FALSE == RocRobotCtrlFlagGet(6))
                {
                    RocRobotCtrlFlagSet(6);

                    g_RobotCtrl.MoveCtrl->CurState.RefImuAngle = g_RobotCtrl.MoveCtrl->CurState.CurImuAngle;

                    g_RobotCtrl.MoveCtrl->CurState.BodyRot.X = -g_RobotCtrl.MoveCtrl->CurState.CurImuAngle.Pitch;
                    g_RobotCtrl.MoveCtrl->CurState.BodyRot.Y = -g_RobotCtrl.MoveCtrl->CurState.CurImuAngle.Roll;
                    //g_RobotCtrl.MoveCtrl->CurState.BodyRot.Z = -g_RobotCtrl.MoveCtrl->CurState.CurImuAngle.Yaw;
                }
            }

            break;
        }

        case ROC_ROBOT_CTRL_CMD_LBKWARD:
        {
            if(ROC_ROBOT_RUN_MODE_HEXAPOD == RocRobotRunModeGet())
            {
                g_RobotCtrl.RemoteCtrl.X = -ROC_ROBOT_DEFAULT_LEG_STEP;
                g_RobotCtrl.RemoteCtrl.Y = -ROC_ROBOT_DEFAULT_LEG_STEP;
                g_RobotCtrl.RemoteCtrl.Z = 0;
                g_RobotCtrl.RemoteCtrl.A = 0;
                g_RobotCtrl.RemoteCtrl.H = ROC_ROBOT_DEFAULT_FEET_LIFT;

                RocRobotMoveStatus_Set(ROC_ROBOT_MOVE_STATUS_BAKWALKING);

                if(ROC_FALSE == RocRobotCtrlFlagGet(7))
                {
                    RocRobotCtrlFlagSet(7);

                    g_RobotCtrl.MoveCtrl->CurState.RefImuAngle = g_RobotCtrl.MoveCtrl->CurState.CurImuAngle;

                    g_RobotCtrl.MoveCtrl->CurState.BodyRot.X = -g_RobotCtrl.MoveCtrl->CurState.CurImuAngle.Pitch;
                    g_RobotCtrl.MoveCtrl->CurState.BodyRot.Y = -g_RobotCtrl.MoveCtrl->CurState.CurImuAngle.Roll;
                    //g_RobotCtrl.MoveCtrl->CurState.BodyRot.Z = -g_RobotCtrl.MoveCtrl->CurState.CurImuAngle.Yaw;
                }
            }

            break;
        }

        case ROC_ROBOT_CTRL_CMD_RBKWARD:
        {
            if(ROC_ROBOT_RUN_MODE_HEXAPOD == RocRobotRunModeGet())
            {
                g_RobotCtrl.RemoteCtrl.X = ROC_ROBOT_DEFAULT_LEG_STEP;
                g_RobotCtrl.RemoteCtrl.Y = -ROC_ROBOT_DEFAULT_LEG_STEP;
                g_RobotCtrl.RemoteCtrl.Z = 0;
                g_RobotCtrl.RemoteCtrl.A = 0;
                g_RobotCtrl.RemoteCtrl.H = ROC_ROBOT_DEFAULT_FEET_LIFT;

                RocRobotMoveStatus_Set(ROC_ROBOT_MOVE_STATUS_BAKWALKING);

                if(ROC_FALSE == RocRobotCtrlFlagGet(8))
                {
                    RocRobotCtrlFlagSet(8);

                    g_RobotCtrl.MoveCtrl->CurState.RefImuAngle = g_RobotCtrl.MoveCtrl->CurState.CurImuAngle;

                    g_RobotCtrl.MoveCtrl->CurState.BodyRot.X = -g_RobotCtrl.MoveCtrl->CurState.CurImuAngle.Pitch;
                    g_RobotCtrl.MoveCtrl->CurState.BodyRot.Y = -g_RobotCtrl.MoveCtrl->CurState.CurImuAngle.Roll;
                    //g_RobotCtrl.MoveCtrl->CurState.BodyRot.Z = -g_RobotCtrl.MoveCtrl->CurState.CurImuAngle.Yaw;
                }
            }

            break;
        }

        case ROC_ROBOT_CTRL_CMD_TURNLDR:
        {
            RocRemoteWaklInfoTransmit(&g_RobotCtrl.MoveCtrl->CurState.CurImuAngle);
            break;
        }

        case ROC_ROBOT_CTRL_CMD_TURNRDR:
        {
            break;
        }

        case ROC_ROBOT_CTRL_CMD_PARAMET:
        {
            break;
        }

        case ROC_ROBOT_CTRL_MEASURE_START:
        {
            RocRobotSensorMeasure();
            break;
        }

        default:
        {
            break;
        }
    }
}

/*********************************************************************************
 *  Description:
 *              Check robot control command is changed
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              The running state
 *
 *  Author:
 *              ROC LiRen(2019.04.06)
**********************************************************************************/
static ROC_RESULT RocRobotCtrlCmdIsChanged(void)
{
    uint8_t         CurCtrlCmd = ROC_NONE;
    static uint8_t  LastCtrlCmd = ROC_NONE;

    CurCtrlCmd = RocBluetoothCtrlCmd_Get();

    if(LastCtrlCmd != CurCtrlCmd)
    {
        LastCtrlCmd = CurCtrlCmd;

        return ROC_TRUE;
    }
    else
    {
        return ROC_FALSE;
    }
}

/*********************************************************************************
 *  Description:
 *              Switch the move context to forbid mutation action
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
static void RocRobotMoveContextSwitch(ROC_ROBOT_MOVE_CTRL_s *pRobotCtrl)
{
    uint8_t i = 0;

    for(i = 0; i < ROC_ROBOT_CNT_LEGS; i++)
    {
        pRobotCtrl->CurState.LegCurPos[i].X = pRobotCtrl->CurState.TravelLength.X / 2;
        pRobotCtrl->CurState.LegCurPos[i].Y = pRobotCtrl->CurState.TravelLength.Y / 2;
        pRobotCtrl->CurState.LegCurPos[i].Z = pRobotCtrl->CurState.TravelLength.Z / 2;
        pRobotCtrl->CurState.LegCurPos[i].A = pRobotCtrl->CurState.TravelLength.A / 2;
    }
}

#if 0
/*********************************************************************************
 *  Description:
 *              Rotate robot body for body IK test
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
static void RocRobotBodyRotateTest(void)
{
    static uint8_t RotateTimes  = 0;
    static uint8_t TestFlag = ROC_FALSE;

    RotateTimes++;

    if(16 >= RotateTimes)
    {
        if(ROC_FALSE == TestFlag)
        {
            g_RobotCtrl.MoveCtrl->CurState.BodyRot.Z = g_RobotCtrl.MoveCtrl->CurState.BodyRot.Z + 5;
        }

        if(ROC_TRUE == TestFlag)
        {
            g_RobotCtrl.MoveCtrl->CurState.BodyRot.Z = g_RobotCtrl.MoveCtrl->CurState.BodyRot.Z - 5;
        }

        if(20 == g_RobotCtrl.MoveCtrl->CurState.BodyRot.Z)
        {
            TestFlag = ROC_TRUE;
        }

        if(-20 == g_RobotCtrl.MoveCtrl->CurState.BodyRot.Z)
        {
            TestFlag = ROC_FALSE;
        }

        ROC_LOGI("BodyRot.Z: %.2f", g_RobotCtrl.MoveCtrl->CurState.BodyRot.Z);
    }
    else if((32 >= RotateTimes) && (16 < RotateTimes))
    {
        if(ROC_FALSE == TestFlag)
        {
            g_RobotCtrl.MoveCtrl->CurState.BodyRot.X = g_RobotCtrl.MoveCtrl->CurState.BodyRot.X + 4;
        }

        if(ROC_TRUE == TestFlag)
        {
            g_RobotCtrl.MoveCtrl->CurState.BodyRot.X = g_RobotCtrl.MoveCtrl->CurState.BodyRot.X - 4;
        }

        if(16 == g_RobotCtrl.MoveCtrl->CurState.BodyRot.X)
        {
            TestFlag = ROC_TRUE;
        }

        if(-16 == g_RobotCtrl.MoveCtrl->CurState.BodyRot.X)
        {
            TestFlag = ROC_FALSE;
        }

        ROC_LOGI("BodyRot.X: %.2f", g_RobotCtrl.MoveCtrl->CurState.BodyRot.X);
    }
    else if((48 >= RotateTimes) && (32 < RotateTimes))
    {
        if(ROC_FALSE == TestFlag)
        {
            g_RobotCtrl.MoveCtrl->CurState.BodyRot.Y = g_RobotCtrl.MoveCtrl->CurState.BodyRot.Y + 4;
        }

        if(ROC_TRUE == TestFlag)
        {
            g_RobotCtrl.MoveCtrl->CurState.BodyRot.Y = g_RobotCtrl.MoveCtrl->CurState.BodyRot.Y - 4;
        }

        if(16 == g_RobotCtrl.MoveCtrl->CurState.BodyRot.Y)
        {
            TestFlag = ROC_TRUE;
        }

        if(-16 == g_RobotCtrl.MoveCtrl->CurState.BodyRot.Y)
        {
            TestFlag = ROC_FALSE;
        }

        ROC_LOGI("BodyRot.Y: %.2f", g_RobotCtrl.MoveCtrl->CurState.BodyRot.Y);
    }
    else
    {
        RotateTimes = 0;
    }
}
#endif

/*********************************************************************************
 *  Description:
 *              Robot move core
 *
 *  Parameter:
 *              pRobotCtrl: the pointer to the robot control structure
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.03.31)
**********************************************************************************/
static void RocRobotMoveCtrlCore(ROC_ROBOT_MOVE_CTRL_s *pRobotCtrl)
{
    ROC_RESULT                  ChangeStatus = ROC_NONE;
    ROC_ROBOT_MOVE_STATUS_e     MoveStatus = ROC_ROBOT_MOVE_STATUS_NUM;

    RocRobotCtrlDeltaMoveCoorInput(g_RobotCtrl.RemoteCtrl.X, g_RobotCtrl.RemoteCtrl.Y,
                                   g_RobotCtrl.RemoteCtrl.Z, g_RobotCtrl.RemoteCtrl.A,
                                   g_RobotCtrl.RemoteCtrl.H);

    MoveStatus = RocRobotMoveStatus_Get();

    switch(MoveStatus)
    {
        case ROC_ROBOT_MOVE_STATUS_POWER_ON:
        {
            RocRobotSingleLegCtrl(&pRobotCtrl->CurServo);

            break;
        }

        case ROC_ROBOT_MOVE_STATUS_FORWALKING:
        case ROC_ROBOT_MOVE_STATUS_BAKWALKING:
        {
            ChangeStatus = RocRobotCtrlCmdIsChanged();
            if(ROC_TRUE == ChangeStatus)
            {
               RocRobotMoveContextSwitch(pRobotCtrl);
            }

            RocRobotGaitSeqUpdate();

#ifdef ROC_ROBOT_CLOSED_LOOP_CONTROL
            //RocRobotBodyRotateTest();

            RocRobotMotionTrackOnLcdDraw(&g_RobotCtrl.MoveCtrl->CurState);

            RocRobotClosedLoopWalkCalculate(&pRobotCtrl->CurServo);
#else
            RocRobotOpenLoopWalkCalculate(&pRobotCtrl->CurServo);
#endif

            break;
        }

        case ROC_ROBOT_MOVE_STATUS_CIRCLING:
        {
            ChangeStatus = RocRobotCtrlCmdIsChanged();
            if(ROC_TRUE == ChangeStatus)
            {
               RocRobotMoveContextSwitch(pRobotCtrl);
            }

            RocRobotGaitSeqUpdate();

            RocRobotOpenLoopCircleCalculate(&pRobotCtrl->CurServo);

            break;
        }

        default:
        {
            break;
        }
    }

    //RocServoSpeedSet(g_RobotCtrl.MoveCtrl->CurGait.NomGaitSpeed);
}

/*********************************************************************************
 *  Description:
 *              Robot run special gait sequence when power on
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
static void RocRobotPowerOnGaitSeq_Run(void)
{
    ROC_ROBOT_LEG_e SelectNum = ROC_ROBOT_CNT_LEGS;

    g_RobotCtrl.RemoteCtrl.X = 0;
    g_RobotCtrl.RemoteCtrl.Y = 0;
    g_RobotCtrl.RemoteCtrl.Z = ROC_ROBOT_DEFAULT_FEET_LIFT;
    g_RobotCtrl.RemoteCtrl.A = 0;
    g_RobotCtrl.RemoteCtrl.H = 0;
    HAL_Delay(ROC_ROBOT_RUN_SPEED_POWER_ON);

    g_RobotCtrl.RemoteCtrl.X = 0;
    g_RobotCtrl.RemoteCtrl.Y = 0;
    g_RobotCtrl.RemoteCtrl.Z = ROC_ROBOT_DEFAULT_FEET_LIFT * 1.8;
    g_RobotCtrl.RemoteCtrl.A = 0;
    g_RobotCtrl.RemoteCtrl.H = 0;
    HAL_Delay(ROC_ROBOT_RUN_SPEED_POWER_ON);

    g_RobotCtrl.RemoteCtrl.X = 0;
    g_RobotCtrl.RemoteCtrl.Y = 0;
    g_RobotCtrl.RemoteCtrl.Z = ROC_ROBOT_DEFAULT_FEET_LIFT;
    g_RobotCtrl.RemoteCtrl.A = 0;
    g_RobotCtrl.RemoteCtrl.H = 0;
    HAL_Delay(ROC_ROBOT_RUN_SPEED_POWER_ON);

    g_RobotCtrl.RemoteCtrl.X = 0;
    g_RobotCtrl.RemoteCtrl.Y = 0;
    g_RobotCtrl.RemoteCtrl.Z = 0;
    g_RobotCtrl.RemoteCtrl.A = 0;
    g_RobotCtrl.RemoteCtrl.H = 0;
    HAL_Delay(ROC_ROBOT_RUN_SPEED_POWER_ON);

    for(SelectNum = ROC_ROBOT_RIG_FRO_LEG; SelectNum < ROC_ROBOT_CNT_LEGS; SelectNum++)
    {
        RocRobotSingleLegSelect(SelectNum);
        HAL_Delay(ROC_ROBOT_RUN_SPEED_POWER_ON);
    }

    RocRobotSingleLegSelect(ROC_ROBOT_CNT_LEGS);
    HAL_Delay(ROC_ROBOT_RUN_SPEED_POWER_ON);

}

/*********************************************************************************
 *  Description:
 *              Robot control init
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.16)
**********************************************************************************/
static ROC_RESULT RocRobotControlInit(void)
{
    ROC_RESULT Ret = RET_OK;

    g_RobotCtrl.MoveCtrl = RocRobotCtrlInfoGet();

    RocRobotOpenLoopWalkCalculate(&g_RobotCtrl.MoveCtrl->CurServo);

    Ret = RocServoInit((int16_t *)(&g_RobotCtrl.MoveCtrl->CurServo));
    if(RET_OK != Ret)
    {
        ROC_LOGE("Robot hardware is in error, the system will not run!");
    }

    return Ret;
}

/*********************************************************************************
 *  Description:
 *              Robot start running
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.16)
**********************************************************************************/
static ROC_RESULT RocRobotStartRun(void)
{
    ROC_RESULT Ret = RET_OK;

    RocRobotRunModeSet(ROC_ROBOT_RUN_MODE_HEXAPOD);

    RocRobotSingleLegSelect(ROC_ROBOT_CNT_LEGS);

    g_RobotCtrl.MoveCtrl->CurGait.NomGaitSpeed = ROC_ROBOT_RUN_SPEED_POWER_ON;
    RocServoSpeedSet(g_RobotCtrl.MoveCtrl->CurGait.NomGaitSpeed);

    RocRobotPowerOnGaitSeq_Run();

    g_RobotCtrl.MoveCtrl->CurGait.NomGaitSpeed = ROC_ROBOT_RUN_SPEED_DEFAULT;
    RocServoSpeedSet(g_RobotCtrl.MoveCtrl->CurGait.NomGaitSpeed);

    RocRobotMoveStatus_Set(ROC_ROBOT_MOVE_STATUS_STANDING);

    return Ret;
}

/*********************************************************************************
 *  Description:
 *              Robot start running
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.16)
**********************************************************************************/
static ROC_RESULT RocRobotStopRun(void)
{
    ROC_RESULT Ret = RET_OK;

    RocServoOutputDisable();

    RocServoSpeedSet(0);

    Ret = RocServoTimerStop();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Robot stop servo motor in error!");
    }

    return Ret;
}

/*********************************************************************************
 *  Description:
 *              Robot control control init
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
**********************************************************************************/
void RocRobotInit(void)
{
    ROC_RESULT Ret = RET_OK;

    ROC_LOGW("############# Robot hardware version is 0.5! #############");

    RocRobotMoveStatus_Set(ROC_ROBOT_MOVE_STATUS_POWER_ON);

    Ret = RocLedInit();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Robot hardware is in error, the system will not run!");

        while(1);
    }

    Ret = RocTftLcdInit();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Robot hardware is in error, the system will not run!");

        while(1);
    }

    Ret = RocBluetoothInit();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Robot hardware is in error, the system will not run!");

        while(1);
    }

    Ret = RocBatteryInit();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Robot hardware is in error, the system will not run!");

        while(1);
    }

    Ret = RocBeeperInit();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Robot hardware is in error, the system will not run!");
    
        while(1);
    }

    Ret = RocPca9685Init();
    if(RET_OK != Ret)
    {
        RocTftLcdShowErrorMsg("PCA9685 ERROR!");

        ROC_LOGE("Robot hardware is in error, the system will not run!");

        while(1);
    }

    Ret = RocRemoteControlInit();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Robot hardware is in error, the system will not run!");

        while(1);
    }

    Ret = RocMpu6050Init();
    if(RET_OK != Ret)
    {
        if(ROC_MPU6050_INIT_ERROR == Ret)
        {
            RocTftLcdShowErrorMsg("MPU6050 INIT ERROR!");
        }
        else if(ROC_MPU6050_DMP_INIT_ERROR == Ret)
        {
            RocTftLcdShowErrorMsg("MPU6050 DMP ERROR!");
        }

        ROC_LOGE("Robot hardware is in error, the system will not run!");

        while(1);
    }

    Ret = RocRobotAlgoCtrlInit();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Robot hardware is in error, the system will not run!");
    
        while(1);
    }

    Ret = RocRobotControlInit();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Robot hardware is in error, the system will not run!");
    
        while(1);
    }

    Ret = RocMotorInit();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Robot hardware is in error, the system will not run!");

        while(1);
    }

    RocRobotInitEndBeeperAction();

    ROC_LOGI("Robot hardware init is in success, and the system start running.");

    Ret = RocRobotStartRun();
    if(RET_OK == Ret)
    {
        ROC_LOGW("############# Robot is running! Be careful! #############");
    }

    RocRobotMoveStatus_Set(ROC_ROBOT_MOVE_STATUS_STANDING);
}

/*********************************************************************************
 *  Description:
 *              Robot power on control task entry
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.10)
**********************************************************************************/
static void RocRobotPowerOnTaskEntry(void)
{
    if(ROC_TRUE == RocServoTurnIsFinshed())
    {
        RocRobotMoveCtrlCore(g_RobotCtrl.MoveCtrl);
    }

    RocServoControl((int16_t *)(&g_RobotCtrl.MoveCtrl->CurServo));
}

/*********************************************************************************
 *  Description:
 *              Robot battery check task entry
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.10)
**********************************************************************************/
static void RocBatteryCheckTaskEntry(void)
{
    if(ROC_TRUE == g_RobotCtrl.CtrlTime.BatTimeIsReady)
    {
        g_RobotCtrl.CtrlTime.BatTimeIsReady = ROC_FALSE;

        RocBatteryVoltageAdcSample();
    }

#ifndef ROC_ROBOT_CONTROL_DEBUG
    if(ROC_ROBOT_BATTERY_LIMITED_VOLTATE > RocBatteryVoltageGet())
    {
        ROC_LOGN("Battery is in low electricity! Charge it!");

        RocRobotStopRun();
        RocRobotBatteryChargeBeeperAction();
    }
#endif

#ifdef ROC_ROBOT_SENSOR_MEASURE
    {
        RocRobotSensorMeasure();
    }
#endif
}

/*********************************************************************************
 *  Description:
 *              Robot LCD display information task entry
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.10)
**********************************************************************************/
static void RocRobotLcdShowInfoEntry(void)
{
    if(ROC_TRUE == g_RobotCtrl.CtrlTime.LcdTimeIsReady)
    {
        g_RobotCtrl.CtrlTime.LcdTimeIsReady = ROC_FALSE;

        RocTftLcdDrawGbk16Num(50, 5, ROC_TFT_LCD_COLOR_WHITE, ROC_TFT_LCD_COLOR_BLUE, g_RobotCtrl.MoveCtrl->CurState.CurImuAngle.Pitch);
        RocTftLcdDrawGbk16Num(130, 5, ROC_TFT_LCD_COLOR_WHITE, ROC_TFT_LCD_COLOR_BLUE, g_RobotCtrl.MoveCtrl->CurState.CurImuAngle.Roll);
        RocTftLcdDrawGbk16Num(210, 5, ROC_TFT_LCD_COLOR_WHITE, ROC_TFT_LCD_COLOR_BLUE, g_RobotCtrl.MoveCtrl->CurState.CurImuAngle.Yaw);
    }
}

/*********************************************************************************
 *  Description:
 *              Robot control task entry
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.10)
**********************************************************************************/
static void RocRobotCtrlTaskEntry(void)
{
    if(ROC_TRUE == g_RobotCtrl.CtrlTime.CtrlTimeIsReady)
    {

#ifdef ROC_ROBOT_GAIT_DEBUG
        uint32_t CurExeTime = HAL_GetTick();
        static uint32_t LastExeTime = 0;

        RocServoOutputDisable();
        //ROC_LOGN("robot execution time interval is %d ms", CurExeTime - LastExeTime);

        LastExeTime = CurExeTime;
#endif

        RocLedToggle();

        RocRobotRemoteControl();

        if(ROC_TRUE == RocServoTurnIsFinshed())
        {
            RocRobotMoveCtrlCore(g_RobotCtrl.MoveCtrl);
        }

        RocServoControl((int16_t *)(&g_RobotCtrl.MoveCtrl->CurServo));

        g_RobotCtrl.CtrlTime.CtrlTimeIsReady = ROC_FALSE;
    }
    else
    {
#ifdef ROC_ROBOT_CLOSED_LOOP_CONTROL
        RocRobotImuEulerAngleGet(&g_RobotCtrl.MoveCtrl->CurState.CurImuAngle);
#endif
    }
}

/*********************************************************************************
 *  Description:
 *              The interrupt service handle for timer
 *
 *  Parameter:
 *              *htim: the point of the interrupt timer
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.16)
**********************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(TIM2 == htim->Instance)
    {
        RocBeeperTaskBackground();
    }
    else if(TIM6 == htim->Instance)
    {
        ROC_ROBOT_MOVE_STATUS_e     MoveStatus;

        MoveStatus = RocRobotMoveStatus_Get();

        if(ROC_ROBOT_MOVE_STATUS_POWER_ON == MoveStatus)
        {
            RocRobotPowerOnTaskEntry();
        }
        else
        {
            g_RobotCtrl.CtrlTime.CtrlTimeIsReady = ROC_TRUE;
        }
    }
    else if(TIM7 == htim->Instance)
    {
        static uint8_t TimeTick = 0;

        TimeTick++;

        g_RobotCtrl.CtrlTime.BatTimeIsReady = ROC_TRUE;

        if(ROC_ROBOT_CTRL_TIME_LCD_TICK == TimeTick)
        {
            TimeTick = 0;

            g_RobotCtrl.CtrlTime.LcdTimeIsReady = ROC_TRUE;
        }
    }
}

/*********************************************************************************
 *  Description:
 *              Robot control control while main
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
**********************************************************************************/
void RocRobotMain(void)
{
    RocRobotCtrlTaskEntry();

    RocBatteryCheckTaskEntry();

    RocRobotLcdShowInfoEntry();

    RocBluetoothRecvIsFinshed();
}


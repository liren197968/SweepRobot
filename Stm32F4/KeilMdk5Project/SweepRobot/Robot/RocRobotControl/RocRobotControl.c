#include <string.h>

#include "tim.h"

#include "RocLog.h"
#include "RocLed.h"
#include "RocServo.h"
#include "RocPca9685.h"
#include "RocAt24c02.h"
#include "RocBluetooth.h"
#include "RocRobotControl.h"
#include "RocRobotDhAlgorithm.h"


#define ROC_ROBOT_RUN_SPEED_DEFAULT     200


static uint8_t          g_StartMoveFlag = 0;
static double           g_ExpectedAngle = 0;


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
    if(TIM6 == htim->Instance)
    {
        RocLedToggle();
        RocServoControl();
    }
}

/*********************************************************************************
 *  Description:
 *              Robot right front leg control function
 *
 *  Parameter:
 *              FirstParm: control the rotate angle of the hip joint sevrvo
 *              SecndParm: control the rotate angle of the leg joint sevrvo
 *              ThirdParm: control the rotate angle of the feet joint sevrvo
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.16)
**********************************************************************************/
static void RocRobotRigForLegCtrl(int16_t FirstParm, int16_t SecndParm, int16_t ThirdParm)
{
    g_PwmExpetVal[0]    = FirstParm;
    g_PwmExpetVal[1]    = SecndParm;
    g_PwmExpetVal[2]    = ThirdParm;
}

/*********************************************************************************
 *  Description:
 *              Robot right middle leg control function
 *
 *  Parameter:
 *              FirstParm: control the rotate angle of the hip joint sevrvo
 *              SecndParm: control the rotate angle of the leg joint sevrvo
 *              ThirdParm: control the rotate angle of the feet joint sevrvo
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.16)
**********************************************************************************/
static void RocRobotRigMidLegCtrl(int16_t FirstParm, int16_t SecndParm, int16_t ThirdParm)
{
    g_PwmExpetVal[3]    = FirstParm;
    g_PwmExpetVal[4]    = SecndParm;
    g_PwmExpetVal[5]    = ThirdParm;
}

/*********************************************************************************
 *  Description:
 *              Robot right rear leg control function
 *
 *  Parameter:
 *              FirstParm: control the rotate angle of the hip joint sevrvo
 *              SecndParm: control the rotate angle of the leg joint sevrvo
 *              ThirdParm: control the rotate angle of the feet joint sevrvo
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.16)
**********************************************************************************/
static void RocRobotRigBakLegCtrl(int16_t FirstParm, int16_t SecndParm, int16_t ThirdParm)
{
    g_PwmExpetVal[6]    = FirstParm;
    g_PwmExpetVal[7]    = SecndParm;
    g_PwmExpetVal[8]    = ThirdParm;
}

/*********************************************************************************
 *  Description:
 *              Robot left front leg control function
 *
 *  Parameter:
 *              FirstParm: control the rotate angle of the hip joint sevrvo
 *              SecndParm: control the rotate angle of the leg joint sevrvo
 *              ThirdParm: control the rotate angle of the feet joint sevrvo
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.16)
**********************************************************************************/
static void RocRobotLefForLegCtrl(int16_t FirstParm, int16_t SecndParm, int16_t ThirdParm)
{
    g_PwmExpetVal[9]    = FirstParm;
    g_PwmExpetVal[10]   = SecndParm;
    g_PwmExpetVal[11]   = ThirdParm;
}

/*********************************************************************************
 *  Description:
 *              Robot left middle leg control function
 *
 *  Parameter:
 *              FirstParm: control the rotate angle of the hip joint sevrvo
 *              SecndParm: control the rotate angle of the leg joint sevrvo
 *              ThirdParm: control the rotate angle of the feet joint sevrvo
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.16)
**********************************************************************************/
static void RocRobotLefMidLegCtrl(int16_t FirstParm, int16_t SecndParm, int16_t ThirdParm)
{
    g_PwmExpetVal[12]   = FirstParm;
    g_PwmExpetVal[13]   = SecndParm;
    g_PwmExpetVal[14]   = ThirdParm;
}

/*********************************************************************************
 *  Description:
 *              Robot left rear leg control function
 *
 *  Parameter:
 *              FirstParm: control the rotate angle of the hip joint sevrvo
 *              SecndParm: control the rotate angle of the leg joint sevrvo
 *              ThirdParm: control the rotate angle of the feet joint sevrvo
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.16)
**********************************************************************************/
static void RocRobotLefBakLegCtrl(int16_t FirstParm, int16_t SecndParm, int16_t ThirdParm)
{
    g_PwmExpetVal[15]   = FirstParm;
    g_PwmExpetVal[16]   = SecndParm;
    g_PwmExpetVal[17]   = ThirdParm;
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
static void RocRobotControlInit(void)
{
    RocRobotRigForLegCtrl(ROC_ROBOT_RIG_FOR_HIP_CENTER, ROC_ROBOT_RIG_FOR_LEG_CENTER, ROC_ROBOT_RIG_FOR_FET_CENTER);
    RocRobotRigMidLegCtrl(ROC_ROBOT_RIG_MID_HIP_CENTER, ROC_ROBOT_RIG_MID_LEG_CENTER, ROC_ROBOT_RIG_MID_FET_CENTER);
    RocRobotRigBakLegCtrl(ROC_ROBOT_RIG_BAK_HIP_CENTER, ROC_ROBOT_RIG_BAK_LEG_CENTER, ROC_ROBOT_RIG_BAK_FET_CENTER);

    RocRobotLefForLegCtrl(ROC_ROBOT_LEF_FOR_HIP_CENTER, ROC_ROBOT_LEF_FOR_LEG_CENTER, ROC_ROBOT_LEF_FOR_FET_CENTER);
    RocRobotLefMidLegCtrl(ROC_ROBOT_LEF_MID_HIP_CENTER, ROC_ROBOT_LEF_MID_LEG_CENTER, ROC_ROBOT_LEF_MID_FET_CENTER);
    RocRobotLefBakLegCtrl(ROC_ROBOT_LEF_BAK_HIP_CENTER, ROC_ROBOT_LEF_BAK_LEG_CENTER, ROC_ROBOT_LEF_BAK_FET_CENTER);
}

/*********************************************************************************
 *  Description:
 *              Robot forward control function
 *
 *  Parameter:
 *              None
 *
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.16)
**********************************************************************************/
static void RocRobotForwardCtrl(void)
{
    static uint8_t      Flag = 0;

    Flag++;

    if(Flag > 4)
    {
        Flag = 1;
    }

    if(!g_StartMoveFlag)
    {
        g_StartMoveFlag = 1;
        //g_ExpectedAngle = yaw;
    }

    switch(Flag)
    {
    case 1:
        RocAdjustStepCalculate(g_ExpectedAngle, 0);

        RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotStandPwmVal[1] + ROC_ROBOT_LEG_LIFT, g_RobotStandPwmVal[2] + ROC_ROBOT_FEET_LIFT);          //第一步：第一组腿抬脚前移
        RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotStandPwmVal[13] + ROC_ROBOT_LEG_LIFT, g_RobotStandPwmVal[14] + ROC_ROBOT_FEET_LIFT);
        RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotStandPwmVal[7] + ROC_ROBOT_LEG_LIFT, g_RobotStandPwmVal[8] + ROC_ROBOT_FEET_LIFT);

        RocRobotLefForLegCtrl(g_RobotStandPwmVal[9], g_RobotStandPwmVal[10], g_RobotStandPwmVal[11]);
        RocRobotRigMidLegCtrl(g_RobotStandPwmVal[3], g_RobotStandPwmVal[4], g_RobotStandPwmVal[5]);                                            //同时，第二组腿向后滑动
        RocRobotLefBakLegCtrl(g_RobotStandPwmVal[15], g_RobotStandPwmVal[16], g_RobotStandPwmVal[17]);
        break;

    case 2:
        RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotForwardPwmVal[1], g_RobotForwardPwmVal[2]);                                             //第二步：第一组腿落地
        RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotForwardPwmVal[13], g_RobotForwardPwmVal[14]);
        RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotForwardPwmVal[7], g_RobotForwardPwmVal[8]);

        RocRobotLefForLegCtrl(g_RobotForwardPwmVal[27], g_RobotForwardPwmVal[28], g_RobotForwardPwmVal[29]);
        RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[21], g_RobotForwardPwmVal[22], g_RobotForwardPwmVal[23]);                                         //同时，第二组腿继续向后滑动
        RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[33], g_RobotForwardPwmVal[34], g_RobotForwardPwmVal[35]);
        break;

    case 3:
        RocRobotRigForLegCtrl(g_RobotStandPwmVal[0], g_RobotStandPwmVal[1], g_RobotStandPwmVal[2]);                                             //第三步：第一组腿向后滑动
        RocRobotLefMidLegCtrl(g_RobotStandPwmVal[12], g_RobotStandPwmVal[13], g_RobotStandPwmVal[14]);
        RocRobotRigBakLegCtrl(g_RobotStandPwmVal[6], g_RobotStandPwmVal[7], g_RobotStandPwmVal[8]);

        RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotStandPwmVal[10] + ROC_ROBOT_LEG_LIFT, g_RobotStandPwmVal[11] + ROC_ROBOT_FEET_LIFT);
        RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotStandPwmVal[4] + ROC_ROBOT_LEG_LIFT, g_RobotStandPwmVal[5] + ROC_ROBOT_FEET_LIFT);           //同时，第二组腿抬腿
        RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotStandPwmVal[16] + ROC_ROBOT_LEG_LIFT, g_RobotStandPwmVal[17] + ROC_ROBOT_FEET_LIFT);
        break;

    case 4:
        RocRobotRigForLegCtrl(g_RobotForwardPwmVal[18], g_RobotForwardPwmVal[19], g_RobotForwardPwmVal[20]);                                          //第四步：第一组腿继续向后滑动
        RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[30], g_RobotForwardPwmVal[31], g_RobotForwardPwmVal[32]);
        RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[24], g_RobotForwardPwmVal[25], g_RobotForwardPwmVal[26]);

        RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotForwardPwmVal[10], g_RobotForwardPwmVal[11]);
        RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotForwardPwmVal[4], g_RobotForwardPwmVal[5]);                                            //同时，第二组腿落地
        RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotForwardPwmVal[16], g_RobotForwardPwmVal[17]);
        break;

    default:
        break;
    }
}

void RocRobotRemoteControl(void)
{
    switch(g_BtRxBuffer[0])
    {
        case 'W':   RocRobotForwardCtrl();
                    break;

        case 'S':
                    break;

        case 'A':
                    break;

        case 'D':
                    break;

        default:    break;
    }
}

/*********************************************************************************
 *  Description:
 *              Robot start running
 *
 *  Parameter:
 *              None
 *
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

    RocRobotControlInit();

    RocServoSpeedSet(ROC_ROBOT_RUN_SPEED_DEFAULT);

    Ret = RocServoTimerStart();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Robot starts servo motor in error, and robot will stop running!");

        while(1);
    }

    HAL_Delay(500);

    RocPca9685Enable();

    return Ret;
}

/*********************************************************************************
 *  Description:
 *              Robot control control init
 *
 *  Parameter:
 *              None
 *
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

    ROC_LOGW("############# Robot hardware version is 0.1! #############");

    Ret = RocLedInit();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Robot hardware is in error, the system will not run!");

        while(1);
    }

    Ret = RocPca9685Init();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Robot hardware is in error, the system will not run!");

        while(1);
    }

    Ret = RocServoInit();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Robot hardware is in error, the system will not run!");

        while(1);
    }

    RocBluetoothInit();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Robot hardware is in error, the system will not run!");

        while(1);
    }

    ROC_LOGI("Robot hardware init is in success, and the system start running.");

    Ret = RocRobotStartRun();
    if(RET_OK == Ret)
    {
        ROC_LOGW("############# Robot is running! Be careful! #############");
    }
}

/*********************************************************************************
 *  Description:
 *              Robot control control while main
 *
 *  Parameter:
 *              None
 *
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
**********************************************************************************/
void RocRobotMain(void)
{
    if(g_BtRecvEnd == ROC_TRUE)
    {
        ROC_LOGI("Bluetooth receive (%d) data(%s).", g_BtRxDatLen, g_BtRxBuffer);

        RocBluetoothData_Send(g_BtRxBuffer, g_BtRxDatLen);
        HAL_Delay(100);

        memset(g_BtRxBuffer, ROC_NONE, sizeof(g_BtRxBuffer));

        g_BtRecvEnd = ROC_FALSE;
    }

}


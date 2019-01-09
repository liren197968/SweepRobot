#include <string.h>

#include "tim.h"

#include "RocLog.h"
#include "RocLed.h"
#include "RocMotor.h"
#include "RocServo.h"
#include "RocBeeper.h"
#include "RocPca9685.h"
#include "RocZmod4410.h"
#include "RocBluetooth.h"
#include "RocRobotControl.h"
#include "RocRobotDhAlgorithm.h"


#define ROC_ROBOT_RUN_SPEED_DEFAULT     300

#ifdef ROC_ROBOT_CLOSED_LOOP_CONTROL
static double           g_ExpectedAngle = 0;
#endif

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
 *              Robot stand control function
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.20)
**********************************************************************************/
static void RocRobotStandCtrl(void)
{
    RocRobotOpenLoopWalkCalculate(0, 0, g_RobotStandPwmVal);

    RocRobotRigForLegCtrl(g_RobotStandPwmVal[0], g_RobotStandPwmVal[1], g_RobotStandPwmVal[2]);
    RocRobotRigMidLegCtrl(g_RobotStandPwmVal[3], g_RobotStandPwmVal[4], g_RobotStandPwmVal[5]);
    RocRobotRigBakLegCtrl(g_RobotStandPwmVal[6], g_RobotStandPwmVal[7], g_RobotStandPwmVal[8]);

    RocRobotLefForLegCtrl(g_RobotStandPwmVal[9], g_RobotStandPwmVal[10], g_RobotStandPwmVal[11]);
    RocRobotLefMidLegCtrl(g_RobotStandPwmVal[12], g_RobotStandPwmVal[13], g_RobotStandPwmVal[14]);
    RocRobotLefBakLegCtrl(g_RobotStandPwmVal[15], g_RobotStandPwmVal[16], g_RobotStandPwmVal[17]);
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
    RocRobotStandCtrl();
}


/*********************************************************************************
 *  Description:
 *              Robot transform walk mode
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.20)
**********************************************************************************/
static void RocRobotWalkModeChangeCtrl(ROC_ROBOT_WALK_MODE_e RobotWalkMode)
{
    static uint8_t      Flag = 0;

    Flag++;
    
    if(ROC_ROBOT_RUN_GAIT_NUM < Flag)
    {
        Flag = ROC_ROBOT_RUN_GAIT_NUM;
    }

    if(ROC_ROBOT_WALK_MODE_CAR == RobotWalkMode)
    {
        switch(Flag)
        {
            case 1:
                    RocRobotOpenLoopWalkCalculate(0, ROC_ROBOT_DEFAULT_FEET_LIFT * 0.25, g_RobotForwardPwmVal);

                    RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotForwardPwmVal[1], g_RobotForwardPwmVal[2]);
                    RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotForwardPwmVal[13], g_RobotForwardPwmVal[14]);
                    RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotForwardPwmVal[7], g_RobotForwardPwmVal[8]);

                    RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotForwardPwmVal[10], g_RobotForwardPwmVal[11]);
                    RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotForwardPwmVal[4], g_RobotForwardPwmVal[5]);
                    RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotForwardPwmVal[16], g_RobotForwardPwmVal[17]);
                    break;

            case 2:
                    RocRobotOpenLoopWalkCalculate(0, ROC_ROBOT_DEFAULT_FEET_LIFT * 0.5, g_RobotForwardPwmVal);

                    RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotForwardPwmVal[1], g_RobotForwardPwmVal[2]);
                    RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotForwardPwmVal[13], g_RobotForwardPwmVal[14]);
                    RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotForwardPwmVal[7], g_RobotForwardPwmVal[8]);
                    
                    RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotForwardPwmVal[10], g_RobotForwardPwmVal[11]);
                    RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotForwardPwmVal[4], g_RobotForwardPwmVal[5]);
                    RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotForwardPwmVal[16], g_RobotForwardPwmVal[17]);
                    break;

            case 3:
                    RocRobotOpenLoopWalkCalculate(0, ROC_ROBOT_DEFAULT_FEET_LIFT * 0.75, g_RobotForwardPwmVal);

                    RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotForwardPwmVal[1], g_RobotForwardPwmVal[2]);
                    RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotForwardPwmVal[13], g_RobotForwardPwmVal[14]);
                    RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotForwardPwmVal[7], g_RobotForwardPwmVal[8]);
                    
                    RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotForwardPwmVal[10], g_RobotForwardPwmVal[11]);
                    RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotForwardPwmVal[4], g_RobotForwardPwmVal[5]);
                    RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotForwardPwmVal[16], g_RobotForwardPwmVal[17]);
                    break;

            case 4:
                    RocRobotOpenLoopWalkCalculate(0, ROC_ROBOT_DEFAULT_FEET_LIFT * 1, g_RobotForwardPwmVal);

                    RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotForwardPwmVal[1], g_RobotForwardPwmVal[2]);
                    RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotForwardPwmVal[13], g_RobotForwardPwmVal[14]);
                    RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotForwardPwmVal[7], g_RobotForwardPwmVal[8]);
                    
                    RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotForwardPwmVal[10], g_RobotForwardPwmVal[11]);
                    RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotForwardPwmVal[4], g_RobotForwardPwmVal[5]);
                    RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotForwardPwmVal[16], g_RobotForwardPwmVal[17]);
                    break;

            default:break;
        }
    }
    else if(ROC_ROBOT_WALK_MODE_CAR == RobotWalkMode)
    {
        switch(Flag)
        {
            case 1:
                    RocRobotOpenLoopWalkCalculate(0, ROC_ROBOT_DEFAULT_FEET_LIFT * 0.75, g_RobotForwardPwmVal);

                    RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotForwardPwmVal[1], g_RobotForwardPwmVal[2]);
                    RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotForwardPwmVal[13], g_RobotForwardPwmVal[14]);
                    RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotForwardPwmVal[7], g_RobotForwardPwmVal[8]);

                    RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotForwardPwmVal[10], g_RobotForwardPwmVal[11]);
                    RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotForwardPwmVal[4], g_RobotForwardPwmVal[5]);
                    RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotForwardPwmVal[16], g_RobotForwardPwmVal[17]);
                    break;

            case 2:
                    RocRobotOpenLoopWalkCalculate(0, ROC_ROBOT_DEFAULT_FEET_LIFT * 0.5, g_RobotForwardPwmVal);

                    RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotForwardPwmVal[1], g_RobotForwardPwmVal[2]);
                    RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotForwardPwmVal[13], g_RobotForwardPwmVal[14]);
                    RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotForwardPwmVal[7], g_RobotForwardPwmVal[8]);
                    
                    RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotForwardPwmVal[10], g_RobotForwardPwmVal[11]);
                    RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotForwardPwmVal[4], g_RobotForwardPwmVal[5]);
                    RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotForwardPwmVal[16], g_RobotForwardPwmVal[17]);
                    break;

            case 3:
                    RocRobotOpenLoopWalkCalculate(0, ROC_ROBOT_DEFAULT_FEET_LIFT * 0.25, g_RobotForwardPwmVal);

                    RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotForwardPwmVal[1], g_RobotForwardPwmVal[2]);
                    RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotForwardPwmVal[13], g_RobotForwardPwmVal[14]);
                    RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotForwardPwmVal[7], g_RobotForwardPwmVal[8]);
                    
                    RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotForwardPwmVal[10], g_RobotForwardPwmVal[11]);
                    RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotForwardPwmVal[4], g_RobotForwardPwmVal[5]);
                    RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotForwardPwmVal[16], g_RobotForwardPwmVal[17]);
                    break;

            case 4:
                    RocRobotOpenLoopWalkCalculate(0, ROC_ROBOT_DEFAULT_FEET_LIFT * 0, g_RobotForwardPwmVal);

                    RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotForwardPwmVal[1], g_RobotForwardPwmVal[2]);
                    RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotForwardPwmVal[13], g_RobotForwardPwmVal[14]);
                    RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotForwardPwmVal[7], g_RobotForwardPwmVal[8]);
                    
                    RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotForwardPwmVal[10], g_RobotForwardPwmVal[11]);
                    RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotForwardPwmVal[4], g_RobotForwardPwmVal[5]);
                    RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotForwardPwmVal[16], g_RobotForwardPwmVal[17]);
                    break;

            default:break;
        }
    }
}

/*********************************************************************************
 *  Description:
 *              Robot forward walk control function
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
static void RocRobotForwardWalkCtrl(void)
{
    static uint8_t      Flag = 0;

    Flag++;

    if(ROC_ROBOT_RUN_GAIT_NUM < Flag)
    {
        Flag = 1;
    }

#ifdef ROC_ROBOT_CLOSED_LOOP_CONTROL
    g_ExpectedAngle = g_ImuYawAngle;
#endif

#if defined(ROC_ROBOT_GAIT_SIX)
    switch(Flag)
    {
        case 1:
                RocRobotOpenLoopWalkCalculate(ROC_ROBOT_DEFAULT_LEG_STEP, ROC_ROBOT_DEFAULT_FEET_LIFT, g_RobotForwardPwmVal);

                RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotStandPwmVal[1] + ROC_ROBOT_DEFAULT_LEG_LIFT, g_RobotStandPwmVal[2] + ROC_ROBOT_DEFAULT_FEET_LIFT);
                RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotStandPwmVal[13] - ROC_ROBOT_DEFAULT_LEG_LIFT, g_RobotStandPwmVal[14] - ROC_ROBOT_DEFAULT_FEET_LIFT);
                RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotStandPwmVal[7] + ROC_ROBOT_DEFAULT_LEG_LIFT, g_RobotStandPwmVal[8] + ROC_ROBOT_DEFAULT_FEET_LIFT);

                RocRobotOpenLoopWalkCalculate(ROC_ROBOT_DEFAULT_LEG_STEP / 2, ROC_ROBOT_DEFAULT_LEG_STEP /2, g_RobotForwardPwmVal);

                RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotForwardPwmVal[10], g_RobotForwardPwmVal[11]);
                RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotForwardPwmVal[4], g_RobotForwardPwmVal[5]);
                RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotForwardPwmVal[16], g_RobotForwardPwmVal[17]);
                break;

        case 2: 
                RocRobotOpenLoopWalkCalculate(ROC_ROBOT_DEFAULT_LEG_STEP, ROC_ROBOT_DEFAULT_LEG_STEP, g_RobotForwardPwmVal);

                RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotForwardPwmVal[1], g_RobotForwardPwmVal[2]);
                RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotForwardPwmVal[13], g_RobotForwardPwmVal[14]);
                RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotForwardPwmVal[7], g_RobotForwardPwmVal[8]);

                RocRobotLefForLegCtrl(g_RobotStandPwmVal[9], g_RobotStandPwmVal[10], g_RobotStandPwmVal[11]);
                RocRobotRigMidLegCtrl(g_RobotStandPwmVal[3], g_RobotStandPwmVal[4], g_RobotStandPwmVal[5]);
                RocRobotLefBakLegCtrl(g_RobotStandPwmVal[15], g_RobotStandPwmVal[16], g_RobotStandPwmVal[17]);
                break;

        case 3:
                RocRobotOpenLoopWalkCalculate(ROC_ROBOT_DEFAULT_LEG_STEP / 2, ROC_ROBOT_DEFAULT_LEG_STEP /2, g_RobotForwardPwmVal);

                RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotForwardPwmVal[1], g_RobotForwardPwmVal[2]);
                RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotForwardPwmVal[13], g_RobotForwardPwmVal[14]);
                RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotForwardPwmVal[7], g_RobotForwardPwmVal[8]);

                RocRobotLefForLegCtrl(g_RobotForwardPwmVal[27], g_RobotForwardPwmVal[28], g_RobotForwardPwmVal[29]);
                RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[21], g_RobotForwardPwmVal[22], g_RobotForwardPwmVal[23]);
                RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[33], g_RobotForwardPwmVal[34], g_RobotForwardPwmVal[35]);
                break;

        case 4:
                RocRobotRigForLegCtrl(g_RobotStandPwmVal[0], g_RobotStandPwmVal[1], g_RobotStandPwmVal[2]);
                RocRobotLefMidLegCtrl(g_RobotStandPwmVal[12], g_RobotStandPwmVal[13], g_RobotStandPwmVal[14]);
                RocRobotRigBakLegCtrl(g_RobotStandPwmVal[6], g_RobotStandPwmVal[7], g_RobotStandPwmVal[8]);

                RocRobotOpenLoopWalkCalculate(ROC_ROBOT_DEFAULT_LEG_STEP, ROC_ROBOT_DEFAULT_LEG_STEP, g_RobotForwardPwmVal);

                RocRobotLefForLegCtrl(g_RobotForwardPwmVal[27], g_RobotForwardPwmVal[28], g_RobotForwardPwmVal[29]);
                RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[21], g_RobotForwardPwmVal[22], g_RobotForwardPwmVal[23]);
                RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[33], g_RobotForwardPwmVal[34], g_RobotForwardPwmVal[35]);
                break;

        case 5:
                RocRobotOpenLoopWalkCalculate(ROC_ROBOT_DEFAULT_LEG_STEP, ROC_ROBOT_DEFAULT_LEG_STEP, g_RobotForwardPwmVal);

                RocRobotRigForLegCtrl(g_RobotForwardPwmVal[18], g_RobotForwardPwmVal[19], g_RobotForwardPwmVal[20]);
                RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[30], g_RobotForwardPwmVal[31], g_RobotForwardPwmVal[32]);
                RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[24], g_RobotForwardPwmVal[25], g_RobotForwardPwmVal[26]);

                RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotStandPwmVal[10] - ROC_ROBOT_DEFAULT_LEG_LIFT, g_RobotStandPwmVal[11] - ROC_ROBOT_DEFAULT_FEET_LIFT);
                RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotStandPwmVal[4] + ROC_ROBOT_DEFAULT_LEG_LIFT, g_RobotStandPwmVal[5] + ROC_ROBOT_DEFAULT_FEET_LIFT);
                RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotStandPwmVal[16] - ROC_ROBOT_DEFAULT_LEG_LIFT, g_RobotStandPwmVal[17] - ROC_ROBOT_DEFAULT_FEET_LIFT);
                break;

        case 6:
                RocRobotRigForLegCtrl(g_RobotForwardPwmVal[18], g_RobotForwardPwmVal[19], g_RobotForwardPwmVal[20]);
                RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[30], g_RobotForwardPwmVal[31], g_RobotForwardPwmVal[32]);
                RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[24], g_RobotForwardPwmVal[25], g_RobotForwardPwmVal[26]);

                RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotForwardPwmVal[10], g_RobotForwardPwmVal[11]);
                RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotForwardPwmVal[4], g_RobotForwardPwmVal[5]);
                RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotForwardPwmVal[16], g_RobotForwardPwmVal[17]);
                break;

        default:break;
    }
#elif defined(ROC_ROBOT_GAIT_FIVE)
switch(Flag)
{
    case 1:
            RocRobotOpenLoopWalkCalculate(ROC_ROBOT_DEFAULT_LEG_STEP, ROC_ROBOT_DEFAULT_FEET_LIFT, g_RobotForwardPwmVal);

            RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotForwardPwmVal[1], g_RobotForwardPwmVal[2]);
            RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotForwardPwmVal[13], g_RobotForwardPwmVal[14]);
            RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotForwardPwmVal[7], g_RobotForwardPwmVal[8]);

            RocRobotLefForLegCtrl(g_RobotStandPwmVal[9], g_RobotStandPwmVal[10], g_RobotStandPwmVal[11]);
            RocRobotRigMidLegCtrl(g_RobotStandPwmVal[3], g_RobotStandPwmVal[4], g_RobotStandPwmVal[5]);
            RocRobotLefBakLegCtrl(g_RobotStandPwmVal[15], g_RobotStandPwmVal[16], g_RobotStandPwmVal[17]);
            break;

    case 2:
            RocRobotOpenLoopWalkCalculate(ROC_ROBOT_DEFAULT_LEG_STEP, 0, g_RobotForwardPwmVal);

            RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotForwardPwmVal[1], g_RobotForwardPwmVal[2]);
            RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotForwardPwmVal[13], g_RobotForwardPwmVal[14]);
            RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotForwardPwmVal[7], g_RobotForwardPwmVal[8]);

            RocRobotLefForLegCtrl(g_RobotForwardPwmVal[27], g_RobotForwardPwmVal[28], g_RobotForwardPwmVal[29]);
            RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[21], g_RobotForwardPwmVal[22], g_RobotForwardPwmVal[23]);
            RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[33], g_RobotForwardPwmVal[34], g_RobotForwardPwmVal[35]);
            break;

    case 3:
            RocRobotOpenLoopWalkCalculate(ROC_ROBOT_DEFAULT_LEG_STEP / 2, 0, g_RobotForwardPwmVal);

            RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotForwardPwmVal[1], g_RobotForwardPwmVal[2]);
            RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotForwardPwmVal[13], g_RobotForwardPwmVal[14]);
            RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotForwardPwmVal[7], g_RobotForwardPwmVal[8]);

            RocRobotOpenLoopWalkCalculate(ROC_ROBOT_DEFAULT_LEG_STEP, ROC_ROBOT_DEFAULT_FEET_LIFT, g_RobotForwardPwmVal);

            RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotForwardPwmVal[10], g_RobotForwardPwmVal[11]);
            RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotForwardPwmVal[4], g_RobotForwardPwmVal[5]);
            RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotForwardPwmVal[16], g_RobotForwardPwmVal[17]);
            break;

    case 4:
            RocRobotRigForLegCtrl(g_RobotStandPwmVal[0], g_RobotStandPwmVal[1], g_RobotStandPwmVal[2]);
            RocRobotLefMidLegCtrl(g_RobotStandPwmVal[12], g_RobotStandPwmVal[13], g_RobotStandPwmVal[14]);
            RocRobotRigBakLegCtrl(g_RobotStandPwmVal[6], g_RobotStandPwmVal[7], g_RobotStandPwmVal[8]);

            RocRobotOpenLoopWalkCalculate(ROC_ROBOT_DEFAULT_LEG_STEP, 0, g_RobotForwardPwmVal);

            RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotForwardPwmVal[10], g_RobotForwardPwmVal[11]);
            RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotForwardPwmVal[4], g_RobotForwardPwmVal[5]);
            RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotForwardPwmVal[16], g_RobotForwardPwmVal[17]);
            break;

    case 5:
            RocRobotOpenLoopWalkCalculate(ROC_ROBOT_DEFAULT_LEG_STEP, 0, g_RobotForwardPwmVal);

            RocRobotRigForLegCtrl(g_RobotForwardPwmVal[18], g_RobotForwardPwmVal[19], g_RobotForwardPwmVal[20]);
            RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[30], g_RobotForwardPwmVal[31], g_RobotForwardPwmVal[32]);
            RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[24], g_RobotForwardPwmVal[25], g_RobotForwardPwmVal[26]);

            RocRobotOpenLoopWalkCalculate(ROC_ROBOT_DEFAULT_LEG_STEP / 2, 0, g_RobotForwardPwmVal);

            RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotForwardPwmVal[10], g_RobotForwardPwmVal[11]);
            RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotForwardPwmVal[4], g_RobotForwardPwmVal[5]);
            RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotForwardPwmVal[16], g_RobotForwardPwmVal[17]);
            break;

    default:break;
}
#else
    switch(Flag)
    {
        case 1:
                RocRobotOpenLoopWalkCalculate(ROC_ROBOT_DEFAULT_LEG_STEP, ROC_ROBOT_DEFAULT_FEET_LIFT, g_RobotForwardPwmVal);

                RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotForwardPwmVal[1], g_RobotForwardPwmVal[2]);
                RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotForwardPwmVal[13], g_RobotForwardPwmVal[14]);
                RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotForwardPwmVal[7], g_RobotForwardPwmVal[8]);

                RocRobotLefForLegCtrl(g_RobotStandPwmVal[9], g_RobotStandPwmVal[10], g_RobotStandPwmVal[11]);
                RocRobotRigMidLegCtrl(g_RobotStandPwmVal[3], g_RobotStandPwmVal[4], g_RobotStandPwmVal[5]);
                RocRobotLefBakLegCtrl(g_RobotStandPwmVal[15], g_RobotStandPwmVal[16], g_RobotStandPwmVal[17]);
                break;

        case 2:
                RocRobotOpenLoopWalkCalculate(ROC_ROBOT_DEFAULT_LEG_STEP, 0, g_RobotForwardPwmVal);

                RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotForwardPwmVal[1], g_RobotForwardPwmVal[2]);
                RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotForwardPwmVal[13], g_RobotForwardPwmVal[14]);
                RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotForwardPwmVal[7], g_RobotForwardPwmVal[8]);

                RocRobotLefForLegCtrl(g_RobotForwardPwmVal[27], g_RobotForwardPwmVal[28], g_RobotForwardPwmVal[29]);
                RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[21], g_RobotForwardPwmVal[22], g_RobotForwardPwmVal[23]);
                RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[33], g_RobotForwardPwmVal[34], g_RobotForwardPwmVal[35]);
                break;

        case 3:
                RocRobotOpenLoopWalkCalculate(ROC_ROBOT_DEFAULT_LEG_STEP, ROC_ROBOT_DEFAULT_FEET_LIFT, g_RobotForwardPwmVal);

                RocRobotRigForLegCtrl(g_RobotStandPwmVal[0], g_RobotStandPwmVal[1], g_RobotStandPwmVal[2]);
                RocRobotLefMidLegCtrl(g_RobotStandPwmVal[12], g_RobotStandPwmVal[13], g_RobotStandPwmVal[14]);
                RocRobotRigBakLegCtrl(g_RobotStandPwmVal[6], g_RobotStandPwmVal[7], g_RobotStandPwmVal[8]);

                RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotForwardPwmVal[10], g_RobotForwardPwmVal[11]);
                RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotForwardPwmVal[4], g_RobotForwardPwmVal[5]);
                RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotForwardPwmVal[16], g_RobotForwardPwmVal[17]);
                break;

        case 4:
                RocRobotOpenLoopWalkCalculate(ROC_ROBOT_DEFAULT_LEG_STEP, 0, g_RobotForwardPwmVal);

                RocRobotRigForLegCtrl(g_RobotForwardPwmVal[18], g_RobotForwardPwmVal[19], g_RobotForwardPwmVal[20]);
                RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[30], g_RobotForwardPwmVal[31], g_RobotForwardPwmVal[32]);
                RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[24], g_RobotForwardPwmVal[25], g_RobotForwardPwmVal[26]);

                RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotForwardPwmVal[10], g_RobotForwardPwmVal[11]);
                RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotForwardPwmVal[4], g_RobotForwardPwmVal[5]);
                RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotForwardPwmVal[16], g_RobotForwardPwmVal[17]);
                break;

        default:break;
    }
#endif
}

/*********************************************************************************
 *  Description:
 *              Robot backword walk control function
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
static void RocRobotBackwardWalkCtrl(void)
{
    static uint8_t      Flag = 0;

    Flag++;

    if(ROC_ROBOT_RUN_GAIT_NUM < Flag)
    {
        Flag = 1;
    }

    switch(Flag)
    {
        case 1:
                RocRobotOpenLoopWalkCalculate(-ROC_ROBOT_DEFAULT_LEG_STEP, ROC_ROBOT_DEFAULT_FEET_LIFT, g_RobotForwardPwmVal);

                RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotForwardPwmVal[1], g_RobotForwardPwmVal[2]);
                RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotForwardPwmVal[13], g_RobotForwardPwmVal[14]);
                RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotForwardPwmVal[7], g_RobotForwardPwmVal[8]);

                RocRobotLefForLegCtrl(g_RobotStandPwmVal[9], g_RobotStandPwmVal[10], g_RobotStandPwmVal[11]);
                RocRobotRigMidLegCtrl(g_RobotStandPwmVal[3], g_RobotStandPwmVal[4], g_RobotStandPwmVal[5]);
                RocRobotLefBakLegCtrl(g_RobotStandPwmVal[15], g_RobotStandPwmVal[16], g_RobotStandPwmVal[17]);
                break;

        case 2:
                RocRobotOpenLoopWalkCalculate(-ROC_ROBOT_DEFAULT_LEG_STEP, 0, g_RobotForwardPwmVal);

                RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotForwardPwmVal[1], g_RobotForwardPwmVal[2]);
                RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotForwardPwmVal[13], g_RobotForwardPwmVal[14]);
                RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotForwardPwmVal[7], g_RobotForwardPwmVal[8]);

                RocRobotLefForLegCtrl(g_RobotForwardPwmVal[27], g_RobotForwardPwmVal[28], g_RobotForwardPwmVal[29]);
                RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[21], g_RobotForwardPwmVal[22], g_RobotForwardPwmVal[23]);
                RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[33], g_RobotForwardPwmVal[34], g_RobotForwardPwmVal[35]);
                break;

        case 3:
                RocRobotOpenLoopWalkCalculate(-ROC_ROBOT_DEFAULT_LEG_STEP, ROC_ROBOT_DEFAULT_FEET_LIFT, g_RobotForwardPwmVal);

                RocRobotRigForLegCtrl(g_RobotStandPwmVal[0], g_RobotStandPwmVal[1], g_RobotStandPwmVal[2]);
                RocRobotLefMidLegCtrl(g_RobotStandPwmVal[12], g_RobotStandPwmVal[13], g_RobotStandPwmVal[14]);
                RocRobotRigBakLegCtrl(g_RobotStandPwmVal[6], g_RobotStandPwmVal[7], g_RobotStandPwmVal[8]);

                RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotForwardPwmVal[10], g_RobotForwardPwmVal[11]);
                RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotForwardPwmVal[4], g_RobotForwardPwmVal[5]);
                RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotForwardPwmVal[16], g_RobotForwardPwmVal[17]);
                break;

        case 4:
                RocRobotOpenLoopWalkCalculate(-ROC_ROBOT_DEFAULT_LEG_STEP, 0, g_RobotForwardPwmVal);

                RocRobotRigForLegCtrl(g_RobotForwardPwmVal[18], g_RobotForwardPwmVal[19], g_RobotForwardPwmVal[20]);
                RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[30], g_RobotForwardPwmVal[31], g_RobotForwardPwmVal[32]);
                RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[24], g_RobotForwardPwmVal[25], g_RobotForwardPwmVal[26]);

                RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotForwardPwmVal[10], g_RobotForwardPwmVal[11]);
                RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotForwardPwmVal[4], g_RobotForwardPwmVal[5]);
                RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotForwardPwmVal[16], g_RobotForwardPwmVal[17]);
                break;

        default:break;
    }
}

/*********************************************************************************
 *  Description:
 *              Robot counterclockwise walk control function
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
static void RocRobotCounterclockwiseWalkCtrl(void)
{
    static uint8_t      Flag = 0;

    Flag++;

    if(ROC_ROBOT_RUN_GAIT_NUM < Flag)
    {
        Flag = 1;
    }

    switch(Flag)
    {
        case 1:
                RocRobotOpenLoopCircleCalculate(ROC_ROBOT_DEFAULT_LEG_ANGLE, ROC_ROBOT_DEFAULT_FEET_LIFT, g_RobotForwardPwmVal);

                RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotForwardPwmVal[1], g_RobotForwardPwmVal[2]);
                RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotForwardPwmVal[13], g_RobotForwardPwmVal[14]);
                RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotForwardPwmVal[7], g_RobotForwardPwmVal[8]);

                RocRobotLefForLegCtrl(g_RobotStandPwmVal[9], g_RobotStandPwmVal[10], g_RobotStandPwmVal[11]);
                RocRobotRigMidLegCtrl(g_RobotStandPwmVal[3], g_RobotStandPwmVal[4], g_RobotStandPwmVal[5]);
                RocRobotLefBakLegCtrl(g_RobotStandPwmVal[15], g_RobotStandPwmVal[16], g_RobotStandPwmVal[17]);
                break;

        case 2:
                RocRobotOpenLoopCircleCalculate(ROC_ROBOT_DEFAULT_LEG_ANGLE, 0, g_RobotForwardPwmVal);

                RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotForwardPwmVal[1], g_RobotForwardPwmVal[2]);
                RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotForwardPwmVal[13], g_RobotForwardPwmVal[14]);
                RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotForwardPwmVal[7], g_RobotForwardPwmVal[8]);

                RocRobotLefForLegCtrl(g_RobotForwardPwmVal[27], g_RobotForwardPwmVal[28], g_RobotForwardPwmVal[29]);
                RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[21], g_RobotForwardPwmVal[22], g_RobotForwardPwmVal[23]);
                RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[33], g_RobotForwardPwmVal[34], g_RobotForwardPwmVal[35]);
                break;

        case 3:
                RocRobotOpenLoopCircleCalculate(ROC_ROBOT_DEFAULT_LEG_ANGLE, ROC_ROBOT_DEFAULT_FEET_LIFT, g_RobotForwardPwmVal);

                RocRobotRigForLegCtrl(g_RobotStandPwmVal[0], g_RobotStandPwmVal[1], g_RobotStandPwmVal[2]);
                RocRobotLefMidLegCtrl(g_RobotStandPwmVal[12], g_RobotStandPwmVal[13], g_RobotStandPwmVal[14]);
                RocRobotRigBakLegCtrl(g_RobotStandPwmVal[6], g_RobotStandPwmVal[7], g_RobotStandPwmVal[8]);

                RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotForwardPwmVal[10], g_RobotForwardPwmVal[11]);
                RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotForwardPwmVal[4], g_RobotForwardPwmVal[5]);
                RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotForwardPwmVal[16], g_RobotForwardPwmVal[17]);
                break;

        case 4:
                RocRobotOpenLoopCircleCalculate(ROC_ROBOT_DEFAULT_LEG_ANGLE, 0, g_RobotForwardPwmVal);

                RocRobotRigForLegCtrl(g_RobotForwardPwmVal[18], g_RobotForwardPwmVal[19], g_RobotForwardPwmVal[20]);
                RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[30], g_RobotForwardPwmVal[31], g_RobotForwardPwmVal[32]);
                RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[24], g_RobotForwardPwmVal[25], g_RobotForwardPwmVal[26]);

                RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotForwardPwmVal[10], g_RobotForwardPwmVal[11]);
                RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotForwardPwmVal[4], g_RobotForwardPwmVal[5]);
                RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotForwardPwmVal[16], g_RobotForwardPwmVal[17]);
                break;

        default:break;
    }
}

/*********************************************************************************
 *  Description:
 *              Robot clockwise walk control function
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
static void RocRobotClockwiseWalkCtrl(void)
{
    static uint8_t      Flag = 0;

    Flag++;

    if(ROC_ROBOT_RUN_GAIT_NUM < Flag)
    {
        Flag = 1;
    }

    switch(Flag)
    {
        case 1:
                RocRobotOpenLoopCircleCalculate(-ROC_ROBOT_DEFAULT_LEG_ANGLE, ROC_ROBOT_DEFAULT_FEET_LIFT, g_RobotForwardPwmVal);

                RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotForwardPwmVal[1], g_RobotForwardPwmVal[2]);
                RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotForwardPwmVal[13], g_RobotForwardPwmVal[14]);
                RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotForwardPwmVal[7], g_RobotForwardPwmVal[8]);

                RocRobotLefForLegCtrl(g_RobotStandPwmVal[9], g_RobotStandPwmVal[10], g_RobotStandPwmVal[11]);
                RocRobotRigMidLegCtrl(g_RobotStandPwmVal[3], g_RobotStandPwmVal[4], g_RobotStandPwmVal[5]);
                RocRobotLefBakLegCtrl(g_RobotStandPwmVal[15], g_RobotStandPwmVal[16], g_RobotStandPwmVal[17]);
                break;

        case 2:
                RocRobotOpenLoopCircleCalculate(-ROC_ROBOT_DEFAULT_LEG_ANGLE, 0, g_RobotForwardPwmVal);

                RocRobotRigForLegCtrl(g_RobotForwardPwmVal[0], g_RobotForwardPwmVal[1], g_RobotForwardPwmVal[2]);
                RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[12], g_RobotForwardPwmVal[13], g_RobotForwardPwmVal[14]);
                RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[6], g_RobotForwardPwmVal[7], g_RobotForwardPwmVal[8]);

                RocRobotLefForLegCtrl(g_RobotForwardPwmVal[27], g_RobotForwardPwmVal[28], g_RobotForwardPwmVal[29]);
                RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[21], g_RobotForwardPwmVal[22], g_RobotForwardPwmVal[23]);
                RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[33], g_RobotForwardPwmVal[34], g_RobotForwardPwmVal[35]);
                break;

        case 3:
                RocRobotOpenLoopCircleCalculate(-ROC_ROBOT_DEFAULT_LEG_ANGLE, ROC_ROBOT_DEFAULT_FEET_LIFT, g_RobotForwardPwmVal);

                RocRobotRigForLegCtrl(g_RobotStandPwmVal[0], g_RobotStandPwmVal[1], g_RobotStandPwmVal[2]);
                RocRobotLefMidLegCtrl(g_RobotStandPwmVal[12], g_RobotStandPwmVal[13], g_RobotStandPwmVal[14]);
                RocRobotRigBakLegCtrl(g_RobotStandPwmVal[6], g_RobotStandPwmVal[7], g_RobotStandPwmVal[8]);

                RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotForwardPwmVal[10], g_RobotForwardPwmVal[11]);
                RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotForwardPwmVal[4], g_RobotForwardPwmVal[5]);
                RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotForwardPwmVal[16], g_RobotForwardPwmVal[17]);
                break;

        case 4:
                RocRobotOpenLoopCircleCalculate(-ROC_ROBOT_DEFAULT_LEG_ANGLE, 0, g_RobotForwardPwmVal);

                RocRobotRigForLegCtrl(g_RobotForwardPwmVal[18], g_RobotForwardPwmVal[19], g_RobotForwardPwmVal[20]);
                RocRobotLefMidLegCtrl(g_RobotForwardPwmVal[30], g_RobotForwardPwmVal[31], g_RobotForwardPwmVal[32]);
                RocRobotRigBakLegCtrl(g_RobotForwardPwmVal[24], g_RobotForwardPwmVal[25], g_RobotForwardPwmVal[26]);

                RocRobotLefForLegCtrl(g_RobotForwardPwmVal[9], g_RobotForwardPwmVal[10], g_RobotForwardPwmVal[11]);
                RocRobotRigMidLegCtrl(g_RobotForwardPwmVal[3], g_RobotForwardPwmVal[4], g_RobotForwardPwmVal[5]);
                RocRobotLefBakLegCtrl(g_RobotForwardPwmVal[15], g_RobotForwardPwmVal[16], g_RobotForwardPwmVal[17]);
                break;

        default:break;
    }
}

/*********************************************************************************
 *  Description:
 *              Robot remote control function
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
void RocRobotRemoteControl(void)
{
    uint8_t RobotCtrlCmd;

    RobotCtrlCmd = RocBluetoothCtrlCmd_Get();

    switch(RobotCtrlCmd)
    {
        case ROC_ROBOT_CTRL_CMD_MOSTAND:    RocRobotStandCtrl();
                                            break;

        case ROC_ROBOT_CTRL_CMD_FORWARD:    RocRobotForwardWalkCtrl();
                                            break;

        case ROC_ROBOT_CTRL_CMD_BAKWARD:    RocRobotBackwardWalkCtrl();
                                            break;

        case ROC_ROBOT_CTRL_CMD_LFCLOCK:    RocRobotCounterclockwiseWalkCtrl();
                                            break;

        case ROC_ROBOT_CTRL_CMD_RGCLOCK:    RocRobotClockwiseWalkCtrl();
                                            break;

#ifdef ROC_ROBOT_GAIT_DEBUG
        case ROC_ROBOT_CTRL_CMD_PARAMET:    RocRobotForwardWalkCtrl();
                                            RocBluetoothCtrlCmd_Set(ROC_NONE);
                                            break;
#endif

        case ROC_ROBOT_CTRL_CMD_CARFORD:    RocMotorRotateDirectionSet(ROC_MOTOR_FORWARD_ROTATE);
                                            break;

        case ROC_ROBOT_CTRL_CMD_CARBAKD:    RocMotorRotateDirectionSet(ROC_MOTOR_REVERSE_ROTATE);
                                            break;

        case ROC_ROBOT_CTRL_CMD_CARMODE:    RocRobotWalkModeChangeCtrl(ROC_ROBOT_WALK_MODE_CAR);
                                            break;

        case ROC_ROBOT_CTRL_CMD_ROTMODE:    RocRobotWalkModeChangeCtrl(ROC_ROBOT_WALK_MODE_HEXAPOD);
                                            break;

        default:                            RocMotorRotateDirectionSet(ROC_MOTOR_STOPPED_ROTATE);
                                            break;
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

    HAL_Delay(ROC_ROBOT_RUN_SPEED_DEFAULT);

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

    ROC_LOGW("############# Robot hardware version is 0.5! #############");

    Ret = RocLedInit();
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

    Ret = RocZmod4410Init();
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
        ROC_LOGE("Robot hardware is in error, the system will not run!");

        while(1);
    }

    Ret = RocServoInit();
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

        g_BtRecvEnd = ROC_FALSE;
    }

    if(ROC_ROBOT_CTRL_MEASURE_START == RocBluetoothCtrlCmd_Get())
    {
        //RocZmode4410MeasureStart();

        if(ROC_TRUE == RocZmod4410SensorStatusIsChange())
        {
            RocBeeperBlink(4, 800);
        }
    }
    else
    {
        RocZmode4410MeasureStop();
    }
}


/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2018/12/15      1.0
********************************************************************************/
#include "tim.h"

#include "RocLog.h"
#include "RocServo.h"
#include "RocPca9685.h"
#include "RocRobotControl.h"


int16_t             g_PwmExpetVal[ROC_SERVO_MAX_SUPPORT_NUM] = {0};

static int16_t      g_PwmIncreVal[ROC_SERVO_MAX_SUPPORT_NUM] = {0};
static int16_t      g_PwmPreseVal[ROC_SERVO_MAX_SUPPORT_NUM] = {0};
static int16_t      g_PwmLastdVal[ROC_SERVO_MAX_SUPPORT_NUM] = {0};


/*********************************************************************************
 *  Description:
 *              Calculate the increment of the servo PWM pulse
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
static void RocServoPwmIncreCalculate(void)
{
    uint8_t         i = 0U;

    for(i = 0U; i < ROC_SERVO_MAX_SUPPORT_NUM; i++)
    {
        if(g_PwmExpetVal[i] < ROC_SERVO_MIN_PWM_VAL)
        {
            ROC_LOGN("Servo(%d) input PWM value(%d) is less than ROC_SERVO_MIN_PWM_VAL! Be careful!", i, g_PwmExpetVal[i]);

            g_PwmExpetVal[i] = ROC_SERVO_MIN_PWM_VAL;
        }
        else if(g_PwmExpetVal[i] > ROC_SERVO_MAX_PWM_VAL)
        {
            ROC_LOGN("Servo(%d) input PWM value(%d) is more than ROC_SERVO_MAX_PWM_VAL! Be careful!", i, g_PwmExpetVal[i]);

            g_PwmExpetVal[i] = ROC_SERVO_MAX_PWM_VAL;
        }

        g_PwmIncreVal[i] = (g_PwmExpetVal[i] - g_PwmLastdVal[i]) / ROC_SERVO_SPEED_DIV_STP;
    }
}

/*********************************************************************************
 *  Description:
 *              Update the next loading date of the servo PWM pulse
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
static void RocServoPwmUpdate(uint8_t CountTimes)
{
    uint8_t         i = 0U;

    if(CountTimes < ROC_SERVO_SPEED_DIV_STP)
    {
        for(i = 0U; i < ROC_SERVO_MAX_SUPPORT_NUM; i++)
        {
            g_PwmPreseVal[i] = g_PwmPreseVal[i] + g_PwmIncreVal[i];
        }
    }
    else
    {
        for(i = 0U; i < ROC_SERVO_MAX_SUPPORT_NUM; i++)
        {
            g_PwmPreseVal[i] = g_PwmExpetVal[i];
        }
    }
}

/*********************************************************************************
 *  Description:
 *              Record the previous date of the servo PWM pulse
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
static void RocServoPwmRecod(void)
{
    uint8_t         i = 0U;

    for(i = 0U; i < ROC_SERVO_MAX_SUPPORT_NUM; i++)
    {
        g_PwmLastdVal[i] = g_PwmPreseVal[i];
    }
}

/*********************************************************************************
 *  Description:
 *              Output the servo PWM pulse
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
static void RocServoPwmOut(void)
{
    uint8_t             i = 0U;
    HAL_StatusTypeDef   WriteStatus = HAL_OK;

    for(i = 0U; i < ROC_SERVO_MAX_SUPPORT_NUM; i++)
    {
        WriteStatus = RocPca9685OutPwm(PWM_ADDRESS_L, i, 0U, (uint16_t)g_PwmPreseVal[i]);

        if(ROC_PCA9685_MAX_NUM <= i)
        {
            WriteStatus = RocPca9685OutPwm(PWM_ADDRESS_H, i - ROC_PCA9685_MAX_NUM, 0U, (uint16_t)g_PwmPreseVal[i]);
        }
    }

    if(HAL_OK != WriteStatus)
    {
        ROC_LOGE("Servo PWM out is in error, and servo will stop running!");
        while(1);
    }
}

/*********************************************************************************
 *  Description:
 *              Control the running of all servos: if the times is eaqule to the
 *              expected, load the next group servo data, using this way to control
 *              the speed of servo.
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
void RocServoControl(void)
{
    static uint8_t  RefreshTimes = 0U;

    RefreshTimes++;     /* record the times of the data update of servo */

    RocServoPwmUpdate(RefreshTimes);

#ifdef ROC_ROBOT_SERVO_DEBUG
    ROC_LOGW("RefreshTimes is %d, g_PwmPreseVal is %d, g_PwmExpetVal is %d, g_PwmIncreVal is %d",
                            RefreshTimes, g_PwmPreseVal[0], g_PwmExpetVal[0], g_PwmIncreVal[0]);
#endif

    if(ROC_SERVO_SPEED_DIV_STP <= RefreshTimes)
    {
        RefreshTimes = 0;

        RocServoPwmRecod();
        RocRobotRemoteControl();
        RocServoPwmIncreCalculate();
    }

    RocServoPwmOut();
}

/*********************************************************************************
 *  Description:
 *              Set the speed of servo running
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
void RocServoSpeedSet(uint16_t ServoRunTimeMs)
{
    ROC_RESULT Ret = RET_OK;

    htim6.Init.Period = ServoRunTimeMs * 10 / ROC_SERVO_SPEED_DIV_STP;
    TIM_Base_SetConfig(htim6.Instance, &htim6.Init);

    if(RET_OK != Ret)
    {
        ROC_LOGE("Servo speed setting is in error!");
    }
}

/*********************************************************************************
 *  Description:
 *              Start the servo timer
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
ROC_RESULT RocServoTimerStart(void)
{
    ROC_RESULT Ret = RET_OK;

    if(HAL_OK != HAL_TIM_Base_Start_IT(&htim6))
    {
        Ret = RET_ERROR;

        Error_Handler();
    }

    return Ret;
}

/*********************************************************************************
 *  Description:
 *              Stop the servo timer
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
ROC_RESULT RocServoTimerStop(void)
{
    ROC_RESULT Ret = RET_OK;

    if(HAL_OK != HAL_TIM_Base_Stop_IT(&htim6))
    {
        Ret = RET_ERROR;

        Error_Handler();
    }

    return Ret;
}

/*********************************************************************************
 *  Description:
 *              Update the servo input PWM data
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
ROC_RESULT RocServoInputUpdate(uint16_t *pServoInputVal)
{
    uint8_t     i = 0;
    ROC_RESULT  Ret = RET_OK;

    for(i = 0; i < ROC_SERVO_MAX_SUPPORT_NUM; i++)
    {
        g_PwmExpetVal[i] = pServoInputVal[i];
    }

    return Ret;
}

/*********************************************************************************
 *  Description:
 *              Servo driver init
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              The servo driver init status
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
**********************************************************************************/
ROC_RESULT RocServoInit(void)
{
    ROC_RESULT Ret = RET_OK;

    Ret = RocServoTimerStop();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Servo timer init is in error!");
    }

    if(RET_OK != Ret)
    {
        ROC_LOGE("Servo module init is in error!");
    }
    else
    {
        ROC_LOGI("Servo module init is in success.");
    }

    return Ret;
}


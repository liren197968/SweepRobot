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


int16_t             g_PwmExpetVal[ROC_SERVO_MAX_SUPPORT_NUM] = {0};

static int16_t      g_PwmIncreVal[ROC_SERVO_MAX_SUPPORT_NUM] = {0};
static int16_t      g_PwmPreseVal[ROC_SERVO_MAX_SUPPORT_NUM] = {0};
static int16_t      g_PwmLastdVal[ROC_SERVO_MAX_SUPPORT_NUM] = {0};

static ROC_RESULT   g_ServoTurnIsFinshed = ROC_FALSE;


/*********************************************************************************
 *  Description:
 *              Init the servo PWM data
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
static void RocServoPwmDatInit(int16_t *pServoInputVal)
{
    uint8_t i = 0;

    for(i = 0; i < ROC_SERVO_MAX_SUPPORT_NUM; i++)
    {
        g_PwmExpetVal[i] = pServoInputVal[i];
        g_PwmPreseVal[i] = pServoInputVal[i];
        g_PwmLastdVal[i] = pServoInputVal[i];
    }
}

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
            ROC_LOGE("Servo(%d) input PWM value(%d) is less than ROC_SERVO_MIN_PWM_VAL! Be careful! \r\n", i, g_PwmExpetVal[i]);

            g_PwmExpetVal[i] = ROC_SERVO_MIN_PWM_VAL;
        }
        else if(g_PwmExpetVal[i] > ROC_SERVO_MAX_PWM_VAL)
        {
            ROC_LOGE("Servo(%d) input PWM value(%d) is more than ROC_SERVO_MAX_PWM_VAL! Be careful! \r\n", i, g_PwmExpetVal[i]);

            g_PwmExpetVal[i] = ROC_SERVO_MAX_PWM_VAL;
        }

        g_PwmIncreVal[i] = (g_PwmExpetVal[i] - g_PwmLastdVal[i]) / ROC_SERVO_SPEED_DIV_STP;
    }
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
static void RocServoInputUpdate(int16_t *pServoInputVal)
{
    uint8_t     i = 0;

    for(i = 0; i < ROC_SERVO_MAX_SUPPORT_NUM; i++)
    {
        g_PwmExpetVal[i] = pServoInputVal[i];
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
static void RocServoPwmOutput(void)
{
    HAL_StatusTypeDef   WriteStatus = HAL_OK;

    WriteStatus = RocPca9685OutPwmAll(PWM_ADDRESS_L, (uint16_t *)g_PwmPreseVal, ROC_PCA9685_CHANNEL_MAX_NUM);
    if(HAL_OK != WriteStatus)
    {
        ROC_LOGE("Servo PWM out is in error, and servo will stop running!");
        while(1);
    }

    WriteStatus = RocPca9685OutPwmAll(PWM_ADDRESS_H, (uint16_t *)g_PwmPreseVal + ROC_PCA9685_CHANNEL_MAX_NUM, ROC_SERVO_MAX_SUPPORT_NUM - ROC_PCA9685_CHANNEL_MAX_NUM);
    if(HAL_OK != WriteStatus)
    {
        ROC_LOGE("Servo PWM out is in error, and servo will stop running!");
        while(1);
    }
}

/*********************************************************************************
 *  Description:
 *              Check servos turns is finshed
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              The running state
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
**********************************************************************************/
ROC_RESULT RocServoTurnIsFinshed(void)
{
    return g_ServoTurnIsFinshed;
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
void RocServoControl(int16_t *pServoInputVal)
{
    static uint8_t  RefreshTimes = 0U;

    RefreshTimes++;     /* record the times of the data update of servo */

    RocServoPwmUpdate(RefreshTimes);

#ifdef ROC_ROBOT_SERVO_DEBUG
    ROC_LOGI("RefreshTimes is %d, g_PwmPreseVal is %d, g_PwmExpetVal is %d, g_PwmIncreVal is %d \r\n",
                            RefreshTimes, g_PwmPreseVal[0], g_PwmExpetVal[0], g_PwmIncreVal[0]);
#endif

    if(ROC_SERVO_SPEED_DIV_STP <= RefreshTimes)
    {
        RefreshTimes = 0;

        RocServoPwmRecod();

        RocServoInputUpdate(pServoInputVal);

        RocServoPwmIncreCalculate();
    }

    if((ROC_SERVO_SPEED_DIV_STP - 1) == RefreshTimes)
    {
        g_ServoTurnIsFinshed = ROC_TRUE;
    }
    else
    {
        g_ServoTurnIsFinshed = ROC_FALSE;
    }

    RocServoPwmOutput();
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

    Ret = RocServoTimerStop();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Servo stop is in error!");

        while(1);
    }

    htim6.Init.Period = (uint32_t)((ServoRunTimeMs + 0) * 2 / ROC_SERVO_SPEED_DIV_STP);    /* 32 is the timer error */

    htim6.Instance->CNT = 0;
    htim6.Instance->ARR = htim6.Init.Period;

    htim6.Instance->CR1 &= ~TIM_CR1_ARPE;

    Ret = RocServoTimerStart();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Servo start is in error!");

        while(1);
    }

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
ROC_RESULT RocServoInit(int16_t *pServoInputVal)
{
    ROC_RESULT Ret = RET_OK;

    Ret = RocServoTimerStop();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Servo timer init is in error!");
    }

    RocServoPwmDatInit(pServoInputVal);

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


/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/01/18      1.0
********************************************************************************/
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "tim.h"

#include "RocLog.h"
#include "RocBeeper.h"


static ROC_BEEPER_CTRL_s g_BeeperCtrl = {ROC_NONE};
/*********************************************************************************
 *  Description:
 *              Turn beeper on
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.01.06)
**********************************************************************************/
static void RocBeeperOn(void)
{
    HAL_GPIO_WritePin(ROC_BEEPER_GPIO_PORT, ROC_BEEPER_CTRL_PIN, GPIO_PIN_SET);
}

/*********************************************************************************
 *  Description:
 *              Turn beeper off
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.01.06)
**********************************************************************************/
static void RocBeeperOff(void)
{
    HAL_GPIO_WritePin(ROC_BEEPER_GPIO_PORT, ROC_BEEPER_CTRL_PIN, GPIO_PIN_RESET);
}

/*********************************************************************************
 *  Description:
 *              Toggle beeper
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.30)
**********************************************************************************/
static void RocBeeperToggle(void)
{
    HAL_GPIO_TogglePin(ROC_BEEPER_GPIO_PORT, ROC_BEEPER_CTRL_PIN);
}

/*********************************************************************************
 *  Description:
 *              Start the beeper timer
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.29)
**********************************************************************************/
ROC_RESULT RocBeeperTimerStart(void)
{
    ROC_RESULT Ret = RET_OK;

    if(HAL_OK != HAL_TIM_Base_Start_IT(&htim2))
    {
        Ret = RET_ERROR;

        Error_Handler();
    }

    return Ret;
}

/*********************************************************************************
 *  Description:
 *              Stop the beeper timer
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.29)
**********************************************************************************/
ROC_RESULT RocBeeperTimerStop(void)
{
    ROC_RESULT Ret = RET_OK;

    if(HAL_OK != HAL_TIM_Base_Stop_IT(&htim2))
    {
        Ret = RET_ERROR;

        Error_Handler();
    }

    return Ret;
}

/*********************************************************************************
 *  Description:
 *              Set the interrupt period of the beeper timer
 *
 *  Parameter:
 *              Period: timer overflow period
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.29)
**********************************************************************************/
static void RocBeeperTimerPeriodSet(uint16_t Period)
{
    ROC_RESULT Ret = RET_OK;

    Ret = RocBeeperTimerStop();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Beeper timer stop is in error!");
    }

    htim2.Init.Period = Period;

    htim2.Instance->CNT = 0;
    htim2.Instance->ARR = htim2.Init.Period;
    htim2.Instance->CR1 &= ~TIM_CR1_ARPE;

    Ret = RocBeeperTimerStart();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Beeper timer start is in error!");

        while(1);
    }

    if(RET_OK != Ret)
    {
        ROC_LOGE("Beeper timer setting is in error!");
    }
}

/*********************************************************************************
 *  Description:
 *              Blink beeper for serval times
 *
 *  Parameter:
 *              BlinkTimes: beeper on and off times
 *              PeriodTime: the time of beeper on for every blink(ms)
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.01.06)
**********************************************************************************/
void RocBeeperBlink(uint16_t BlinkTimes, uint16_t PeriodTime)
{
    while(0 != g_BeeperCtrl.RunTimes);  /* Wait for the last beeper action finshed */

    if(ROC_BEEPER_BLINK_FOREVER == BlinkTimes)
    {
        g_BeeperCtrl.RunTimes = ROC_BEEPER_BLINK_FOREVER;
    }
    else
    {
        g_BeeperCtrl.RunTimes = BlinkTimes * 2 - 1;
    }

    RocBeeperTimerPeriodSet(PeriodTime * ROC_BEEPER_ONE_SECOND_TICKS);

    RocBeeperOn();
}

/*********************************************************************************
 *  Description:
 *              Control beeper on and off with timer interrupt
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.29)
**********************************************************************************/
void RocBeeperTaskBackground(void)
{
    if(0 < g_BeeperCtrl.RunTimes)
    {
        g_BeeperCtrl.RunTimes--;

        RocBeeperToggle();
    }
    else
    {
        RocBeeperOff();
    }
}
/*********************************************************************************
 *  Description:
 *              Beeper driver init
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.01.06)
**********************************************************************************/
ROC_RESULT RocBeeperInit(void)
{
    ROC_RESULT Ret = RET_OK;

    RocBeeperOff();
    RocBeeperTimerStart();

    if(RET_OK != Ret)
    {
        ROC_LOGE("Beeper module init is in error!");
    }
    else
    {
        ROC_LOGI("Beeper module init is in success");
    }

    return Ret;
}


/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/01/18      1.0
********************************************************************************/
#include "stm32f4xx_hal.h"
#include "gpio.h"

#include "RocLog.h"
#include "RocBeeper.h"

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
 *              Blink beeper for serval times
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
void RocBeeperBlink(uint16_t Times, uint16_t Time)
{
    uint16_t i = 0;
    uint16_t LoopTime = Time / (Times * 2);

    for(i = 0; i < Times; i++)
    {
        RocBeeperOn();
        HAL_Delay(LoopTime);

        RocBeeperOff();
        HAL_Delay(LoopTime);
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

    RocBeeperBlink(2, 300);

    if(RET_OK != Ret)
    {
        ROC_LOGE("Beeper module init is in error!");
    }
    else
    {
        ROC_LOGI("Beeper module init is in success.");
    }

    return Ret;
}


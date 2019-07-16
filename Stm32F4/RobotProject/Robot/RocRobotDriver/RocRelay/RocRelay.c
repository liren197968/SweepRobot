/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/06/17      1.0
********************************************************************************/
#include "stm32f4xx_hal.h"
#include "gpio.h"

#include "RocLog.h"
#include "RocRelay.h"


/*********************************************************************************
 *  Description:
 *              Turn relay on
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.06.17)
**********************************************************************************/
void RocRelayTurnOn(void)
{
    HAL_GPIO_WritePin(ROC_ROBOT_RELAY_PORT, ROC_ROBOT_RELAY_PIN, GPIO_PIN_RESET);
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
 *              ROC LiRen(2019.06.17)
**********************************************************************************/
void RocRelayTurnOff(void)
{
    HAL_GPIO_WritePin(ROC_ROBOT_RELAY_PORT, ROC_ROBOT_RELAY_PIN, GPIO_PIN_SET);
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
 *              ROC LiRen(2019.06.17)
**********************************************************************************/
void RocRelayToggle(void)
{
    HAL_GPIO_TogglePin(ROC_ROBOT_RELAY_PORT, ROC_ROBOT_RELAY_PIN);
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
 *              ROC LiRen(2019.06.17)
**********************************************************************************/
ROC_RESULT RocRelayInit(void)
{
    ROC_RESULT Ret = RET_OK;

    RocRelayTurnOff();

    if(RET_OK != Ret)
    {
        ROC_LOGE("Relay module init is in error!");
    }
    else
    {
        ROC_LOGI("Relay module init is in success");
    }

    return Ret;
}


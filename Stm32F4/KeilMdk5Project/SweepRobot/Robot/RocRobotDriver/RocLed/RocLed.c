#include "gpio.h"

#include "RocLog.h"
#include "RocLed.h"


/*********************************************************************************
 *  Description:
 *              Turn on LED
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
void RocLedTurnOn(void)
{
    HAL_GPIO_WritePin(ROC_DEBUG_LED_PORT, ROC_DEBUG_LED_PIN, GPIO_PIN_RESET);
}

/*********************************************************************************
 *  Description:
 *              Turn off LED
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
void RocLedTurnOff(void)
{
    HAL_GPIO_WritePin(ROC_DEBUG_LED_PORT, ROC_DEBUG_LED_PIN, GPIO_PIN_SET);
}

/*********************************************************************************
 *  Description:
 *              Toggle LED
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
void RocLedToggle(void)
{
    HAL_GPIO_TogglePin(ROC_DEBUG_LED_PORT, ROC_DEBUG_LED_PIN);
}

/*********************************************************************************
 *  Description:
 *              LED init
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
ROC_RESULT RocLedInit(void)
{
    ROC_RESULT Ret = RET_OK;

    RocLedTurnOff();

    ROC_LOGI("Robot led module hardware init success.");

    return Ret;
}


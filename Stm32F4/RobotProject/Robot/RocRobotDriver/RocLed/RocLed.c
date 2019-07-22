/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2018/12/16      1.0
********************************************************************************/
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
void RocLedTurnOn(ROC_LED_TYPE_e Led)
{
    switch(Led)
    {
        case ROC_LED_DEBUG:
        {
            HAL_GPIO_WritePin(ROC_LED_DEBUG_PORT, ROC_LED_DEBUG_PIN, GPIO_PIN_RESET);
            break;
        }

        case ROC_LED_1:
        {
            HAL_GPIO_WritePin(ROC_LED_1_PORT, ROC_LED_1_PIN, GPIO_PIN_RESET);
            break;
        }

        case ROC_LED_2:
        {
            HAL_GPIO_WritePin(ROC_LED_2_PORT, ROC_LED_2_PIN, GPIO_PIN_RESET);
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
void RocLedTurnOff(ROC_LED_TYPE_e Led)
{
    switch(Led)
    {
        case ROC_LED_DEBUG:
        {
            HAL_GPIO_WritePin(ROC_LED_DEBUG_PORT, ROC_LED_DEBUG_PIN, GPIO_PIN_SET);
            break;
        }

        case ROC_LED_1:
        {
            HAL_GPIO_WritePin(ROC_LED_1_PORT, ROC_LED_1_PIN, GPIO_PIN_SET);
            break;
        }

        case ROC_LED_2:
        {
            HAL_GPIO_WritePin(ROC_LED_2_PORT, ROC_LED_2_PIN, GPIO_PIN_SET);
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
 *              Toggle LED
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
void RocLedToggle(ROC_LED_TYPE_e Led)
{
    switch(Led)
    {
        case ROC_LED_DEBUG:
        {
            HAL_GPIO_TogglePin(ROC_LED_DEBUG_PORT, ROC_LED_DEBUG_PIN);
            break;
        }

        case ROC_LED_1:
        {
            HAL_GPIO_TogglePin(ROC_LED_1_PORT, ROC_LED_1_PIN);
            break;
        }

        case ROC_LED_2:
        {
            HAL_GPIO_TogglePin(ROC_LED_2_PORT, ROC_LED_2_PIN);
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

    ROC_LOGI("Robot led module hardware init success.");

    return Ret;
}


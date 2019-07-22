/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/07/14      1.0
********************************************************************************/

#include "gpio.h"
#include "tim.h"

#include "RocLog.h"
#include "RocKey.h"


static uint32_t g_KeyMask = 0x00;
static uint16_t g_KeyCoder[ROC_KEY_NUM] = {0};

/*********************************************************************************
 *  Description:
 *              Scan the key GPIO state
 *
 *  Parameter:
 *              None
 *
 *  Return
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.07.14)
**********************************************************************************/
void RocKeyTaskBackground(void)
{
    ROC_KEY_TYPE_e KeyNum;

    for(KeyNum = ROC_KEY_1; KeyNum < ROC_KEY_NUM; KeyNum++)
    {
        g_KeyCoder[KeyNum] <<= 1;
    }

    g_KeyCoder[ROC_KEY_1]  |= HAL_GPIO_ReadPin(ROC_KEY_1_PORT, ROC_KEY_1_PIN);
    g_KeyCoder[ROC_KEY_2]  |= HAL_GPIO_ReadPin(ROC_KEY_2_PORT, ROC_KEY_2_PIN);
    g_KeyCoder[ROC_KEY_3]  |= HAL_GPIO_ReadPin(ROC_KEY_3_PORT, ROC_KEY_3_PIN);
    g_KeyCoder[ROC_KEY_4]  |= HAL_GPIO_ReadPin(ROC_KEY_4_PORT, ROC_KEY_4_PIN);
    g_KeyCoder[ROC_KEY_5]  |= HAL_GPIO_ReadPin(ROC_KEY_5_PORT, ROC_KEY_5_PIN);
    g_KeyCoder[ROC_KEY_6]  |= HAL_GPIO_ReadPin(ROC_KEY_6_PORT, ROC_KEY_6_PIN);
    g_KeyCoder[ROC_KEY_7]  |= HAL_GPIO_ReadPin(ROC_KEY_7_PORT, ROC_KEY_7_PIN);
    g_KeyCoder[ROC_KEY_8]  |= HAL_GPIO_ReadPin(ROC_KEY_8_PORT, ROC_KEY_8_PIN);
    g_KeyCoder[ROC_KEY_9]  |= HAL_GPIO_ReadPin(ROC_KEY_9_PORT, ROC_KEY_9_PIN);
    g_KeyCoder[ROC_KEY_10] |= HAL_GPIO_ReadPin(ROC_KEY_10_PORT, ROC_KEY_10_PIN);
    g_KeyCoder[ROC_KEY_11] |= HAL_GPIO_ReadPin(ROC_KEY_11_PORT, ROC_KEY_11_PIN);
    g_KeyCoder[ROC_KEY_12] |= HAL_GPIO_ReadPin(ROC_KEY_12_PORT, ROC_KEY_12_PIN);
    g_KeyCoder[ROC_KEY_13] |= HAL_GPIO_ReadPin(ROC_KEY_13_PORT, ROC_KEY_13_PIN);
    g_KeyCoder[ROC_KEY_14] |= HAL_GPIO_ReadPin(ROC_KEY_14_PORT, ROC_KEY_14_PIN);
    g_KeyCoder[ROC_KEY_15] |= HAL_GPIO_ReadPin(ROC_KEY_15_PORT, ROC_KEY_15_PIN);
    g_KeyCoder[ROC_KEY_16] |= HAL_GPIO_ReadPin(ROC_KEY_LT_PORT, ROC_KEY_LT_PIN);
    g_KeyCoder[ROC_KEY_17] |= HAL_GPIO_ReadPin(ROC_KEY_RT_PORT, ROC_KEY_RT_PIN);

    for(KeyNum = ROC_KEY_1; KeyNum < ROC_KEY_NUM; KeyNum++)
    {
        if((g_KeyCoder[KeyNum] & ROC_KEY_DURATION_TIME_MASK) == 0x00)
        {
            //g_KeyMask |= (1U << KeyNum);
            g_KeyMask = KeyNum;     /*TODO, support multiple key */
        }
    }
}

/*********************************************************************************
 *  Description:
 *              Get the press key number
 *
 *  Parameter:
 *              None
 *
 *  Return
 *              The number of pressed key
 *
 *  Author:
 *              ROC LiRen(2019.07.14)
**********************************************************************************/
uint32_t RocPressKeyNumGet(void)
{
    ROC_LOGN("Key: %x", g_KeyMask);
    return g_KeyMask;
}

/*********************************************************************************
 *  Description:
 *              Clear the press key event
 *
 *  Parameter:
 *              KeyNum: The number of pressed key
 *
 *  Return
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.07.14)
**********************************************************************************/
void RocKeyEventClear(ROC_KEY_TYPE_e KeyNum)
{
    //g_KeyMask &= ~(1U << KeyNum);
    g_KeyMask = ROC_KEY_0_MASK;
}

/*********************************************************************************
 *  Description:
 *              Key module init
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
ROC_RESULT RocKeyInit(void)
{
    uint8_t i = 0;
    ROC_RESULT Ret = RET_OK;

    for(i = 0; i < ROC_KEY_NUM; i++)
    {
        g_KeyCoder[i] = ROC_KEY_DURATION_TIME_MASK;
    }

    if(HAL_OK != HAL_TIM_Base_Start_IT(&htim6))
    {
        Error_Handler();
    }

    if(RET_OK != Ret)
    {
        ROC_LOGE("Key module init is in error!");
    }
    else
    {
        ROC_LOGI("Key module init is in success");
    }

    return Ret;
}



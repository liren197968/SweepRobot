/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/07/14      1.0
********************************************************************************/

#include <string.h>

#include "tim.h"
#include "gpio.h"
#include "usart.h"

#include "RocLog.h"
#include "RocLed.h"
#include "RocKey.h"
#include "RocOled.h"
#include "RocBeeper.h"
#include "RocBattery.h"
#include "RocRemoteControl.h"
#include "RocJoystick.h"


static uint8_t g_BatTimeIsReady = ROC_NONE;

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    ROC_LOGW("%d pin interrupt triggers", GPIO_Pin);
}

/*********************************************************************************
 *  Description:
 *              Robot init success beeper aciton
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.05.11)
**********************************************************************************/
static void RocJoystickInitEndAction(void)
{
    RocBeeperBlink(4, 100);
}

/*********************************************************************************
 *  Description:
 *              Robot battery need charge beeper aciton
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.05.11)
**********************************************************************************/
static void RocBatteryChargeBeeperAction(void)
{
    RocBeeperBlink(ROC_BEEPER_BLINK_FOREVER, 1000);
}

/*********************************************************************************
 *  Description:
 *              Joystick battery check task entry
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.07.14)
**********************************************************************************/
static void RocBatteryCheckTaskEntry(void)
{
    static uint8_t Count = 0;
    if(ROC_TRUE == g_BatTimeIsReady)
    {
        g_BatTimeIsReady = ROC_FALSE;

        RocBatteryVoltageAdcSample();

        Count++;

        if(10 == Count)
        {
            Count = 0;
            //RocAdcVoltageGet();
            RocPressKeyNumGet();
        }
    }

#if 0
    if(ROC_ROBOT_BATTERY_LIMITED_VOLTATE > RocBatteryVoltageGet())
    {
        ROC_LOGN("Battery is in low electricity! Charge it!");

        RocBatteryChargeBeeperAction();
    }
#endif
}

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
    if(TIM2 == htim->Instance)
    {
        RocBeeperTaskBackground();
    }
    else if(TIM6 == htim->Instance)
    {
        RocKeyTaskBackground();
    }
    else if(TIM7 == htim->Instance)
    {
        static uint8_t TimeTick = 0;

        TimeTick++;

        g_BatTimeIsReady = ROC_TRUE;
    }
}

/*********************************************************************************
 *  Description:
 *              Robot joystick control init
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.07.14)
**********************************************************************************/
void RocJoystickInit(void)
{
    ROC_RESULT Ret = RET_OK;

    ROC_LOGW("############# Robot hardware version is V1! #############");

//    Ret = RocLedInit();
//    if(RET_OK != Ret)
//    {
//        ROC_LOGE("Robot hardware is in error, the system will not run!");
//
//        while(1);
//    }

    Ret = RocKeyInit();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Robot hardware is in error, the system will not run!");

        while(1);
    }

    Ret = RocOledInit();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Robot hardware is in error, the system will not run!");

        while(1);
    }

    Ret = RocBatteryInit();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Robot hardware is in error, the system will not run!");

        while(1);
    }

//    Ret = RocBeeperInit();
//    if(RET_OK != Ret)
//    {
//        ROC_LOGE("Robot hardware is in error, the system will not run!");
//    
//        while(1);
//    }

//    Ret = RocRemoteControlInit();
//    if(RET_OK != Ret)
//    {
//        ROC_LOGE("Robot hardware is in error, the system will not run!");
//
//        while(1);
//    }

//    RocJoystickInitEndAction();

    ROC_LOGI("Robot hardware init is in success, and the system start running.");

}
/*********************************************************************************
 *  Description:
 *              Robot joystick control while main
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.07.14)
**********************************************************************************/
void RocJoystickMain(void)
{
    RocBatteryCheckTaskEntry();
}


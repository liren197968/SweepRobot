/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/01/16      1.0
********************************************************************************/
#include "adc.h"
#include "tim.h"

#include "RocLog.h"
#include "RocBattery.h"


static uint16_t g_AdcConvertedValue = 0;
static float g_BatteryVoltageValue = ROC_ROBOT_BATTERY_LIMITED_VOLTATE + 0.02F;

/*********************************************************************************
 *  Description:
 *              Start battery ADC DMA convert
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.01.16)
**********************************************************************************/
void RocBatteryVoltageConvertStart(void)
{
    if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&g_AdcConvertedValue, 1) != HAL_OK)
    {
        Error_Handler();
    }

    if(HAL_OK != HAL_TIM_Base_Start_IT(&htim7))
    {
        Error_Handler();
    }
}

/*********************************************************************************
 *  Description:
 *              Sample the battery voltage ADC value
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.01.16)
**********************************************************************************/
void RocBatteryVoltageAdcSample(void)
{
    uint8_t             i;
    uint16_t            ConvertedValueSumData = 0;
    static uint8_t      AdcSampleIndex = 0;
    static uint8_t      AdcSampleTimes = 0;
    static uint16_t     SampleData[ROC_ADC_VALUE_FILTER_TIMES];

    AdcSampleTimes++;

    if(AdcSampleTimes == ROC_ADC_VALUE_SAMPLE_CYCLE)
    {
        AdcSampleTimes = 0;

        SampleData[AdcSampleIndex] = g_AdcConvertedValue;

        AdcSampleIndex++;
    }

    if(AdcSampleIndex == ROC_ADC_VALUE_FILTER_TIMES)
    {
        AdcSampleIndex = 0;

        ConvertedValueSumData = 0;

        for(i = 0; i < ROC_ADC_VALUE_FILTER_TIMES; i++)
        {
            ConvertedValueSumData = ConvertedValueSumData + SampleData[i];
        }

        g_AdcConvertedValue = ConvertedValueSumData / ROC_ADC_VALUE_FILTER_TIMES;

        g_BatteryVoltageValue = ROC_ADC_VOLTAGE_DIVIDE_FACTOR * g_AdcConvertedValue * ROC_ADC_CONVERTED_TO_VOLTAGE + ROC_ADC_VOLTAGE_DROP_ERROR;
    }
    else
    {
        g_BatteryVoltageValue = g_BatteryVoltageValue;
    }
}

/*********************************************************************************
 *  Description:
 *              Get the battery voltage
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.01.16)
**********************************************************************************/
float RocBatteryVoltageGet(void)
{
    return g_BatteryVoltageValue;
}

/*********************************************************************************
 *  Description:
 *              ADC complete converting callback function
 *
 *  Parameter:
 *              None
 *
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.20)
**********************************************************************************/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
    if(ADC1 == AdcHandle->Instance)
    {
        ROC_LOGW("ADC convert success(%d).", g_AdcConvertedValue);
    }
}

/*********************************************************************************
 *  Description:
 *              Battery module init
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              The init status
 *
 *  Author:
 *              ROC LiRen(2019.01.16)
**********************************************************************************/
ROC_RESULT RocBatteryInit(void)
{
    ROC_RESULT Ret = RET_OK;

    RocBatteryVoltageConvertStart();

    if(RET_OK != Ret)
    {
        ROC_LOGE("Battery module init is in error(%d)!", Ret);
    }
    else
    {
        ROC_LOGI("Battery module init is in success.");
    }

    return Ret;
}


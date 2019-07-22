/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/01/16      1.0
********************************************************************************/
#ifndef _ROC_BATTERY_H
#define _ROC_BATTERY_H


#define ROC_ADC_REFERENCE_VOLTAGE               3.3F
#define ROC_ADC_CHANNEL_RESOLUTION              12
#define ROC_ADC_CONVERTED_TO_VOLTAGE            (ROC_ADC_REFERENCE_VOLTAGE / (1 << 12))

#define ROC_ADC_CONVERTED_CHANNEL_NUM           5

#define ROC_ADC_VALUE_SAMPLE_CYCLE              5
#define ROC_ADC_VALUE_FILTER_TIMES              20

#define ROC_ADC_VOLTAGE_DIVIDE_FACTOR           4
#define ROC_ADC_VOLTAGE_DROP_ERROR              0.53F
#define ROC_ROBOT_BATTERY_LIMITED_VOLTATE       7.4F


void RocBatteryVoltageAdcSample(void);
float RocBatteryVoltageGet(void);
ROC_RESULT RocBatteryInit(void);
void RocJoystickAdcGet(uint16_t *JoystickAdc);



#endif

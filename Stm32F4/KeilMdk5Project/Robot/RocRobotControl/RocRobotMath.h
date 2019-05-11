/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/04/13      1.0
********************************************************************************/
#ifndef __ROC_ROBOT_MATH_H
#define __ROC_ROBOT_MATH_H


#include <math.h>

#include "stm32f4xx_hal.h"
#include "arm_math.h"


#if (__FPU_USED == 1U)
#define Sin(x)              arm_sin_f32(x)
#define Cos(x)              arm_cos_f32(x)
#define ASin(x)             asin(x)
#define ACos(x)             acos(x)
#define ATan(x)             atan(x)
#define ATan2(x, y)         atan2(x, y)
#define Pow(x, y)           pow(x, y)
#define Sqrt(in, out)       arm_sqrt_f32(in, out)
#else
#define Sin(x)              sin(x)
#define Cos(x)              cos(x)
#define ASin(x)             asin(x)
#define ACos(x)             acos(x)
#define ATan(x)             atan(x)
#define ATan2(x, y)         atan2(x, y)
#define Pow(x, y)           pow(x, y)
#define Sqrt(x)             sqrt(x)
#endif


#endif

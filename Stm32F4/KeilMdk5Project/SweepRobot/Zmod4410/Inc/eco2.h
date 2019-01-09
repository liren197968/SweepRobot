/*******************************************************************************
 * Copyright (c) 2018 Integrated Device Technology, Inc.
 * All Rights Reserved.
 *
 * This code is proprietary to IDT, and is license pursuant to the terms and
 * conditions that may be accessed at:
 * https://www.idt.com/document/msc/idt-software-license-terms-gas-sensor-software
 *
 ******************************************************************************/

/**
 * @file    eco2.h
 * @date    2017-11-08
 * @author  Franziska Naepelt
 * @version 1.1.4 - https://semver.org/
 * @brief   This file contains the data structure definition and
 *          the function definition for the eCO2 algorithm.
 * @details The library contains an algorithm to calculate an estimated CO2
 *          value in ppm from a given TVOC (Total Volatile Organic Compound)
 *          value in mg/m^3. Refer to the Application Note *ZMOD4410 Estimating
 *          Carbon Dioxide* for further information.
 */


#ifndef ECO2_H_
#define ECO2_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdint.h>

// parameters for eCO2 calculations

#define FILTER_TAU              (20.0)    /**< filter coefficient */
#define CNT_PREV_SAMPLES_TVOC   (4)       /**< jump over first x samples */

/**
 * @brief Parameters to control the eCO2 algorithm.
 */
typedef struct {
    uint16_t    min_co2;          /**< lowest possible CO2 value in ppm */
    uint16_t    max_co2;          /**< highest possible CO2 value in ppm */
    float       tvoc_to_eco2;     /**< how much ppm CO2 are generated per mg/m^3 TVOC */
    float       hot_wine_rate;    /**< rate for fast positive changes */
    float       open_window_rate; /**< rate for fast negative changes */
}eco2_params;

/**
 * @brief   calculates estimated CO2 value from TVOC
 * @param   [in] conc_tvoc TVOC concentration
 * @param   [in] sample_rate sample rate in seconds
 * @param   [in] params algorithm control parameters
 * @return  eCO2 value
 */
float calc_eco2(float conc_tvoc, float sample_rate, eco2_params* params);

#ifdef __cplusplus
}
#endif

#endif /* ECO2_H_ */

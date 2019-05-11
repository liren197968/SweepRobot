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
 * @file    tvoc.h
 * @date    2017-11-07
 * @author  Franziska Naepelt
 * @version 1.0.11 - https://semver.org/
 * @brief   This file contains the data structure definition and the TVOC
 *          algorithm function definition.
 * @details The library contains an algorithm to calculate the TVOC concentration
 *          (Total Volatile Organic Compound) from a given MOx and clean dry air
 *          resistance value.
 */

#ifndef TVOC_H_
#define TVOC_H_

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Parameters to control the TVOC algorithm.
 */
typedef struct {
    float A;        /**< intercept parameter for TVOC algorithm */
    float alpha;    /**< slope parameter for TVOC algorithm */
}tvoc_params;

/**
 * @brief   calculates TVOC from r_mox and r_cda
 * @param   [in] r_mox MOx resistance
 * @param   [in] r_cda clean dry air resistance
 * @param   [in] params TVOC algorithm parameters
 * @return  tvoc concentration in mg/m^3
 */
float calc_tvoc(float r_mox, float r_cda, tvoc_params* params);

#ifdef __cplusplus
}
#endif

#endif /* TVOC_H_ */

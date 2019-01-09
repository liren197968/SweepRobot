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
 * @file    iaq.h
 * @date    2018-03-23
 * @author  Franziska Naepelt
 * @version 1.0.5 - https://semver.org/
 * @brief   This file contains the IAQ algorithm function definition.
 * @details The library contains an algorithm that calculates the Indoor Air
 *          Quality based on the scale introduced from the German Federal
 *          Environment Agency, which is divided into five levels from "1" for
 *          very good to "5" for bad indoor air quality. For further details see
 *          *Umweltbundesamt, Beurteilung von Innenraumluftkontaminationen
 *          mittels Referenz- und Richtwerten*, (Bundesgesundheitsblatt -
 *          Gesundheitsforschung - Gesundheitsschutz, 2007).
 */

#ifndef IAQ_H_
#define IAQ_H_

#include <math.h>
#include <stdint.h>

#include "tvoc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief IAQ ratings (German Federal Environmental Agency)
 */
enum {
    VERY_GOOD = 1,
    GOOD = 2,
    MEDIUM = 3,
    POOR = 4,
    BAD = 5
};

/**
 * @brief   calculates IAQ from r_mox, r_cda and settings
 * @param   [in] r_mox MOx resistance
 * @param   [in] r_cda clean dry air resistance
 * @param   [in] params TVOC algorithm parameters
 * @return  iaq rating
 */
uint8_t calc_iaq(float r_mox, float r_cda, tvoc_params* params);

#ifdef __cplusplus
}
#endif

#endif /* IAQ_H_ */

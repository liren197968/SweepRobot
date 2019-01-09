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
 * @file    r_cda.h
 * @brief   This file contains the clean dry air resistance tracker function
 *          definition.
 * @date    2017-08-04
 * @author  Franziska Naepelt
 * @version 1.0.7 - https://semver.org/
 * @details The library contains an algorithm to calculate and track the clean
 *          dry air resistance from a given MOx resistance value. The r_cda is
 *          the MOx resistance in ohms at clean dry air, and it is used as the
 *          reference value for further gas measurements.
 */

#ifndef __R_CDA__
#define __R_CDA__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   tracks r_cda
 * @param   [in] r_mox current MOx resistance
 * @return  new r_cda
 */
float r_cda_tracker(float r_mox);

#ifdef __cplusplus
}
#endif

#endif // __R_CDA__

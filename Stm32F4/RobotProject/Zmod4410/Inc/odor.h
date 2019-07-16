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
 * @file    odor.h
 * @date    2018-09-10
 * @author  Franziska Naepelt
 * @version 1.0.0 - https://semver.org/
 * @brief   This file contains the data structure definition and the odor
 *          algorithm function definition.
 * @details This file contains the function definitions for the odor algorithm.
 */

#ifndef ODOR_H_
#define ODOR_H_

#include <math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Parameters to control the odor algorithm.
 */
typedef struct {
    float alpha;            /**< slope parameter for odor algorithm */
    uint32_t stop_delay;    /**< control signal follow-up time */
    float threshold;        /**< threshold to switch, i.e. 1.3 - corresponds to 30 % rise in concentration */
    uint32_t tau;           /**< time constant for averaging */
    uint32_t stabilization_samples; /**< ignore number of samples for sensor stabilization */
}odor_params;

/**
 * @brief Control signal states.
 */
typedef enum {
    OFF = 0,
    ON = 1,
} control_signal_state_t;


/**
 * @brief   calculates TVOC from r_mox and r_cda
 * @param   [in] r_mox MOx resistance
 * @param   [in] params odor algorithm parameters
 * @return  control signal input
 * @retval  0 below threshold
 * @retval  1 above threshold
 */
control_signal_state_t calc_odor(float r_mox, odor_params* params);

#ifdef __cplusplus
}
#endif

#endif /* ODOR_H_ */

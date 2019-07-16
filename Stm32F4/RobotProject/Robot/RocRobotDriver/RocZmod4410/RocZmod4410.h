#ifndef __ROC_ZMOD4410_H
#define __ROC_ZMOD4410_H

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
 * @file    RocZmod4410.h
 * @brief   This is an example for the ZMOD4410 gas sensor module.
 * @version 1.0.2
 * @date    2017-05-15
 * @author  Franziska Naepelt <franziska.naepelt@idt.com>
 */

#include <stdio.h>

/* Algorithm library header files, download the target specific library
 * from the IDT webpage. */
#include "eco2.h"
#include "iaq.h"
#include "odor.h"
#include "tvoc.h"
#include "r_cda.h"


/* files to control the sensor */
#include "zmod44xx.h"

// start sequencer defines

#define FIRST_SEQ_STEP      0
#define LAST_SEQ_STEP       1

#define STATUS_LAST_SEQ_STEP_MASK   0x0F

#define ROC_ROBOT_IAQ_LEVEL_LIMIT_VALUE             3

#define ROC_ROBOT_CTRL_MEASURE_START                'O'
#define ROC_ROBOT_CTRL_MEASURE_STOP                 'P'


ROC_RESULT RocZmod4410Init(void);
ROC_RESULT RocZmode4410MeasureStart(void);
ROC_RESULT RocZmode4410MeasureStop(void);
uint32_t RocZmod4410SensorStatusIsChange(void);


#endif

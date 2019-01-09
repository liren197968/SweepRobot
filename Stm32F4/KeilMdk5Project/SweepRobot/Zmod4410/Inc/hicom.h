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
 * @file    hicom.h
 * @brief   Application programming interface for IDT ZMOD4xxx gas sensor
 * @version 0.0.1
 * @date    2017-02-06 10:54:20
 * @author  rschreib
 * @author  fnaepelt
 *
 */

#ifndef __HICOM_H__
#define __HICOM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FTCI2C.h"


#define HICOM_NAME          "Dual RS232-HS A"
#define HICOM_I2C_SPEED     100000

// enable debug messages
#define HICOM_DEBUG_MESSAGES

typedef FTC_STATUS hicom_status_t;
typedef FTC_HANDLE hicom_handle_t;

// API functions
hicom_status_t hicom_open(hicom_handle_t *p_handle);
hicom_status_t hicom_close(hicom_handle_t handle);
hicom_status_t hicom_power_on(hicom_handle_t handle);
hicom_status_t hicom_power_off(hicom_handle_t handle);
hicom_status_t hicom_get_error_string(hicom_status_t Status, char *buf, DWORD len);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __HICOM_H__

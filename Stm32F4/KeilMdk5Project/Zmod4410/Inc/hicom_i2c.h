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
 * @file    hicom_i2c.h
 * @brief   HiCom specific function definitions
 * @version 0.0.1
 * @date    2018-05-23
 * @author  fnaepelt
 */

#ifndef _HICOM_I2C_H
#define _HICOM_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <windows.h>
#include "hicom.h"


/**
 * @brief set a handle to the hardware
 * @param[in] hc_handle handle to the hardware
 */
void set_hicom_handle(hicom_handle_t* hc_handle);

/**
 * @brief Sleep for some time. Depending on target and application this can \n
 *        be used to go into power down or to do task switching.
 * @param [in] ms will sleep for at least this number of milliseconds
 */
void hicom_sleep(uint32_t ms);

/* I2C communication */
/**
 * @brief Read a register over I2C
 * @param [in] i2c_addr 7-bit I2C slave address of the ZMOD44xx
 * @param [in] reg_addr address of internal register to read
 * @param [out] buf destination buffer; must have at least a size of len*uint8_t
 * @param [in] len number of bytes to read
 * @return error code
 */
int8_t hicom_i2c_read
(
        uint8_t i2c_addr,
        uint8_t reg_addr,
        uint8_t *buf,
        uint8_t len
);

/**
 * @brief Write a register over I2C using protocol described in IDT App Note \n
 *        ZMOD4xxx functional description.
 * @param [in] i2c_addr 7-bit I2C slave address of the ZMOD4xxx
 * @param [in] reg_addr address of internal register to write
 * @param [in] buf source buffer; must have at least a size of len*uint8_t
 * @param [in] len number of bytes to write
 * @return error code
 */
int8_t hicom_i2c_write
(
        uint8_t i2c_addr,
        uint8_t reg_addr,
        uint8_t *buf,
        uint8_t len
);

#ifdef __cplusplus
}
#endif

#endif // __ZMOD4XXX_TARGET_SPECIFIC__

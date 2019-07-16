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
 * @file    hicom_i2c.c
 * @brief   HiCom specific function definitions
 * @version 0.0.1
 * @date    2018-05-23
 * @author  fnaepelt
 */

#include "hicom_i2c.h"

static hicom_handle_t* hicom_handle;

void set_hicom_handle(hicom_handle_t* hc_handle)
{
    hicom_handle = hc_handle;
}

void hicom_sleep(uint32_t ms)
{
    Sleep(ms);
}

int8_t hicom_i2c_read
(
        uint8_t i2c_addr,
        uint8_t reg_addr,
        uint8_t *buf,
        uint8_t len
)
{
    hicom_status_t hc_status;
    uint8_t control_buf[2];
    char error_str[512];

    control_buf[0] = (i2c_addr << 1) | 1;
    control_buf[1] = reg_addr;

    hc_status = I2C_Read(*hicom_handle, (PWriteControlByteBuffer)control_buf, 2,
                TRUE, 20, BLOCK_READ_TYPE, (PReadDataByteBuffer)buf, len);
    if (FTC_SUCCESS != hc_status) {
        return hc_status;
    };
    return 0;
}

int8_t hicom_i2c_write(
        uint8_t i2c_addr,
        uint8_t reg_addr,
        uint8_t *buf,
        uint8_t len
)
{
    hicom_status_t hc_status;
    uint8_t control_buf[2];
    FTC_PAGE_WRITE_DATA page_write_data = {0, 0};
    char error_str[512];

    control_buf[0] = i2c_addr << 1;
    control_buf[1] = reg_addr;

    hc_status = I2C_Write(*hicom_handle, (PWriteControlByteBuffer)control_buf, 2,
            TRUE, 20, TRUE, PAGE_WRITE_TYPE, (PWriteDataByteBuffer)buf, len,
            TRUE, 20, &page_write_data);

    if (FTC_SUCCESS != hc_status) {
        return hc_status;
    };
    return 0;
}


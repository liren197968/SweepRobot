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
 * @file    zmod44xx.c
 * @brief   ZMOD44xx functions
 * @version 1.0.2
 * @date    2018-05-17
 * @author  fnaepelt
 */

#include "zmod44xx.h"


static zmod44xx_conf* init_conf;
static zmod44xx_conf* meas_conf;

int8_t zmod44xx_read_sensor_info(zmod44xx_dev_t* dev)
{
    int8_t ret = 0;
    uint8_t data[ZMOD44XX_LEN_PID];
    uint8_t status = 0;
    uint8_t cmd = 0;
    uint16_t i = 0;

    /* wait for sensor ready */
    do {

        ret = dev->write(dev->i2c_addr, ZMOD44XX_ADDR_CMD, &cmd, 1);
        if(ret) {
            return ERROR_I2C;
        }

        dev->delay_ms(200);

        ret = zmod44xx_read_status(dev, &status);
        if(ret) {
            return ret;
        }
        i++;
    } while ((0x00 != (status & 0x80)) && (i < 1000));

    if (1000 <= i) {
        return ERROR_GAS_TIMEOUT;
    }

    ret = dev->read(dev->i2c_addr, ZMOD44XX_ADDR_PID, data, ZMOD44XX_LEN_PID);
    if(ret) {
        return ERROR_I2C;
    }
    dev->pid = data[0] << 8 | data[1];

    ret = dev->read(dev->i2c_addr, ZMOD44XX_ADDR_CONF, dev->config,
                    ZMOD44XX_LEN_CONF);
    if(ret) {
        return ERROR_I2C;
    }

    switch (dev->pid) {

        case ZMOD4410_PID: {
            meas_conf = &zmod4410;
            init_conf = &zmod44xxi;

        } break;

        default: {
            return ERROR_SENSOR_UNSUPPORTED;
        } break;
    }

    return ZMOD44XX_OK;
}

int8_t zmod44xx_init_sensor(zmod44xx_dev_t* dev)
{
    int8_t ret = 0;
    uint8_t data[4] = {0};

    if (!init_conf) {
        return ERROR_CONFIG_MISSING;
    }

    /* prepare sensor */
    ret = dev->read(dev->i2c_addr, 0xB7, data, 1);
    if(ret) {
        return ERROR_I2C;
    }

    float hspf = (-((float)dev->config[2] * 256.0f + dev->config[3]) *
                 ((dev->config[4] + 640.0) * (dev->config[5] + 80.0) - 512000.0))
                 / 12288000.0;

    if ((0.0f > hspf) || (4096.0f < hspf)) {
        return ERROR_INIT_OUT_OF_RANGE;
    }

    data[0] = (uint8_t)((uint16_t)hspf >> 8);
    data[1] = (uint8_t)((uint16_t)hspf & 0x00FF);

    ret = dev->write(dev->i2c_addr, init_conf->h.addr, data, init_conf->h.len);
    if(ret) {
        return ERROR_I2C;
    }

    ret = dev->write(dev->i2c_addr, init_conf->d.addr, init_conf->d.data,
                    init_conf->d.len);
    if(ret) {
        return ERROR_I2C;
    }

    ret = dev->write(dev->i2c_addr, init_conf->m.addr, init_conf->m.data,
                    init_conf->m.len);
    if(ret) {
        return ERROR_I2C;
    }

    ret = dev->write(dev->i2c_addr, init_conf->s.addr, init_conf->s.data,
                    init_conf->s.len);
    if(ret) {
        return ERROR_I2C;
    }

    ret = dev->write(dev->i2c_addr, ZMOD44XX_ADDR_CMD, &init_conf->start, 1);
    if(ret) {
        return ERROR_I2C;
    }

    dev->delay_ms(20);

    ret = zmod44xx_read_status(dev, data);
    if (0x80 == (0x80 & data[0])) {
        return ERROR_GAS_TIMEOUT;
    }

    ret = dev->read(dev->i2c_addr, init_conf->r.addr, data, init_conf->r.len);
    if(ret) {
        return ERROR_I2C;
    }

    dev->mox_lr = (uint16_t)(data[0] << 8) | data[1];
    dev->mox_er = (uint16_t)(data[2] << 8) | data[3];

    return ZMOD44XX_OK;
}

int8_t zmod44xx_init_measurement(zmod44xx_dev_t* dev)
{
    int8_t ret = 0;
    uint8_t data[2] = {0};

    if (!meas_conf) {
        return ERROR_CONFIG_MISSING;
    }

    ret = dev->read(dev->i2c_addr, 0xB7, data, 1);
    if(ret) {
        return ERROR_I2C;
    }

    float hspf = (-((float)dev->config[2] * 256.0f + dev->config[3]) *
                 ((dev->config[4] + 640.0) * (dev->config[5] - 600.0) - 512000.0))
                 / 12288000.0;
    if ((0.0f > hspf) || (4096.0f < hspf)) {
        return ERROR_INIT_OUT_OF_RANGE;
    }

    data[0] = (uint8_t)((uint16_t)hspf >> 8);
    data[1] = (uint8_t)((uint16_t)hspf & 0x00FF);
    ret = dev->write(dev->i2c_addr, meas_conf->h.addr, data, meas_conf->h.len);
    if(ret) {
        return ERROR_I2C;
    }

    ret = dev->write(dev->i2c_addr, meas_conf->d.addr, meas_conf->d.data,
                    meas_conf->d.len);
    if(ret) {
        return ERROR_I2C;
    }

    ret = dev->write(dev->i2c_addr, meas_conf->m.addr, meas_conf->m.data,
                    meas_conf->m.len);
    if(ret) {
        return ERROR_I2C;
    }

    ret = dev->write(dev->i2c_addr, meas_conf->s.addr, meas_conf->s.data,
                    meas_conf->s.len);
    if(ret) {
        return ERROR_I2C;
    }

    return ZMOD44XX_OK;
}

int8_t zmod44xx_start_measurement(zmod44xx_dev_t* dev)
{
    int8_t ret = 0;

    ret = dev->write(dev->i2c_addr, ZMOD44XX_ADDR_CMD, &meas_conf->start, 1);
    if (ret) {
        return ERROR_I2C;
    }
    return ZMOD44XX_OK;
}

int8_t zmod44xx_read_status(zmod44xx_dev_t* dev, uint8_t* status)
{
    int8_t ret;
    uint8_t st;

    ret = dev->read(dev->i2c_addr, ZMOD44XX_ADDR_STATUS, &st, 1);
    if (0 != ret) {
        return ret;
    }
    *status = st;
    return ZMOD44XX_OK;
}

int8_t zmod44xx_read_rmox(zmod44xx_dev_t* dev, float* rmox)
{
    int8_t ret = 0;
    uint8_t data[2] = {0};
    uint16_t adc_result = 0;

    ret = dev->read(dev->i2c_addr, meas_conf->r.addr, data, meas_conf->r.len);
    if (ret) {
        return ERROR_I2C;
    }

    adc_result = (uint16_t)(data[0] << 8) | data[1];

    ret = dev->read(dev->i2c_addr, 0xB7, data, 1);
    if (ret) {
        return ERROR_I2C;
    }

    if (0 != data[0]) {
        return ERROR_SENSOR;
    }

    if (0.0 > (adc_result - dev->mox_lr)) {
        *rmox = 1e-3;
    } else if (0.0 >= (dev->mox_er - adc_result)) {
        *rmox = 10e9;
    } else {
        *rmox = dev->config[1] * 1000.0 * (adc_result - dev->mox_lr) /
                                          (dev->mox_er - adc_result);
    }

    return ZMOD44XX_OK;
}

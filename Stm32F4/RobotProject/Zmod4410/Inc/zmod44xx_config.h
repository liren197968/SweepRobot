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
 * @file   zmod44xx_config.h
 * @brief  ZMOD44xx configuration
 * @version 1.0.2
 * @date   2018-05-25
 * @author fnaepelt
 */

#ifndef _ZMOD44XX_CONFIG_H
#define _ZMOD44XX_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "zmod44xx_types.h"

/* ZMOD4410 measurement configuration */
static uint8_t data_set_4410[] =  {0x20, 0x04, 0x40, 0x09, 0x03,
                                   0x00, 0x00, 0x80, 0x08};

/* ZMOD4410 initialization configuration */
static uint8_t data_set_4410i[] = {0x00, 0x28, 0xC3, 0xE3,
                                   0x00, 0x00, 0x80, 0x40};

/**
 * @brief ZMOD4410 configuration
 */
static zmod44xx_conf zmod4410  = {
        .name = "4410",
        .start = 0xC0,
        .h = {.addr = 0x40, .len = 2},
        .d = {.addr = 0x50, .len = 4, .data = &data_set_4410[0]},
        .m = {.addr = 0x60, .len = 1, .data = &data_set_4410[4]},
        .s = {.addr = 0x68, .len = 4, .data = &data_set_4410[5]},
        .r = {.addr = 0x99, .len = 2}

};

/**
 * @brief ZMOD44XX sensor initialization configuration
 */
static zmod44xx_conf zmod44xxi = {
        .name = "4410i",
        .start = 0x80,
        .h = {.addr = 0x40, .len = 2},
        .d = {.addr = 0x50, .len = 2, .data = &data_set_4410i[0]},
        .m = {.addr = 0x60, .len = 2, .data = &data_set_4410i[2]},
        .s = {.addr = 0x68, .len = 4, .data = &data_set_4410i[4]},
        .r = {.addr = 0x97, .len = 4}

};

#ifdef __cplusplus
}
#endif

#endif //  _ZMOD44XX_CONFIG_H

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
 * @file    main.c
 * @brief   This is an example for the ZMOD4410 gas sensor module.
 * @version 1.0.2
 * @date    2018-05-15
 * @author  fnaepelt
 **/
#include "stm32f4xx_hal.h"

#include "i2c.h"

#include "RocLog.h"
#include "RocZmod4410.h"


zmod44xx_dev_t g_dev;   /* ZMOD4410 IIC device */

/*********************************************************************************
 *  Description:
 *              The ZMOD4410 gas sensor odor status value, which will be used to
 *              show the gas level status, if the gas level change suddenly, it
 *              will be setted ROC_TRUE.
 *
 *  Author:
 *              ROC LiRen(2019.01.09)
**********************************************************************************/
static uint32_t g_Zmod4410SensorStatusChange = ROC_FALSE;


/*********************************************************************************
 *  Description:
 *              Write ZMOD4410 register from IIC communication
 *
 *  Parameter:
 *              SlaveAddr:  the address of slave device
 *              Reg:        the address or slave device register
 *              *BufferAddr:the pointer of data storage buffer address
 *
 *  Return:
 *              The ZMOD4410 write register status
 *
 *  Author:
 *              ROC LiRen(2019.01.05)
**********************************************************************************/
static int8_t RocZmode4410WriteReg(uint8_t SlaveAddr, uint8_t Reg, uint8_t *BufferAddr, uint8_t DatLen)
{
    HAL_StatusTypeDef   WriteStatus = HAL_OK;

    WriteStatus = HAL_I2C_Mem_Write(&hi2c2, (uint16_t)SlaveAddr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, BufferAddr, DatLen, 1000);
    if(HAL_OK != WriteStatus)
    {
        ROC_LOGE("IIC2 wirte reg is in error(%d)!", WriteStatus);
    }

    return WriteStatus;
}

/*********************************************************************************
 *  Description:
 *              Read ZMOD4410 register from IIC communication
 *
 *  Parameter:
 *              SlaveAddr:  the address of slave device
 *              Reg:        the address or slave device register
 *              *BufferAddr:the pointer of data storage buffer address
 *
 *  Return:
 *              The ZMOD4410 read register status
 *
 *  Author:
 *              ROC LiRen(2019.01.05)
**********************************************************************************/
static int8_t RocZmode4410ReadReg(uint8_t SlaveAddr, uint8_t Reg, uint8_t *BufferAddr, uint8_t DatLen)
{
    HAL_StatusTypeDef   ReadStatus = HAL_OK;

    ReadStatus = HAL_I2C_Mem_Read(&hi2c2, (uint16_t)SlaveAddr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, BufferAddr, DatLen, 1000);
    if(HAL_OK != ReadStatus)
    {
        ROC_LOGE("IIC2 reg reg is in error(%d)!", ReadStatus);
    }

    return (int8_t)ReadStatus;
}

/*********************************************************************************
 *  Description:
 *              Set the ZMOD4410 sensor status
 *
 *  Parameter:
 *              SensorStatus: the expected sensor status
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.01.09)
**********************************************************************************/
static void RocZmod4410SensorStatusSet(uint32_t SensorStatus)
{
    g_Zmod4410SensorStatusChange = SensorStatus;
}

/*********************************************************************************
 *  Description:
 *              Get the ZMOD4410 sensor status
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              The current ZMOD4410 sensor status
 *
 *  Author:
 *              ROC LiRen(2019.01.09)
**********************************************************************************/
uint32_t RocZmod4410SensorStatusIsChange(void)
{
    return g_Zmod4410SensorStatusChange;
}

/*********************************************************************************
 *  Description:
 *              Start the ZMOD4410 measurement
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              The ZMOD4410 measurement status
 *
 *  Author:
 *              ROC LiRen(2019.01.05)
**********************************************************************************/
ROC_RESULT RocZmode4410MeasureStart(void)
{
    int8_t ret;
    uint64_t count_time = 0;
    uint8_t zmod44xx_status;
    float r_mox;

    /* To work with the algorithms target specific libraries needs to be
     * downloaded from IDT webpage and included into the project. */
    uint8_t iaq;
    float eco2;
    float r_cda;
    float tvoc;
    control_signal_state_t cs_state;

    eco2_params eco2_par = {
            .min_co2 = 400,
            .max_co2 = 5000,
            .tvoc_to_eco2 = 800,
            .hot_wine_rate = 0.3,
            .open_window_rate = -0.05,
    };

    tvoc_params tvoc_par = {
            .A = 680034,
            .alpha = 0.7,
    };

    odor_params odor_par = {
            .alpha = 0.8,
            .stop_delay = 24,
            .threshold = 1.3,
            .tau = 720,
            .stabilization_samples = 15,
    };

    {
        /* INSTEAD OF POLLING THE INTERRUPT CAN BE USED FOR OTHER HW */
        /* wait until readout result is possible */
        while (LAST_SEQ_STEP != (zmod44xx_status & 0x07)) {
            g_dev.delay_ms(50);
            ret = zmod44xx_read_status(&g_dev, &zmod44xx_status);
            if(ret) {
                ROC_LOGE("zmod44xx_read_status error %d, exiting program!\r\n", ret);
                goto exit;
            }
        }

        count_time++;

        /* evaluate and show measurement results */
        ret = zmod44xx_read_rmox(&g_dev, &r_mox);
        if(ret) {
            ROC_LOGE("zmod44xx_read_rmox error %d, exiting program!\r\n", ret);
            goto exit;
        }

        /* To work with the algorithms target specific libraries needs to be
         * downloaded from IDT webpage and included into the project */
        ROC_LOGI("ZMOD4410 [%lld] measurement result is:", count_time);
        ROC_LOGI("Rmox  = %5.0f kOhm", (r_mox / 1000.0f));

        /* calculate clean dry air resistor */
        r_cda = r_cda_tracker(r_mox);
        ROC_LOGI("CDA %.3f", r_cda);

        /* calculate result to TVOC value */
        tvoc = calc_tvoc(r_mox, r_cda, &tvoc_par);
        ROC_LOGI("TVOC %f mg/m^3", tvoc);

        /* calculate IAQ index */
        iaq = calc_iaq(r_mox, r_cda, &tvoc_par);
        ROC_LOGI("IAQ %d", iaq);

        /* calculate estimated CO2 */
        eco2 = calc_eco2(tvoc, 3, &eco2_par);
        ROC_LOGI("eCO2 %f", eco2);

        /* get odor control signal */
        cs_state = calc_odor(r_mox, &odor_par);
        ROC_LOGI("odor control state %d \r\n", cs_state);

        RocZmod4410SensorStatusSet((uint32_t)cs_state);

        /* INSTEAD OF POLLING THE INTERRUPT CAN BE USED FOR OTHER HW */
        /* waiting for sensor ready */
        while (FIRST_SEQ_STEP != (zmod44xx_status & 0x07)) {
            g_dev.delay_ms(50);
            ret = zmod44xx_read_status(&g_dev, &zmod44xx_status);
            if(ret) {
                ROC_LOGE("Error %d, exiting program!\r\n", ret);
                goto exit;
            }
        }
    }

    return ret;

exit:
    /* ****** BEGIN TARGET SPECIFIC DEINITIALIZATION ****** */
    ROC_LOGE("ZMOD4410 measurement is in error! Be careful!");
    while(1);
    /* ****** END TARGET SPECIFIC DEINITIALIZATION ****** */
}

/*********************************************************************************
 *  Description:
 *              Stop the ZMOD4410 measurement
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              The ZMOD4410 measurement status
 *
 *  Author:
 *              ROC LiRen(2019.01.05)
**********************************************************************************/
ROC_RESULT RocZmode4410MeasureStop(void)
{
    return RET_OK;
}

/*********************************************************************************
 *  Description:
 *              ZMODE4410 init
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              The ZMOD4410 driver init status
 *
 *  Author:
 *              ROC LiRen(2019.01.05)
**********************************************************************************/
ROC_RESULT RocZmod4410Init(void)
{
    int8_t ret;
    uint8_t zmod44xx_status;

    /* These are the hardware handles which needs to be adjusted to specific HW */
    /* Set initial hardware parameter */
    g_dev.read = RocZmode4410ReadReg;
    g_dev.write = RocZmode4410WriteReg;
    g_dev.delay_ms = HAL_Delay;
    g_dev.i2c_addr = ZMOD4410_I2C_ADDRESS;

    /* initialize and start sensor */
    ret = zmod44xx_read_sensor_info(&g_dev);
    if(ret) {
        ROC_LOGE("zmod44xx_read_sensor_info error %d, exiting program!\r\n", ret);
        goto exit;
    }

    ret = zmod44xx_init_sensor(&g_dev);
    if(ret) {
        ROC_LOGE("zmod44xx_init_sensor error %d, exiting program!\r\n", ret);
        goto exit;
    }

    ret = zmod44xx_init_measurement(&g_dev);
    if(ret) {
        ROC_LOGE("zmod44xx_init_measurement error %d, exiting program!\r\n", ret);
        goto exit;
    }

    ret = zmod44xx_start_measurement(&g_dev);
    if(ret) {
        ROC_LOGE("zmod44xx_start_measurement error %d, exiting program!\r\n", ret);
        goto exit;
    }
    /* Wait for initialization finished. */
    do {
        g_dev.delay_ms(50);
        ret = zmod44xx_read_status(&g_dev, &zmod44xx_status);
        if(ret) {
            ROC_LOGE("zmod44xx_read_status error %d, exiting program!\r\n", ret);
            goto exit;
        }
    } while (FIRST_SEQ_STEP != (zmod44xx_status & 0x07));

    ROC_LOGI("ZMOD4410 measurements init is successful.");

    return ret;

exit:
    /* ****** BEGIN TARGET SPECIFIC DEINITIALIZATION ****** */
    ROC_LOGE("ZMOD4410 init is in error! Be careful!");
    while(1);
    /* ****** END TARGET SPECIFIC DEINITIALIZATION ****** */
}


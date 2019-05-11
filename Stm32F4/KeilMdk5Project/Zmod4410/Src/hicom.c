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
 * @file    hicom.c
 * @brief   application programming interface for IDT ZMOD4xxx gas sensor
 * @version 0.0.1
 * @date    2017-02-06 10:54:20
 * @author  rschreib
 * @author  fnaepelt
 *
 ********************************************************************/

#include "hicom.h"

static FTC_INPUT_OUTPUT_PINS low_pins_vdd = {
    //  InputOutputState, LowHighState
    FALSE, FALSE,  //  low pin 1
    TRUE, FALSE,   //  low pin 2
    TRUE, FALSE,   //  low pin 3
    TRUE, TRUE,    //  low pin 4: VDD
};

static FTC_INPUT_OUTPUT_PINS low_pins = {
    // InputOutputState, LowHighState
    FALSE, FALSE,   // low pin 1
    TRUE, FALSE,    // low pin 2
    TRUE, FALSE,    // low pin 3
    TRUE, FALSE,    // low pin 4: VDD
};

static FTH_INPUT_OUTPUT_PINS high_pins = {
    // InputOutputState, LowHighState
    TRUE, FALSE,    // high pin 1: Res_N
    TRUE, FALSE,    // high pin 2
    TRUE, FALSE,    // high pin 3
    TRUE, FALSE,    // high pin 4
    FALSE, FALSE,   // high pin 5
    FALSE, FALSE,   // high pin 6
    FALSE, FALSE,   // high pin 7
    FALSE, FALSE,   // high pin 8
};

static FTH_INPUT_OUTPUT_PINS high_pins_res_n = {
    //  InputOutputState, LowHighState
    TRUE, TRUE,   //  high pin 1: Res_N
    TRUE, FALSE,   //  high pin 2
    TRUE, FALSE,   //  high pin 3
    TRUE, FALSE,   //  high pin 4
    FALSE, FALSE,  //  high pin 5
    FALSE, FALSE,  //  high pin 6
    FALSE, FALSE,  //  high pin 7
    FALSE, FALSE,  //  high pin 8
};

hicom_status_t hicom_open(hicom_handle_t *p_handle)
{
    hicom_status_t ftdi_status;
    DWORD num_hi_speed_devices;
    DWORD dev_idx;
    char dev_name[200];
    DWORD loc_id;
    char channel[8];
    DWORD hi_speed_device_type;
    DWORD clock_freq;

    ftdi_status = I2C_GetNumHiSpeedDevices(&num_hi_speed_devices);
    if (FTC_SUCCESS != ftdi_status) {
        return ftdi_status;
    }

#ifdef HICOM_DEBUG_MESSAGES
    printf("Dbg (HiCom): NumHiSpeedDevices = %lu\n", num_hi_speed_devices);
#endif

    if (num_hi_speed_devices < 1) {
        return FTC_DEVICE_NOT_FOUND;
    }

    // search for HiCom device
    for (dev_idx = 0; dev_idx < num_hi_speed_devices; dev_idx++){
        // ask for device name
        ftdi_status = I2C_GetHiSpeedDeviceNameLocIDChannel(dev_idx, dev_name,
                200, &loc_id, channel, 8, &hi_speed_device_type);
        if (FTC_SUCCESS != ftdi_status) {
            return ftdi_status;
        }

#ifdef HICOM_DEBUG_MESSAGES
        printf("Dbg (HiCom): DevIdx=%lu, DevName=%s, LocID=%lu, Channel=%s, HiSpeedDeviceType=%lu\n",
               dev_idx, dev_name, loc_id, channel,hi_speed_device_type);
#endif
        if (strcmp(dev_name, HICOM_NAME) == 0) {
#ifdef HICOM_DEBUG_MESSAGES
            printf("Dbg (HiCom): found matching device.\n");
#endif
           //  found right device!
           //  open it
            ftdi_status = I2C_OpenHiSpeedDevice(dev_name, loc_id, channel,
                                                                    p_handle);
            if (FTC_SUCCESS != ftdi_status) {
                return ftdi_status;
            }

           //  initialize
            ftdi_status = I2C_InitDevice(*p_handle, 74);
            if (FTC_SUCCESS != ftdi_status) {
                return ftdi_status;
            }

            ftdi_status = I2C_SetDeviceLatencyTimer(*p_handle, 2);
            if (FTC_SUCCESS != ftdi_status) {
                return ftdi_status;
            }

            ftdi_status = I2C_TurnOffDivideByFiveClockingHiSpeedDevice(*p_handle);
            if (FTC_SUCCESS != ftdi_status) {
                return ftdi_status;
            }

            ftdi_status = I2C_SetClock(*p_handle,
                    (DWORD)((60.0e6 / 2 / HICOM_I2C_SPEED) - 1), &clock_freq);
            if (FTC_SUCCESS != ftdi_status) {
                return ftdi_status;
            }

#ifdef HICOM_DEBUG_MESSAGES
            printf("Dbg (HiCom): ClockFreq=%lu\n", clock_freq);
#endif

            ftdi_status = I2C_SetLoopback(*p_handle, FALSE);
            if (FTC_SUCCESS != ftdi_status) {
                return ftdi_status;
            }

            ftdi_status = I2C_SetMode(*p_handle, FAST_MODE);
            if (FTC_SUCCESS != ftdi_status) {
                return ftdi_status;
            }

            ftdi_status = I2C_SetHiSpeedDeviceGPIOs(*p_handle, TRUE, &low_pins,
                                                        TRUE, &high_pins);
            if (FTC_SUCCESS != ftdi_status) {
                return ftdi_status;
            }

#ifdef HICOM_DEBUG_MESSAGES
            printf("Dbg (HiCom): Device opened.\n");
#endif
            return FTC_SUCCESS;
        }

    }
#ifdef HICOM_DEBUG_MESSAGES
    printf("Dbg (HiCom): Device not opened!\n");
#endif
    return FTC_DEVICE_NOT_FOUND;
}

hicom_status_t hicom_close(hicom_handle_t handle)
{
    hicom_status_t ftdi_status;

    ftdi_status = I2C_SetHiSpeedDeviceGPIOs(handle, TRUE, &low_pins, TRUE,
                                                          &high_pins);
    if (FTC_SUCCESS != ftdi_status) {
        return ftdi_status;
    }

    ftdi_status = I2C_Close(handle);
    if (FTC_SUCCESS != ftdi_status) {
        return ftdi_status;
    }

#ifdef HICOM_DEBUG_MESSAGES
    printf("Dbg (HiCom): Device closed.\n");
#endif
    return FTC_SUCCESS;
}

hicom_status_t hicom_power_on(hicom_handle_t handle)
{
    hicom_status_t ftdi_status;

   //  switch VDD on and wait 100ms
    ftdi_status = I2C_SetHiSpeedDeviceGPIOs(handle, TRUE, &low_pins_vdd,
                                            TRUE, &high_pins);
    if (FTC_SUCCESS != ftdi_status) {
        return ftdi_status;
    }
    Sleep(100);

    //  raise Res_N and wait 100ms
    ftdi_status = I2C_SetHiSpeedDeviceGPIOs(handle, TRUE, &low_pins_vdd, TRUE,
                                                          &high_pins_res_n);
    if (FTC_SUCCESS != ftdi_status) {
        return ftdi_status;
    }
    Sleep(100);

#ifdef HICOM_DEBUG_MESSAGES
    printf("Dbg (HiCom): Power on.\n");
#endif
    return FTC_SUCCESS;
}

hicom_status_t hicom_power_off(hicom_handle_t handle)
{
    hicom_status_t ftdi_status;

   //  switch VDD and Res_N off and wait 1000ms
    ftdi_status = I2C_SetHiSpeedDeviceGPIOs(handle, TRUE, &low_pins,
                                                            TRUE, &high_pins);
    if (FTC_SUCCESS != ftdi_status) {
        return ftdi_status;
    }
    Sleep(1000);

#ifdef HICOM_DEBUG_MESSAGES
    printf("Dbg (HiCom): Power off.\n");
#endif
    return FTC_SUCCESS;
}

hicom_status_t hicom_get_error_string(hicom_status_t status, char *buf, DWORD len)
{
    char lang[] = "EN";
    return I2C_GetErrorCodeString(lang, status, buf, len);
}

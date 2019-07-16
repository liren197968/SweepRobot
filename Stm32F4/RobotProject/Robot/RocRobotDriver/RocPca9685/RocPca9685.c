/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2018/12/15      1.0
********************************************************************************/
#include <string.h>

#include "gpio.h"
#include "i2c.h"

#include "RocLog.h"
#include "RocPca9685.h"


/*********************************************************************************
 *  Description:
 *              Enable PCA9685 output PWM pulse
 *
 *  Parameter:
 *              None
 *
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
**********************************************************************************/
void RocPca9685PwmOutEnable(void)
{
    HAL_GPIO_WritePin(ROC_PCA9685_A_EN_GPIO_PORT, ROC_PCA9685_A_EN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ROC_PCA9685_B_EN_GPIO_PORT, ROC_PCA9685_B_EN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ROC_PCA9685_C_EN_GPIO_PORT, ROC_PCA9685_C_EN_PIN, GPIO_PIN_RESET);
}

/*********************************************************************************

 *  Description:
 *              Disable PCA9685 output PWM pulse
 *
 *  Parameter:
 *              None
 *  
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
**********************************************************************************/
void RocPca9685PwmOutDisable(void)
{
    HAL_GPIO_WritePin(ROC_PCA9685_A_EN_GPIO_PORT, ROC_PCA9685_A_EN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ROC_PCA9685_B_EN_GPIO_PORT, ROC_PCA9685_B_EN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ROC_PCA9685_C_EN_GPIO_PORT, ROC_PCA9685_C_EN_PIN, GPIO_PIN_SET);
}

/*********************************************************************************
 *  Description:
 *              Write PCA9685 register from IIC communication
 *
 *  Parameter:
 *              SlaveAddr:  the address of slave device
 *              Reg:        the address or slave device register
 *              *BufferAddr:the pointer of data storage buffer address
 *
 *  Return:
 *              The PCA9685 write register status
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
**********************************************************************************/
static HAL_StatusTypeDef RocPca9685WriteReg(uint16_t SlaveAddr, uint16_t Reg, uint8_t *BufferAddr)
{
    HAL_StatusTypeDef   WriteStatus = HAL_OK;

    WriteStatus = HAL_I2C_Mem_Write(&hi2c1, SlaveAddr, Reg, I2C_MEMADD_SIZE_8BIT, BufferAddr, 1, 10);
    if(HAL_OK != WriteStatus)
    {
        ROC_LOGE("IIC1 write reg is in error(%d)!", WriteStatus);
    }

    return WriteStatus;
}

/*********************************************************************************
 *  Description:
 *              Read PCA9685 register from IIC communication
 *
 *  Parameter:
 *              SlaveAddr:  the address of slave device
 *              Reg:        the address or slave device register
 *              *BufferAddr:the pointer of data storage buffer address
 *
 *  Return:
 *              The PCA9685 read register status
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
**********************************************************************************/
static HAL_StatusTypeDef RocPca9685ReadReg(uint16_t SlaveAddr, uint16_t Reg, uint8_t *BufferAddr)
{
    HAL_StatusTypeDef   ReadStatus = HAL_OK;

    ReadStatus = HAL_I2C_Mem_Read(&hi2c1, SlaveAddr, Reg, I2C_MEMADD_SIZE_8BIT, BufferAddr, 1, 10);
    if(HAL_OK != ReadStatus)
    {
        ROC_LOGE("IIC1 read reg is in error(%d)!", ReadStatus);
    }

    return ReadStatus;
}

/*********************************************************************************
 *  Description:
 *              Set the PCA9685 output PWM pulse frequency
 *
 *  Parameter:
 *              SlaveAddr:  the address of slave device
 *              Freq:       the expected frequency of PCA9685 output PWM pulse
 *
 *  Return:
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
**********************************************************************************/
static HAL_StatusTypeDef RocPca9685SetPwmFreq(uint8_t SlaveAddr, uint8_t Freq)
{
    uint8_t             Prescale = 0, Prescaleval = 0;
    uint8_t             OldMode = 0, NewMode = 0;
    HAL_StatusTypeDef   WriteStatus = HAL_OK;

    Freq *= 0.92;                                                               // Correct for overshoot in the frequency setting

    Prescaleval = (uint8_t)(25000000 / (4096 * Freq));
    Prescaleval -= 1;
    Prescale = (uint8_t)(Prescaleval + 0.5);

    RocPca9685ReadReg(SlaveAddr, PCA9685_MODE1, &OldMode);

    NewMode = (OldMode & 0x7F) | 0x10;                                          // sleep
    RocPca9685WriteReg(SlaveAddr, PCA9685_MODE1, &NewMode);                     // go to sleep

    WriteStatus = RocPca9685WriteReg(SlaveAddr, PCA9685_PRESCALE, &Prescale);   // set the prescaler

    RocPca9685WriteReg(SlaveAddr, PCA9685_MODE1, &OldMode);

    OldMode = OldMode | 0xA1;
    RocPca9685WriteReg(SlaveAddr, PCA9685_MODE1, &OldMode);                     // This sets the MODE1 register to turn on auto increment

    return WriteStatus;
}

/*********************************************************************************
 *  Description:
 *              Sets pin without having to deal with on/off tick placement and
 *              properly handles a zero value as completely off
 *
 *  Parameter:
 *              SlaveAddr:  the address of slave device
 *              NumPin:     the expected pin to output PWM pulse
 *              LedOnTime:  the time to start turn on LED
 *              LedOffTime:  the time to start turn off LED
 *
 *  Return:
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
**********************************************************************************/
HAL_StatusTypeDef RocPca9685OutPwm(uint8_t SlaveAddr, uint8_t NumPin, uint16_t LedOnTime, uint16_t LedOffTime)
{
    uint8_t             Buffer[5];
    HAL_StatusTypeDef   WriteStatus = HAL_OK;

    Buffer[0] = LED0_ON_L + NumPin * 4;
    Buffer[1] = LedOnTime & 0xFFU;
    Buffer[2] = (LedOnTime >> 8U) & 0xFFU;
    Buffer[3] = LedOffTime & 0xFFU;
    Buffer[4] = (LedOffTime >> 8U) & 0xFFU;

    WriteStatus = HAL_I2C_Mem_Write(&hi2c1, SlaveAddr, Buffer[0], I2C_MEMADD_SIZE_8BIT, Buffer + 1, 4, 10);

    return WriteStatus;
}

/*********************************************************************************
 *  Description:
 *              Sets pin without having to deal with on/off tick placement and
 *              properly handles a zero value as completely off
 *
 *  Parameter:
 *              SlaveAddr:  the address of slave device
 *              NumPin:     the expected pin to output PWM pulse
 *              LedOnTime:  the time to start turn on LED
 *              LedOffTime:  the time to start turn off LED
 *
 *  Return:
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
**********************************************************************************/
HAL_StatusTypeDef RocPca9685OutPwmAll(uint8_t SlaveAddr, uint16_t *pPwmData, uint16_t PwmDataNum)
{
    uint8_t             i = 0;
    HAL_StatusTypeDef   WriteStatus = HAL_OK;
    uint8_t             Buffer[ROC_PCA9685_DATA_REG_NUM * PwmDataNum + 1];

    Buffer[0] = LED0_ON_L;

    for(i = 0; i < PwmDataNum; i++)
    {
        Buffer[i * ROC_PCA9685_DATA_REG_NUM + 1] = 0x00U;
        Buffer[i * ROC_PCA9685_DATA_REG_NUM + 2] = 0x00U;
        Buffer[i * ROC_PCA9685_DATA_REG_NUM + 3] = pPwmData[i] & 0xFFU;
        Buffer[i * ROC_PCA9685_DATA_REG_NUM + 4] = (pPwmData[i] >> 8U) & 0xFFU;
    }

    WriteStatus = HAL_I2C_Mem_Write(&hi2c1, SlaveAddr, Buffer[0], I2C_MEMADD_SIZE_8BIT, Buffer + 1, ROC_PCA9685_DATA_REG_NUM * PwmDataNum, 10);

    return WriteStatus;
}

/*********************************************************************************
 *  Description:
 *              Set the specific pin output PWM pulse
 *
 *  Parameter:
 *              SlaveAddr:  the address of slave device
 *              NumPin:     the expected pin to output PWM pulse
 *              Val:        the high pulse width of the output PWM pulse
 *              Invert:     the option for setting the high pulse width of the output PWM pulse
 *
 *  Return:
 *
 *  Author:
 *              ROC LiRen(2018.12.05)
**********************************************************************************/
HAL_StatusTypeDef RocPca9685SetPinOutPwm(uint8_t SlaveAddr, uint8_t NumPin, uint16_t Val, uint8_t Invert)
{
    HAL_StatusTypeDef   WriteStatus;

    if(Val > 4095)
    {
        Val = 4095;
    }

    if(Invert)
    {
        if(Val == 0)
        {
            WriteStatus = RocPca9685OutPwm(SlaveAddr, NumPin, 4096, 0);          // Special value for signal fully on.
        }
        else if (Val == 4095)
        {
            WriteStatus = RocPca9685OutPwm(SlaveAddr, NumPin, 0, 4096);          // Special value for signal fully off.
        }
        else
        {
            WriteStatus = RocPca9685OutPwm(SlaveAddr, NumPin, 0, 4095 - Val);
        }
    }
    else
    {
        if(Val == 4095)
        {
            WriteStatus = RocPca9685OutPwm(SlaveAddr, NumPin, 4096, 0);          // Special value for signal fully on.
        }
        else if (Val == 0)
        {
            WriteStatus = RocPca9685OutPwm(SlaveAddr, NumPin, 0, 4096);          // Special value for signal fully off.
        }
        else
        {
            WriteStatus = RocPca9685OutPwm(SlaveAddr, NumPin, 0, Val);
        }
    }

    return WriteStatus;
}

/*********************************************************************************
 *  Description:
 *              PCA9685 init
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              The PCA9685 driver init status
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
**********************************************************************************/
ROC_RESULT RocPca9685Init(void)
{
    uint8_t             InitDat = 0X00;
    ROC_RESULT          Ret = RET_OK;
    HAL_StatusTypeDef   WriteStatus = HAL_OK;

    RocPca9685PwmOutDisable();

    WriteStatus = RocPca9685WriteReg(PWM_ADDRESS_L, PCA9685_MODE1, &InitDat);
    while(HAL_OK != WriteStatus)
    {
        HAL_Delay(5);

        ROC_LOGE("Setting PCA9685 Mode is in error, and will reset it one more time!");

        HAL_I2C_DeInit(&hi2c1);     /* For the ST IIC BUSY flag hardware error */
        HAL_I2C_Init(&hi2c1);       /* Reset the IIC */

        WriteStatus = RocPca9685WriteReg(PWM_ADDRESS_L, PCA9685_MODE1, &InitDat);
    }

    RocPca9685WriteReg(PWM_ADDRESS_H, PCA9685_MODE1, &InitDat);

    WriteStatus = RocPca9685SetPwmFreq(PWM_ADDRESS_L, 50);
    if(HAL_OK != WriteStatus)
    {
        Ret = RET_ERROR;
        ROC_LOGE("Set low PCA9685 PWM frequent error(%d)!", WriteStatus);
    }

    WriteStatus = RocPca9685SetPwmFreq(PWM_ADDRESS_H, 50);
    if(HAL_OK != WriteStatus)
    {
        Ret = RET_ERROR;
        ROC_LOGE("Set High PCA9685 PWM frequent error(%d)!", WriteStatus);
    }

    if(RET_OK != Ret)
    {
        ROC_LOGE("PCA9685 init error(%d)!", Ret);
    }
    else
    {
        ROC_LOGI("PCA9685 module init is in success.");
    }

    return Ret;
}


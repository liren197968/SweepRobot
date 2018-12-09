#include "i2c.h"
#include "RocPca9685.h"

static void RocPca9685GpioInit(void)
{

}

void RocPca9685Enable(void)
{
    HAL_GPIO_WritePin(GPIOC, ROC_PCA9685_A_EN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, ROC_PCA9685_B_EN, GPIO_PIN_RESET);
}

void RocPca9685Disable(void)
{
    HAL_GPIO_WritePin(GPIOC, ROC_PCA9685_A_EN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, ROC_PCA9685_B_EN, GPIO_PIN_SET);
}

static HAL_StatusTypeDef RocPca9685WriteReg(uint8_t SlaveAddr, uint8_t Reg, uint8_t Dat)
{
    HAL_StatusTypeDef   WriteStatus;

    WriteStatus = HAL_I2C_Mem_Write(&hi2c1, SlaveAddr, Reg, I2C_MEMADD_SIZE_8BIT, &Dat, 1, 0x10);

    printf("WriteStatus: %d! \r\n", WriteStatus);
    return WriteStatus;
}

static HAL_StatusTypeDef RocPca9685ReadReg(uint8_t SlaveAddr, uint8_t Reg, uint8_t *Dat)
{
    HAL_StatusTypeDef   ReadStatus;

    ReadStatus = HAL_I2C_Mem_Read(&hi2c1, SlaveAddr, Reg, I2C_MEMADD_SIZE_8BIT, Dat, 1, 0x10);

    printf("ReadStatus: %d! \r\n", ReadStatus);
    return ReadStatus;
}

static void RocPca9685SetPwmFreq(uint8_t SlaveAddr, uint8_t Freq)
{
    uint8_t             Prescale = 0, Prescaleval = 0;
    uint8_t             OldMode = 0, NewMode = 0;

    Freq *= 0.92;                                                   // Correct for overshoot in the frequency setting

    Prescaleval = (uint8_t)(25000000/(4096 * Freq));
    Prescaleval -= 1;
    Prescale = (uint8_t)(Prescaleval + 0.5);

    RocPca9685ReadReg(SlaveAddr, PCA9685_MODE1, &OldMode);
    NewMode = (OldMode & 0x7F) | 0x10;                              // sleep

    RocPca9685WriteReg(SlaveAddr, PCA9685_MODE1, NewMode);          // go to sleep
    RocPca9685WriteReg(SlaveAddr, PCA9685_PRESCALE, Prescale);      // set the prescaler
    RocPca9685WriteReg(SlaveAddr, PCA9685_MODE1, OldMode);
    HAL_Delay(5);

    RocPca9685WriteReg(SlaveAddr, PCA9685_MODE1, OldMode | 0xA1);   // This sets the MODE1 register to turn on auto increment

    printf("Value1: %d! \r\n", OldMode | 0xA1);
    RocPca9685ReadReg(SlaveAddr, PCA9685_MODE1, &OldMode);
    printf("Value2: %d! \r\n", OldMode);
}

HAL_StatusTypeDef RocPca9685OutPwm(uint8_t SlaveAddr, uint8_t Num, uint16_t HigBitDat, uint16_t LowBitDat)
{
    uint8_t             Buffer[5];
    HAL_StatusTypeDef   WriteStatus;

    Buffer[0] = LED0_ON_L + Num * 4;
    Buffer[1] = HigBitDat & 0xFFU;
    Buffer[2] = (HigBitDat >> 8U) & 0xFFU;
    Buffer[3] = LowBitDat & 0xFFU;
    Buffer[4] = (LowBitDat >> 8U) & 0xFFU;

    WriteStatus = HAL_I2C_Mem_Write(&hi2c1, SlaveAddr, Buffer[0], I2C_MEMADD_SIZE_8BIT, &Buffer[1], 4, 0x10);

    return WriteStatus;
}

/*********************************************************************************
 *  Description:
 *              Sets pin without having to deal with on/off tick placement and
 *              properly handles a zero value as completely off
 *
 *  Parameter:
 *              Parameter:  Supports inverting the pulse for sinking to ground
 *
             Val:        Should be a value from 0 to 4095 inclusive
 *              Optional:   Invert
 *
 *  Return:
 *
 *  Author:
 *              ROC LiRen(2018.12.05)
**********************************************************************************/
void RocPca9685SetPin(uint8_t SlaveAddr, uint8_t Num, uint16_t Val, uint8_t Invert)
{
    if(Val > 4095)
    {
        Val = 4095;
    }

    if(Invert)
    {
        if(Val == 0)
        {
            RocPca9685OutPwm(SlaveAddr, Num, 4096, 0);          // Special value for signal fully on.
        }
        else if (Val == 4095)
        {
            RocPca9685OutPwm(SlaveAddr, Num, 0, 4096);          // Special value for signal fully off.
        }
        else
        {
            RocPca9685OutPwm(SlaveAddr, Num, 0, 4095 - Val);
        }
    }
    else
    {
        if(Val == 4095)
        {
            RocPca9685OutPwm(SlaveAddr, Num, 4096, 0);          // Special value for signal fully on.
        }
        else if (Val == 0)
        {
            RocPca9685OutPwm(SlaveAddr, Num, 0, 4096);          // Special value for signal fully off.
        }
        else 
        {
            RocPca9685OutPwm(SlaveAddr, Num, 0, Val);
        }
    }
}

void RocPca9685Init(void)
{
    uint8_t Dat = 0;

    RocPca9685GpioInit();
    RocPca9685Enable();

    RocPca9685WriteReg(PWM_ADDRESS_L, PCA9685_MODE1, 0x00);
    HAL_Delay(5);

    RocPca9685WriteReg(PWM_ADDRESS_H, PCA9685_MODE1, 0x00);
    HAL_Delay(5);
    RocPca9685ReadReg(PWM_ADDRESS_H, PCA9685_MODE1, &Dat);
    printf("Value: %d! \r\n", Dat);

    RocPca9685SetPwmFreq(PWM_ADDRESS_L, 50);
    RocPca9685SetPwmFreq(PWM_ADDRESS_H, 50);
}


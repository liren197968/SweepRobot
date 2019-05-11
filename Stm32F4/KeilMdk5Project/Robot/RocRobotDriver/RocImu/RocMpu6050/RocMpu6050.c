/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/04/15      1.0
********************************************************************************/

#include "i2c.h"

#include "RocLog.h"
#include "RocMpu6050.h"


/*********************************************************************************
 *  Description:
 *              Write serval data to MPU6050 register
 *
 *  Parameter:
 *              Addr: the MPU6050 address
 *              Reg:  the register of MPU6050
 *              Len:  the data length
 *              *Buf: the point to the storage buffer
 *
 *  Return:
 *              The write status
 *
 *  Author:
 *              ROC LiRen(2019.04.15)
**********************************************************************************/
uint8_t RocMpu6050WriteLen(uint8_t Addr, uint8_t Reg, uint8_t Len, uint8_t *Buf)
{
    HAL_StatusTypeDef   WriteStatus = HAL_OK;

    while(HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_BUSY_TX);

    WriteStatus = HAL_I2C_Mem_Write(&hi2c2, Addr << 1, Reg, I2C_MEMADD_SIZE_8BIT, Buf, Len, 10);
    while(HAL_OK != WriteStatus)
    {
        HAL_Delay(5);

        ROC_LOGE("Write MPU6050 is in busy, and will reset it one more time(%d)!", WriteStatus);

        HAL_I2C_DeInit(&hi2c2);     /* For the ST IIC BUSY flag hardware error */
        HAL_I2C_Init(&hi2c2);       /* Reset the IIC */

        WriteStatus = HAL_I2C_Mem_Write(&hi2c2, Addr << 1, Reg, I2C_MEMADD_SIZE_8BIT, Buf, Len, 10);
    }

    return WriteStatus;
}

/*********************************************************************************
 *  Description:
 *              Read serval data from MPU6050 register
 *
 *  Parameter:
 *              Addr: the MPU6050 address
 *              Reg:  the register of MPU6050
 *              Len:  the data length
 *              *Buf: the point to the storage buffer
 *
 *  Return:
 *              The read status
 *
 *  Author:
 *              ROC LiRen(2019.04.15)
**********************************************************************************/
uint8_t RocMpu6050ReadLen(uint8_t Addr, uint8_t Reg, uint8_t Len, uint8_t *Buf)
{
    HAL_StatusTypeDef   ReadStatus = HAL_OK;

    ReadStatus = HAL_I2C_Mem_Read(&hi2c2, Addr << 1, Reg, I2C_MEMADD_SIZE_8BIT, Buf, Len, 10);
    if(HAL_OK != ReadStatus)
    {
        ROC_LOGE("IIC2 read reg is in error(%d)!", ReadStatus);
    }

    return (uint8_t)ReadStatus;
}

/*********************************************************************************
 *  Description:
 *              Read MPU6050 register
 *
 *  Parameter:
 *              Reg: the register
 *              Dat: the write data
 *
 *  Return:
 *              The write status
 *
 *  Author:
 *              ROC LiRen(2019.04.15)
**********************************************************************************/
uint8_t RocMpu6050WriteByte(uint8_t Reg, uint8_t Dat)
{
    HAL_StatusTypeDef   WriteStatus = HAL_OK;

    WriteStatus = HAL_I2C_Mem_Write(&hi2c2, ROC_MPU6050_ADDRESS << 1, Reg, I2C_MEMADD_SIZE_8BIT, &Dat, 1, 10);
    while(HAL_OK != WriteStatus)
    {
        HAL_Delay(5);

        ROC_LOGE("Write MPU6050 is in busy, and will reset it one more time(%d)!", WriteStatus);

        HAL_I2C_DeInit(&hi2c2);     /* For the ST IIC BUSY flag hardware error */
        HAL_I2C_Init(&hi2c2);       /* Reset the IIC */

        WriteStatus = HAL_I2C_Mem_Write(&hi2c2, ROC_MPU6050_ADDRESS << 1, Reg, I2C_MEMADD_SIZE_8BIT, &Dat, 1, 10);
    }

    return (uint8_t)WriteStatus;
}

/*********************************************************************************
 *  Description:
 *              Read MPU6050 register
 *
 *  Parameter:
 *              Reg: the register
 *
 *  Return:
 *              The register data
 *
 *  Author:
 *              ROC LiRen(2019.04.15)
**********************************************************************************/
uint8_t RocMpu6050ReadByte(uint8_t Reg)
{
    uint8_t Dat;
    HAL_StatusTypeDef   ReadStatus = HAL_OK;

    ReadStatus = HAL_I2C_Mem_Read(&hi2c2, ROC_MPU6050_ADDRESS << 1, Reg, I2C_MEMADD_SIZE_8BIT, &Dat, 1, 10);
    if(HAL_OK != ReadStatus)
    {
        ROC_LOGE("IIC2 read reg is in error(%d)!", ReadStatus);
    }

    return Dat;
}

/*********************************************************************************
 *  Description:
 *              MPU6050 sensor register init
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              The init status
 *
 *  Author:
 *              ROC LiRen(2019.04.15)
**********************************************************************************/
static uint8_t RocMpu6050RegInit(void)
{
    uint8_t Res;

    RocMpu6050WriteByte(ROC_MPU6050_PWR_MGMT1_REG,0X80);
    RocMpu6050WriteByte(ROC_MPU6050_PWR_MGMT1_REG,0X00);
    RocMpu6050SetGyroFsr(3);
    RocMpu6050SetAccelFsr(0);
    RocMpu6050SetRate(50);
    RocMpu6050WriteByte(ROC_MPU6050_INT_EN_REG,0X00);
    RocMpu6050WriteByte(ROC_MPU6050_USER_CTRL_REG,0X00);
    RocMpu6050WriteByte(ROC_MPU6050_FIFO_EN_REG,0X00);
    RocMpu6050WriteByte(ROC_MPU6050_INTBP_CFG_REG,0X80);

    Res=RocMpu6050ReadByte(ROC_MPU6050_DEVICE_ID_REG);
    if(Res==ROC_MPU6050_ADDRESS)
    {
        RocMpu6050WriteByte(ROC_MPU6050_PWR_MGMT1_REG,0X01);
        RocMpu6050WriteByte(ROC_MPU6050_PWR_MGMT2_REG,0X00);
        RocMpu6050SetRate(50);
    }
    else
    {
        return 1;
    }

    return 0;
}

/*********************************************************************************
 *  Description:
 *              Set MPU6050 gyro fsr
 *
 *  Parameter:
 *              Fsr: gyro fsr
 *
 *  Return:
 *              The write status
 *
 *  Author:
 *              ROC LiRen(2019.04.15)
**********************************************************************************/
uint8_t RocMpu6050SetGyroFsr(uint8_t Fsr)
{
    return RocMpu6050WriteByte(ROC_MPU6050_GYRO_CFG_REG, Fsr << 3);
}

/*********************************************************************************
 *  Description:
 *              Set MPU6050 accel fsr
 *
 *  Parameter:
 *              Fsr: accel fsr
 *
 *  Return:
 *              The write status
 *
 *  Author:
 *              ROC LiRen(2019.04.15)
**********************************************************************************/
uint8_t RocMpu6050SetAccelFsr(uint8_t Fsr)
{
    return RocMpu6050WriteByte(ROC_MPU6050_ACCEL_CFG_REG, Fsr << 3);
}

/*********************************************************************************
 *  Description:
 *              Set MPU6050 gyro LPF
 *
 *  Parameter:
 *              Lpf: LPF
 *
 *  Return:
 *              The write status
 *
 *  Author:
 *              ROC LiRen(2019.04.15)
**********************************************************************************/
uint8_t RocMpu6050SetLpf(uint16_t Lpf)
{
    uint8_t Dat = 0;

    if(Lpf >= 188)  Dat = 1;
    else if(Lpf >= 98)  Dat = 2;
    else if(Lpf >= 42)  Dat = 3;
    else if(Lpf >= 20)  Dat = 4;
    else if(Lpf >= 10)  Dat = 5;
    else    Dat = 6;

    return RocMpu6050WriteByte(ROC_MPU6050_CFG_REG, Dat);
}

/*********************************************************************************
 *  Description:
 *              Set MPU6050 rate
 *
 *  Parameter:
 *              Rate: rate
 *
 *  Return:
 *              The write status
 *
 *  Author:
 *              ROC LiRen(2019.04.15)
**********************************************************************************/
uint8_t RocMpu6050SetRate(uint16_t Rate)
{
    uint8_t Dat;

    if(Rate > 1000) Rate = 1000;
    if(Rate < 4)    Rate = 4;

    Dat = 1000 / Rate - 1;
    Dat = RocMpu6050WriteByte(ROC_MPU6050_SAMPLE_RATE_REG, Dat);

    return RocMpu6050SetLpf(Rate / 2);
}

/*********************************************************************************
 *  Description:
 *              Get MPU6050 temperature
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              The temperature
 *
 *  Author:
 *              ROC LiRen(2019.04.15)
**********************************************************************************/
uint16_t RocMpu6050GetTemperature(void)
{
    uint8_t     Buf[2];
    uint16_t    Raw;
    float       Temp;

    RocMpu6050ReadLen(ROC_MPU6050_ADDRESS, ROC_MPU6050_TEMP_OUTH_REG, 2, Buf);

    Raw = ((uint16_t)Buf[0] << 8) | Buf[1];
    Temp = 36.53 + ((double)Raw) / 340;

    return Temp * 100;
}

/*********************************************************************************
 *  Description:
 *              Get MPU6050 temperature
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              The temperature
 *
 *  Author:
 *              ROC LiRen(2019.04.15)
**********************************************************************************/
uint8_t RocMpu6050GetGyroscope(uint16_t *Gx, uint16_t *Gy, uint16_t *Gz)
{
    uint8_t Buf[6];
    uint8_t Res;

    Res=RocMpu6050ReadLen(ROC_MPU6050_ADDRESS, ROC_MPU6050_GYRO_XOUTH_REG, 6, Buf);
    if(Res == 0)
    {
        *Gx = ((uint16_t)Buf[0] << 8) | Buf[1];
        *Gy = ((uint16_t)Buf[2] << 8) | Buf[3];
        *Gz = ((uint16_t)Buf[4] << 8) | Buf[5];
    }

    return Res;
}

/*********************************************************************************
 *  Description:
 *              Get MPU6050 temperature
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              The temperature
 *
 *  Author:
 *              ROC LiRen(2019.04.15)
**********************************************************************************/
uint8_t RocMpu6050GetAccelerometer(uint16_t *Ax, uint16_t *Ay, uint16_t *Az)
{
    uint8_t Buf[6];
    uint8_t Res;

    Res = RocMpu6050ReadLen(ROC_MPU6050_ADDRESS, ROC_MPU6050_ACCEL_XOUTH_REG, 6, Buf);
    if(Res == 0)
    {
        *Ax = ((uint16_t)Buf[0] << 8) | Buf[1];
        *Ay = ((uint16_t)Buf[2] << 8) | Buf[3];
        *Az = ((uint16_t)Buf[4] << 8) | Buf[5];
    }

    return Res;
}

/*********************************************************************************
 *  Description:
 *              Get current MPU6050 euler angle
 *
 *  Parameter:
 *              *Pitch: the pointer to the pitch axis angle data
 *              *Roll:  the pointer to the roll axis angle data
 *              *Yaw:   the pointer to the Yaw axis angle data
 *
 *  Return:
 *              The PCA9685 driver init status
 *
 *  Author:
 *              ROC LiRen(2019.04.15)
**********************************************************************************/
ROC_RESULT RocMpu6050EulerAngleGet(float *Pitch, float *Roll, float *Yaw)
{
    ROC_RESULT Ret = RET_OK;

    Ret = (ROC_RESULT)mpu_dmp_get_data(Pitch, Roll, Yaw);

    return Ret;
}

/*********************************************************************************
 *  Description:
 *              MPU6050 sensor and DMP init
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.15)
**********************************************************************************/
ROC_RESULT RocMpu6050Init(void)
{
    ROC_RESULT Ret = RET_OK;

    Ret = (ROC_RESULT)RocMpu6050RegInit();
    if(RET_OK !=Ret)
    {
        ROC_LOGE("MPU6050 register init is in error!");
        return ROC_MPU6050_INIT_ERROR;
    }
    else
    {
        ROC_LOGI("MPU6050 register init is in success");
    }

    Ret = (ROC_RESULT)mpu_dmp_init();
    if(RET_OK !=Ret)
    {
        ROC_LOGE("MPU6050 DMP init is in error!");
        return ROC_MPU6050_DMP_INIT_ERROR;
    }
    else
    {
        ROC_LOGI("MPU6050 DMP init is in success");
    }

    ROC_LOGI("MPU6050 init is in success");

    return RET_OK;
}


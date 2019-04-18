/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/04/15      1.0
********************************************************************************/

#include "i2c.h"

#include "RocLog.h"
#include "RocMpu6050.h"


uint8_t MPU_Init(void)
{
	uint8_t res;

	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050 
	MPU_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(0);					//加速度传感器,±2g
	MPU_Set_Rate(50);						//设置采样率50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//器件ID正确
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
		MPU_Set_Rate(50);						//设置采样率为50Hz
 	}else return 1;
	return 0;
}

uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}

uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}

uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器  
}

uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

short MPU_Get_Temperature(void)
{
    uint8_t buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((uint16_t)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}

uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((uint16_t)buf[0]<<8)|buf[1];  
		*gy=((uint16_t)buf[2]<<8)|buf[3];  
		*gz=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;;
}

uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];  
		*ay=((uint16_t)buf[2]<<8)|buf[3];  
		*az=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;;
}

uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    HAL_StatusTypeDef   WriteStatus = HAL_OK;

    WriteStatus = HAL_I2C_Mem_Write(&hi2c2, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 10);
    while(HAL_OK != WriteStatus)
    {
        HAL_Delay(5);

        ROC_LOGE("Setting PCA9685 Mode is in error, and will reset it one more time!");

        HAL_I2C_DeInit(&hi2c2);     /* For the ST IIC BUSY flag hardware error */
        HAL_I2C_Init(&hi2c2);       /* Reset the IIC */

        WriteStatus = HAL_I2C_Mem_Write(&hi2c2, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 10);
    }

    return WriteStatus;
} 

uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    HAL_StatusTypeDef   ReadStatus = HAL_OK;

    ReadStatus = HAL_I2C_Mem_Read(&hi2c2, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 10);
    if(HAL_OK != ReadStatus)
    {
        ROC_LOGE("IIC2 read reg is in error(%d)!", ReadStatus);
    }

    return (uint8_t)ReadStatus;
}

uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data)
{
    HAL_StatusTypeDef   WriteStatus = HAL_OK;

    WriteStatus = HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
    if(HAL_OK != WriteStatus)
    {
        ROC_LOGE("IIC2 write reg is in error(%d)!", WriteStatus);
    }

    return (uint8_t)WriteStatus;
}

uint8_t MPU_Read_Byte(uint8_t reg)
{
    uint8_t Dat;
    HAL_StatusTypeDef   ReadStatus = HAL_OK;

    ReadStatus = HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &Dat, 1, 10);
    if(HAL_OK != ReadStatus)
    {
        ROC_LOGE("IIC2 read reg is in error(%d)!", ReadStatus);
    }

    return Dat;
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

    if(Ret = MPU_Init())
    {
        ROC_LOGE("MPU6050 init is in error!");
        return MPU_INIT_ERROR;
    }

    if(Ret = mpu_dmp_init())
    {
        ROC_LOGE("MPU6050 DMP init is in error!");
        return DMP_INIT_ERROR;
    }

    if(RET_OK != Ret)
    {
        ROC_LOGE("MPU6050 init is in error!");
    }

    return RET_OK;
}


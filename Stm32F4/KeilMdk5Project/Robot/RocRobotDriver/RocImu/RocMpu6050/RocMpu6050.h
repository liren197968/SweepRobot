#ifndef __ROC_MPU6050_H
#define __ROC_MPU6050_H


#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#include "RocError.h"
#include "RocSimulatedI2c.h"


#define ROC_MPU6050_INIT_ERROR              -1
#define ROC_MPU6050_DMP_INIT_ERROR          -2


//#define ROC_MPU6050_ACCEL_OFFS_REG        0X06
//#define ROC_MPU6050_PROD_ID_REG           0X0C
#define ROC_MPU6050_SELF_TESTX_REG          0X0D
#define ROC_MPU6050_SELF_TESTY_REG          0X0E
#define ROC_MPU6050_SELF_TESTZ_REG          0X0F
#define ROC_MPU6050_SELF_TESTA_REG          0X10
#define ROC_MPU6050_SAMPLE_RATE_REG         0X19
#define ROC_MPU6050_CFG_REG                 0X1A
#define ROC_MPU6050_GYRO_CFG_REG            0X1B
#define ROC_MPU6050_ACCEL_CFG_REG           0X1C
#define ROC_MPU6050_MOTION_DET_REG          0X1F
#define ROC_MPU6050_FIFO_EN_REG             0X23
#define ROC_MPU6050_I2CMST_CTRL_REG         0X24
#define ROC_MPU6050_I2CSLV0_ADDR_REG        0X25
#define ROC_MPU6050_I2CSLV0_REG             0X26
#define ROC_MPU6050_I2CSLV0_CTRL_REG        0X27
#define ROC_MPU6050_I2CSLV1_ADDR_REG        0X28
#define ROC_MPU6050_I2CSLV1_REG             0X29
#define ROC_MPU6050_I2CSLV1_CTRL_REG        0X2A
#define ROC_MPU6050_I2CSLV2_ADDR_REG        0X2B
#define ROC_MPU6050_I2CSLV2_REG             0X2C
#define ROC_MPU6050_I2CSLV2_CTRL_REG        0X2D
#define ROC_MPU6050_I2CSLV3_ADDR_REG        0X2E
#define ROC_MPU6050_I2CSLV3_REG	            0X2F
#define ROC_MPU6050_I2CSLV3_CTRL_REG        0X30
#define ROC_MPU6050_I2CSLV4_ADDR_REG        0X31
#define ROC_MPU6050_I2CSLV4_REG             0X32
#define ROC_MPU6050_I2CSLV4_DO_REG          0X33
#define ROC_MPU6050_I2CSLV4_CTRL_REG        0X34
#define ROC_MPU6050_I2CSLV4_DI_REG          0X35

#define ROC_MPU6050_I2CMST_STA_REG          0X36
#define ROC_MPU6050_INTBP_CFG_REG           0X37
#define ROC_MPU6050_INT_EN_REG              0X38
#define ROC_MPU6050_INT_STA_REG             0X3A

#define ROC_MPU6050_ACCEL_XOUTH_REG         0X3B
#define ROC_MPU6050_ACCEL_XOUTL_REG         0X3C
#define ROC_MPU6050_ACCEL_YOUTH_REG         0X3D
#define ROC_MPU6050_ACCEL_YOUTL_REG         0X3E
#define ROC_MPU6050_ACCEL_ZOUTH_REG         0X3F
#define ROC_MPU6050_ACCEL_ZOUTL_REG         0X40

#define ROC_MPU6050_TEMP_OUTH_REG           0X41
#define ROC_MPU6050_TEMP_OUTL_REG           0X42

#define ROC_MPU6050_GYRO_XOUTH_REG          0X43
#define ROC_MPU6050_GYRO_XOUTL_REG          0X44
#define ROC_MPU6050_GYRO_YOUTH_REG          0X45
#define ROC_MPU6050_GYRO_YOUTL_REG          0X46
#define ROC_MPU6050_GYRO_ZOUTH_REG          0X47
#define ROC_MPU6050_GYRO_ZOUTL_REG          0X48

#define ROC_MPU6050_I2CSLV0_DO_REG          0X63
#define ROC_MPU6050_I2CSLV1_DO_REG          0X64
#define ROC_MPU6050_I2CSLV2_DO_REG          0X65
#define ROC_MPU6050_I2CSLV3_DO_REG          0X66

#define ROC_MPU6050_I2CMST_DELAY_REG        0X67
#define ROC_MPU6050_SIGPATH_RST_REG         0X68
#define ROC_MPU6050_MDETECT_CTRL_REG        0X69
#define ROC_MPU6050_USER_CTRL_REG           0X6A
#define ROC_MPU6050_PWR_MGMT1_REG           0X6B
#define ROC_MPU6050_PWR_MGMT2_REG           0X6C
#define ROC_MPU6050_FIFO_CNTH_REG           0X72
#define ROC_MPU6050_FIFO_CNTL_REG           0X73
#define ROC_MPU6050_FIFO_RW_REG             0X74
#define ROC_MPU6050_DEVICE_ID_REG           0X75
 

#define ROC_MPU6050_ADDRESS                 0X68

//#define ROC_MPU6050_READ_ADDRESS          0XD1
//#define ROC_MPU6050_WRITE_ADDRESS         0XD0


uint8_t RocMpu6050RegInit(void);
uint8_t RocMpu6050WriteLen(uint8_t Addr, uint8_t Reg, uint8_t Len, uint8_t *Buf);
uint8_t RocMpu6050ReadLen(uint8_t Addr, uint8_t Reg, uint8_t Len, uint8_t *Buf);
uint8_t RocMpu6050WriteByte(uint8_t Reg,uint8_t Dat);
uint8_t RocMpu6050ReadByte(uint8_t Reg);

uint8_t RocMpu6050SetGyroFsr(uint8_t Fsr);
uint8_t RocMpu6050SetAccelFsr(uint8_t Fsr);
uint8_t RocMpu6050SetLpf(uint16_t Lpf);
uint8_t RocMpu6050SetRate(uint16_t Rate);
uint8_t RocMPU6050SetFifo(uint8_t Sens);

uint16_t RocMpu6050GetTemperature(void);
uint8_t RocMpu6050GetGyroscope(uint16_t *Gx, uint16_t *Gy, uint16_t *Gz);
uint8_t RocMpu6050GetAccelerometer(uint16_t *Ax, uint16_t *Ay, uint16_t *Az);

ROC_RESULT RocMpu6050Init(void);
ROC_RESULT RocMpu6050EulerAngleGet(float *Pitch, float *Roll, float *Yaw);


#endif


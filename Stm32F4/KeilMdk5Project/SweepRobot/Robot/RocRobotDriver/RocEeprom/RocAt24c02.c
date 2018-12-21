#include <string.h>

#include "i2c.h"

#include "RocLog.h"
#include "RocAt24c02.h"


#define ADDR_AT24C02_Write  0xA0
#define ADDR_AT24C02_Read   0xA1


uint8_t I2cBufferWrite[256];
uint8_t I2cBufferRead[256];


static HAL_StatusTypeDef RocAt24c02WriteReg(uint16_t SlaveAddr, uint16_t Reg, uint8_t *BufferAddr, uint16_t DatNum)
{
    HAL_StatusTypeDef   WriteStatus;

    WriteStatus = HAL_I2C_Mem_Write(&hi2c1, SlaveAddr, Reg, I2C_MEMADD_SIZE_8BIT, BufferAddr, DatNum, 1000);

    return WriteStatus;
}

static HAL_StatusTypeDef RocAt24c02ReadReg(uint16_t SlaveAddr, uint16_t Reg, uint8_t *BufferAddr, uint16_t DatNum)
{
    HAL_StatusTypeDef   ReadStatus;

    ReadStatus = HAL_I2C_Mem_Read(&hi2c1, SlaveAddr, Reg, I2C_MEMADD_SIZE_8BIT, BufferAddr, DatNum, 1000);

    return ReadStatus;
}

void RocAt24c02Init(void)
{
    uint16_t i = 0;

    printf("\r\n \r\n*********STM32CubeMX I2C AT24C02 Example*********\r\n");
    printf("\r\n I2C Write Buffer:\r\n");

    for(i = 0; i < 256; i++)
    {
        I2cBufferWrite[i] = i;  /* WriteBuffer Initialization */
        printf("%02X ", I2cBufferWrite[i]);
    }

    /* write data to AT24C02 */
    for(i = 0; i < 256; i = i + 8)
    {
        if (RocAt24c02WriteReg(ADDR_AT24C02_Write, i, &(I2cBufferWrite[i]), 8) == HAL_OK)
        {
            printf("\r\n Byte %02d to Byte %02d Write OK", i, i + 8);
            HAL_Delay(5);
        }
        else
        {
            printf("\r\n Byte %02d to Byte %02d Write Failed", i, i + 8);
        }
    }

    /* read data from EEPROM */
    printf("\r\n Reading from AT24C02:\r\n");

    RocAt24c02ReadReg(ADDR_AT24C02_Read, 0, I2cBufferRead, 256);

    for(i = 0; i < 256; i++)
    {
        printf("0x%02X  ", I2cBufferRead[i]);
    }

    if(memcmp(I2cBufferRead, I2cBufferWrite, 256) == 0) /* check data */
    {
        printf("\r\n AT24C02 Read Test OK\r\n");
    }
    else
    {
        printf("\r\n AT24C02 Read Test Failed\r\n");
    }
}


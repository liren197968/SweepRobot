#include "stm32f4xx_hal.h"

#include "RocSimulatedI2c.h"


/*********************************************************************************
 *  Description:
 *              Init IIC GPIO
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
void RocSimulatedI2cInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    ROC_SIMULATED_I2C_SCL(1);
    ROC_SIMULATED_I2C_SDA(1);
}

/*********************************************************************************
 *  Description:
 *              Send a IIC start signal
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
static void RocSimulatedI2cStart(void)
{
    ROC_SIMULATED_I2C_SDA_OUT();

    ROC_SIMULATED_I2C_SDA(1);
    ROC_SIMULATED_I2C_SCL(1);
    HAL_DelayUs(4);

    ROC_SIMULATED_I2C_SDA(0);
    HAL_DelayUs(4);

    ROC_SIMULATED_I2C_SCL(0);
}

/*********************************************************************************
 *  Description:
 *              Send a IIC stop signal
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
static void RocSimulatedI2cStop(void)
{
    ROC_SIMULATED_I2C_SDA_OUT();
    ROC_SIMULATED_I2C_SCL(0);
    ROC_SIMULATED_I2C_SDA(0);
    HAL_DelayUs(4);

    ROC_SIMULATED_I2C_SCL(1);
    ROC_SIMULATED_I2C_SDA(1);
    HAL_DelayUs(4);
}

/*********************************************************************************
 *  Description:
 *              Wait a IIC acknowledge signal
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              The wait status
 *
 *  Author:
 *              ROC LiRen(2019.04.15)
**********************************************************************************/
static uint8_t RocSimulatedI2cInitWaitAck(void)
{
    uint8_t ErrTime=0;

    ROC_SIMULATED_I2C_SDA_IN();

    ROC_SIMULATED_I2C_SDA(1);   HAL_DelayUs(1);
    ROC_SIMULATED_I2C_SCL(1);   HAL_DelayUs(1);

    while(ROC_SIMULATED_I2C_SDA_READ)
    {
        ErrTime++;

        if(ErrTime > 250)
        {
            RocSimulatedI2cStop();

            return 1;
        }
    }

    ROC_SIMULATED_I2C_SCL(0);

    return 0;
}

/*********************************************************************************
 *  Description:
 *              Send a IIC acknowledge signal
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
static void RocSimulatedI2cAck(void)
{
    ROC_SIMULATED_I2C_SCL(0);
    ROC_SIMULATED_I2C_SDA_OUT();
    ROC_SIMULATED_I2C_SDA(0);
    HAL_DelayUs(2);

    ROC_SIMULATED_I2C_SCL(1);
    HAL_DelayUs(2);

    ROC_SIMULATED_I2C_SCL(0);
}

/*********************************************************************************
 *  Description:
 *              Send a IIC no acknowledge signal
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
static void RocSimulatedI2cNAck(void)
{
    ROC_SIMULATED_I2C_SCL(0);
    ROC_SIMULATED_I2C_SDA_OUT();
    ROC_SIMULATED_I2C_SDA(1);
    HAL_DelayUs(2);

    ROC_SIMULATED_I2C_SCL(1);
    HAL_DelayUs(2);

    ROC_SIMULATED_I2C_SCL(0);
}

/*********************************************************************************
 *  Description:
 *              Send a IIC byte data
 *
 *  Parameter:
 *              Dat: the send data
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.15)
**********************************************************************************/
void RocSimulatedI2cSendByte(uint8_t Dat)
{
    uint8_t t;

    ROC_SIMULATED_I2C_SDA_OUT();
    ROC_SIMULATED_I2C_SCL(0);

    for(t = 0; t < 8; t++)
    {
        ROC_SIMULATED_I2C_SDA((Dat & 0x80) >> 7);

        Dat <<= 1;
        HAL_DelayUs(2);

        ROC_SIMULATED_I2C_SCL(1);
        HAL_DelayUs(2);

        ROC_SIMULATED_I2C_SCL(0);
        HAL_DelayUs(2);
    }
}

/*********************************************************************************
 *  Description:
 *              Read a IIC byte data with acknowledge signal
 *
 *  Parameter:
 *              Ack: the acknowledge data
 *
 *  Return:
 *              The read status
 *
 *  Author:
 *              ROC LiRen(2019.04.15)
**********************************************************************************/
uint8_t RocSimulatedI2cReadByte(uint8_t Ack)
{
    uint8_t i;
    uint8_t Receive = 0;

    ROC_SIMULATED_I2C_SDA_IN();

    for(i = 0; i < 8; i++)
    {
        ROC_SIMULATED_I2C_SCL(0);
        HAL_DelayUs(2);

        ROC_SIMULATED_I2C_SCL(1);

        Receive <<= 1;

        if(ROC_SIMULATED_I2C_SDA_READ)  Receive++;

        HAL_DelayUs(1);
    }
    if(!Ack)
    {
        RocSimulatedI2cNAck();
    }
    else
    {
        RocSimulatedI2cAck();
    }
    return Receive;
}


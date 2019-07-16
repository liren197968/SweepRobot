#ifndef __ROC_SIMULATED_I2C_H
#define __ROC_SIMULATED_I2C_H


#include <stdint.h>


#define ROC_SIMULATED_I2C_SDA_IN()      {GPIOB->MODER &= ~(3 << (11 * 2)); GPIOB->MODER |= 0 << 11 * 2;}
#define ROC_SIMULATED_I2C_SDA_OUT()     {GPIOB->MODER &= ~(3 << (11 * 2)); GPIOB->MODER |= 1 << 11 * 2;}

#define ROC_SIMULATED_I2C_SCL(N)        (N ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET))   //SCL
#define ROC_SIMULATED_I2C_SDA(N)        (N ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET))   //SDA
#define ROC_SIMULATED_I2C_SDA_READ      HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)


void RocSimulatedI2cInit(void);
void RocSimulatedI2cSendByte(uint8_t Dat);
uint8_t RocSimulatedI2cReadByte(uint8_t Ack);
void RocSimulatedI2cWriteOneByte(uint8_t Daddr,uint8_t Addr,uint8_t Data);
uint8_t RocSimulatedI2cReadOneByte(uint8_t Daddr,uint8_t Addr);

#endif


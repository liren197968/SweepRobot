#ifndef __ROC_IIC_H
#define __ROC_IIC_H


#include <stdint.h>


#define SDA_IN()        {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=0<<11*2;}   //PB11输入模式
#define SDA_OUT()       {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=1<<11*2;}   //PB11输出模式

#define IIC_SCL(n)      (n?HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET)) //SCL
#define IIC_SDA(n)      (n?HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET)) //SDA
#define READ_SDA        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)                //输入SDA 

void IIC_Init(void);                                                        //初始化IIC的IO口
void IIC_Start(void);                                                       //发送IIC开始信号
void IIC_Stop(void);                                                        //发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);                                                 //IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);                                        //IIC读取一个字节
uint8_t IIC_Wait_Ack(void);                                                      //IIC等待ACK信号
void IIC_Ack(void);                                                         //IIC发送ACK信号
void IIC_NAck(void);                                                        //IIC不发送ACK信号
void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);

#endif















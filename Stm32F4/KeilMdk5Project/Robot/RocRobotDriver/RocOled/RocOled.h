/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/05/02      1.0
********************************************************************************/
#ifndef __ROC_OLED_H
#define __ROC_OLED_H


#include "gpio.h"
#include "spi.h"


#define     ROC_OLED_X_SIZE             128
#define     ROC_OLED_Y_SIZE             64

#define     ROC_OLED_SPI_CHANNEL        (&hspi1)
#define     ROC_OLED_SUPPORT_NUM_LEN    6

#define     ROC_OLED_X_LEVEL_L          0x00
#define     ROC_OLED_X_LEVEL_H          0x10
#define     ROC_OLED_X_LEVEL            ((ROC_OLED_X_LEVEL_H & 0x0F) * 16 + ROC_OLED_X_LEVEL_L)
#define     ROC_OLED_BRIGHT             0xCF

#define     ROC_OLED_RST_SET()          HAL_GPIO_WritePin(ROC_OLED_RST_PORT, ROC_OLED_RST_PIN, GPIO_PIN_SET)
#define     ROC_OLED_RST_CLR()          HAL_GPIO_WritePin(ROC_OLED_RST_PORT, ROC_OLED_RST_PIN, GPIO_PIN_RESET)
#define     ROC_OLED_DC_SET()           HAL_GPIO_WritePin(ROC_OLED_DC_PORT, ROC_OLED_DC_PIN, GPIO_PIN_SET)
#define     ROC_OLED_DC_CLR()           HAL_GPIO_WritePin(ROC_OLED_DC_PORT, ROC_OLED_DC_PIN, GPIO_PIN_RESET)


void RocOledClearScreen(void);
void RocOledFillScreen(uint8_t BmpDat);
void RocOledDrawGbk8Char(uint8_t X, uint8_t Y, uint8_t Char);
void RocOledDrawGbk16Char(uint8_t X, uint8_t Y, uint8_t Char);
void RocOledDrawGbk8Str(uint8_t X, uint8_t Y, uint8_t *S);
void RocOledDrawGbk16Str(uint8_t X, uint8_t Y, uint8_t *S);
void RocOledDrawGbk8Num(uint8_t X, uint8_t Y, float N);
void RocOledDrawGbk16Num(uint8_t X, uint8_t Y, float N);
ROC_RESULT RocOledInit(void);

#endif


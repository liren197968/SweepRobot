/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/04/21      1.0
********************************************************************************/
#ifndef __ROC_TFT_LCD_H
#define __ROC_TFT_LCD_H


#include "stm32f4xx_hal.h"
#include "gpio.h"


#define ROC_TFT_LCD_WRITE_TIME_OUT      100 /* ms */


#define ROC_TFT_LCD_HORIZONTAL          ROC_ENABLE


#define ROC_TFT_LCD_SUPPORT_MAX_NUM     99999.99F
#define ROC_TFT_LCD_SUPPORT_MIN_NUM     -9999.99F
#define ROC_TFT_LCD_SUPPORT_NUM_LEN     8


#define ROC_TFT_LCD_X_SIZE              240
#define ROC_TFT_LCD_Y_SIZE              320
#define ROC_TFT_LCD_DATA_SIZE           16
#define ROC_TFT_LCD_PIXEL_SIZE          (ROC_TFT_LCD_X_SIZE * ROC_TFT_LCD_Y_SIZE)
#define ROC_TFT_LCD_ONE_PIXEL_BYTE      (ROC_TFT_LCD_DATA_SIZE / 8)
#define ROC_TFT_LCD_PIXEL_DATA_SIZE     (ROC_TFT_LCD_PIXEL_SIZE * ROC_TFT_LCD_ONE_PIXEL_BYTE)


#define ROC_TFT_LCD_BUFF_SIZE           (ROC_TFT_LCD_PIXEL_DATA_SIZE / 3)    /* allocate some RAM to enhance the LCD filling speed */
#define ROC_TFT_LCD_BUFF_STORAGE_PIXEL  (ROC_TFT_LCD_BUFF_SIZE / ROC_TFT_LCD_ONE_PIXEL_BYTE)


#define ROC_TFT_LCD_STR_PIXEL_SIZE      (ROC_TFT_LCD_SUPPORT_NUM_LEN * ROC_TFT_LCD_WIDTH_GBK_16 * ROC_TFT_LCD_HEIGHT_GBK_16)
#define ROC_TFT_LCD_STR_BUFF_SIZE       (ROC_TFT_LCD_STR_PIXEL_SIZE * ROC_TFT_LCD_ONE_PIXEL_BYTE)


#if ROC_TFT_LCD_HORIZONTAL
#define ROC_TFT_LCD_X_MAX_PIXEL         ROC_TFT_LCD_Y_SIZE
#define ROC_TFT_LCD_Y_MAX_PIXEL         ROC_TFT_LCD_X_SIZE
#else
#define ROC_TFT_LCD_X_MAX_PIXEL         ROC_TFT_LCD_X_SIZE
#define ROC_TFT_LCD_Y_MAX_PIXEL         ROC_TFT_LCD_Y_SIZE
#endif


#define ROC_TFT_LCD_COLOR_RED           0xf800
#define ROC_TFT_LCD_COLOR_GREEN         0x07e0
#define ROC_TFT_LCD_COLOR_BLUE          0x001f
#define ROC_TFT_LCD_COLOR_WHITE         0xffff
#define ROC_TFT_LCD_COLOR_BLACK         0x0000
#define ROC_TFT_LCD_COLOR_YELLOW        0xFFE0
#define ROC_TFT_LCD_COLOR_GRAY_0        0xEF7D
#define ROC_TFT_LCD_COLOR_GRAY_1        0x8410
#define ROC_TFT_LCD_COLOR_GRAY_2        0x4208


#define ROC_TFT_LCD_COLOR_DEFAULT_BAK   ROC_TFT_LCD_COLOR_BLUE
#define ROC_TFT_LCD_COLOR_DEFAULT_FOR   ROC_TFT_LCD_COLOR_YELLOW


#define ROC_TFT_LCD_RS_SET()            HAL_GPIO_WritePin(ROC_TFTLCD_DC_PORT, ROC_TFTLCD_DC_PIN, GPIO_PIN_SET);
#define ROC_TFT_LCD_RST_SET()           HAL_GPIO_WritePin(ROC_TFTLCD_RST_PORT, ROC_TFTLCD_RST_PIN, GPIO_PIN_SET);

#define ROC_TFT_LCD_RS_CLR()            HAL_GPIO_WritePin(ROC_TFTLCD_DC_PORT, ROC_TFTLCD_DC_PIN, GPIO_PIN_RESET);
#define ROC_TFT_LCD_RST_CLR()           HAL_GPIO_WritePin(ROC_TFTLCD_RST_PORT, ROC_TFTLCD_RST_PIN, GPIO_PIN_RESET);


typedef enum _ROC_TFT_LCD_SPI_DAT_FORMAT_e
{
    ROC_TFT_LCD_SPI_DAT_8_BIT = 0,
    ROC_TFT_LCD_SPI_DAT_16_BIT,

}ROC_TFT_LCD_SPI_DAT_FORMAT_e;

typedef enum _ROC_TFT_LCD_SPI_DAT_SPEED_e
{
    ROC_TFT_LCD_SPI_DAT_LOW_SPEED = 0,
    ROC_TFT_LCD_SPI_DAT_HIGH_SPEED,

}ROC_TFT_LCD_SPI_DAT_SPEED_e;


ROC_RESULT RocTftLcdInit(void);
void RocTftLcdAllClear(uint16_t BakColor);
void RocTftLcdShowErrorMsg(uint8_t *pStr);
void RocTftLcdDrawPoint(uint16_t X, uint16_t Y, uint16_t Color);
ROC_RESULT RocDoubleDatToStringDat(float FloatData, uint8_t *pString);
void RocTftLcdDrawTubeNum(uint16_t X, uint16_t Y, uint16_t Fc, uint16_t Bc, uint16_t Num);
void RocTftLcdDrawLine(uint16_t XStart, uint16_t YStart,uint16_t XEnd, uint16_t YEnd,uint16_t Color);
void RocTftLcdDrawCircle(uint16_t X, uint16_t Y, uint16_t R, uint16_t Color);
void RocTftLcdDrawRectangle(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, uint16_t Color);
void RocTftLcdDrawRectangle2(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, uint8_t Mode);
void RocTftLcdDrawGbk16Str(uint16_t X, uint16_t Y, uint16_t Fc, uint16_t Bc, uint8_t *pStr);
void RocTftLcdDrawGbk24Str(uint16_t X, uint16_t Y, uint16_t Fc, uint16_t Bc, uint8_t *pStr);
void RocTftLcdDrawGbk16Num(uint16_t X, uint16_t Y, uint16_t Fc, uint16_t Bc, float Num);
void RocTftLcdDrawGbk24Num(uint16_t X, uint16_t Y, uint16_t Fc, uint16_t Bc, float Num);


#endif


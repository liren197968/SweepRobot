/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/04/21      1.0
********************************************************************************/
#ifndef __ROC_TFT_LCD_H
#define __ROC_TFT_LCD_H


#include "gpio.h"


#define ROC_TFT_LCD_WRITE_TIME_OUT      10  /* ms */


#define ROC_TFT_LCD_HORIZONTAL          ROC_ENABLE


#define ROC_TFT_LCD_X_SIZE	            240
#define ROC_TFT_LCD_Y_SIZE	            320


#if ROC_TFT_LCD_HORIZONTAL
#define ROC_TFT_LCD_X_MAX_PIXEL	        ROC_TFT_LCD_Y_SIZE
#define ROC_TFT_LCD_Y_MAX_PIXEL	        ROC_TFT_LCD_X_SIZE
#else
#define ROC_TFT_LCD_X_MAX_PIXEL	        ROC_TFT_LCD_X_SIZE
#define ROC_TFT_LCD_Y_MAX_PIXEL	        ROC_TFT_LCD_Y_SIZE
#endif


#define ROC_TFT_LCD_COLOR_RED           0xf800
#define ROC_TFT_LCD_COLOR_GREEN	        0x07e0
#define ROC_TFT_LCD_COLOR_BLUE          0x001f
#define ROC_TFT_LCD_COLOR_WHITE         0xffff
#define ROC_TFT_LCD_COLOR_BLACK         0x0000
#define ROC_TFT_LCD_COLOR_YELLOW        0xFFE0
#define ROC_TFT_LCD_COLOR_GRAY_0        0xEF7D
#define ROC_TFT_LCD_COLOR_GRAY_1        0x8410
#define ROC_TFT_LCD_COLOR_GRAY_2        0x4208


#define ROC_TFT_LCD_COLOR_DEFAULT_BAK   ROC_TFT_LCD_COLOR_BLUE
#define ROC_TFT_LCD_COLOR_DEFAULT_FOR   ROC_TFT_LCD_COLOR_YELLOW


#define	ROC_TFT_LCD_RS_SET()            HAL_GPIO_WritePin(ROC_TFTLCD_DC_PORT, ROC_TFTLCD_DC_PIN, GPIO_PIN_SET);
#define	ROC_TFT_LCD_RST_SET()           HAL_GPIO_WritePin(ROC_TFTLCD_RST_PORT, ROC_TFTLCD_RST_PIN, GPIO_PIN_SET);

#define	ROC_TFT_LCD_RS_CLR()            HAL_GPIO_WritePin(ROC_TFTLCD_DC_PORT, ROC_TFTLCD_DC_PIN, GPIO_PIN_RESET);
#define	ROC_TFT_LCD_RST_CLR()           HAL_GPIO_WritePin(ROC_TFTLCD_RST_PORT, ROC_TFTLCD_RST_PIN, GPIO_PIN_RESET);



ROC_RESULT RocTftLcdInit(void);
void RocTftLcdAllClear(uint16_t BakColor);
void RocSpiSpeedSet(SPI_TypeDef* Spix,uint8_t SpeedSet);
void RocTftLcdDrawPoint(uint16_t x, uint16_t y, uint16_t Data);
void RocDrawFontDigitalTubeNum(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint16_t num);



#endif


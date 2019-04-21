/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/04/21      1.0
********************************************************************************/
#ifndef __ROC_TFT_LCD_H
#define __ROC_TFT_LCD_H


#include "gpio.h"


#define ROC_TFT_LCD_WRITE_TIME_OUT  10  /* ms */


#define USE_HORIZONTAL		ROC_ENABLE

#define LCD_X_SIZE	        240
#define LCD_Y_SIZE	        320


#if USE_HORIZONTAL
#define X_MAX_PIXEL	        LCD_Y_SIZE
#define Y_MAX_PIXEL	        LCD_X_SIZE
#else
#define X_MAX_PIXEL	        LCD_X_SIZE
#define Y_MAX_PIXEL	        LCD_Y_SIZE
#endif


#define RED  	0xf800
#define GREEN	0x07e0
#define BLUE 	0x001f
#define WHITE	0xffff
#define BLACK	0x0000
#define YELLOW  0xFFE0
#define GRAY0   0xEF7D
#define GRAY1   0x8410
#define GRAY2   0x4208


#define	LCD_RS_SET  	HAL_GPIO_WritePin(ROC_TFTLCD_DC_PORT, ROC_TFTLCD_DC_PIN, GPIO_PIN_SET);
#define	LCD_RST_SET  	HAL_GPIO_WritePin(ROC_TFTLCD_RST_PORT, ROC_TFTLCD_RST_PIN, GPIO_PIN_SET);

#define	LCD_RS_CLR  	HAL_GPIO_WritePin(ROC_TFTLCD_DC_PORT, ROC_TFTLCD_DC_PIN, GPIO_PIN_RESET);
#define	LCD_RST_CLR  	HAL_GPIO_WritePin(ROC_TFTLCD_RST_PORT, ROC_TFTLCD_RST_PIN, GPIO_PIN_RESET);


void Gui_DrawFont_Num32(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint16_t num);
void RocLcdDrawPoint(uint16_t x, uint16_t y, uint16_t Data);
ROC_RESULT RocTftLcdInit(void);

#endif


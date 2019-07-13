#ifndef _ROC_FONT_H_
#define _ROC_FONT_H_


#define ROC_USE_ONCHIP_FLASH_FONT   1


#define ROC_TFT_LCD_WIDTH_GBK_16    8
#define ROC_TFT_LCD_WIDTH_GBK_24    12
#define ROC_TFT_LCD_HEIGHT_GBK_16   16
#define ROC_TFT_LCD_HEIGHT_GBK_24   24


#define ROC_TFT_LCD_HZ16_NUM        50
#define ROC_TFT_LCD_HZ24_NUM        20


typedef struct _ROC_TFT_LCD_GB162_s
{
    uint8_t Index[2];
    int8_t  Msk[32];

}ROC_TFT_LCD_GB162_s;

typedef struct _ROC_TFT_LCD_GB242_s
{
    uint8_t Index[2];
    int8_t  Msk[72];

}ROC_TFT_LCD_GB242_s;


extern const uint8_t g_Ascii16[];
extern const uint8_t g_Sz32[];
extern const ROC_TFT_LCD_GB162_s g_Hz16[];
extern const ROC_TFT_LCD_GB242_s g_Hz24[];
extern const uint8_t g_OledFont6x8[][6];
extern const uint8_t g_OledFont8x16[];

#endif


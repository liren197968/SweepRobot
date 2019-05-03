/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/04/21      1.0
********************************************************************************/
#include <string.h>

#include "gpio.h"
#include "spi.h"

#include "RocFont.h"
#include "RocPicture.h"

#include "RocLog.h"
#include "RocTftLcd.h"


uint8_t g_DisplayNum[10]={0,1,2,3,4,5,6,7,8,9};

/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle.
  * @note   This example shows a simple way to report end of DMA TxRx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(SPI1 == hspi->Instance)
    {
        ROC_LOGI("SPI data send is in success");
    }
}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
 void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if(SPI1 == hspi->Instance)
    {
        ROC_LOGI("SPI data send is in error");
    }
}

/*********************************************************************************
 *  Description:
 *              Set the TFT LCD SPI data transmission format
 *
 *  Parameter:
 *              DatFat: the control data format
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
static void RocTftLcdSpiDataFormatSet(ROC_TFT_LCD_SPI_DAT_FORMAT_e DatFot)
{
    hspi1.Instance->CR1 &= ~SPI_DATASIZE_16BIT;

    if(ROC_TFT_LCD_SPI_DAT_8_BIT == DatFot)
    {
        hspi1.Instance->CR1 |= SPI_DATASIZE_8BIT;
    }
    else if(ROC_TFT_LCD_SPI_DAT_16_BIT == DatFot)
    {
        hspi1.Instance->CR1 |= SPI_DATASIZE_16BIT;
    }
}

/*********************************************************************************
 *  Description:
 *              Set the TFT LCD SPI speed
 *
 *  Parameter:
 *              SpeedSet: the control speed
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
static void RocTftLcdSpiSpeedSet(ROC_TFT_LCD_SPI_DAT_SPEED_e Speed)
{
    hspi1.Instance->CR1 &= 0XFFC7;

    if(ROC_TFT_LCD_SPI_DAT_HIGH_SPEED == Speed)
    {
        hspi1.Instance->CR1 |= SPI_BAUDRATEPRESCALER_2;
    }
    else if(ROC_TFT_LCD_SPI_DAT_LOW_SPEED == Speed)
    {
        hspi1.Instance->CR1 |= SPI_BAUDRATEPRESCALER_32;
    }

    hspi1.Instance->CR1 |= 1 << 6;
}

/*********************************************************************************
 *  Description:
 *              Write a byte data with SPI communication
 *
 *  Parameter:
 *              Data: the data written to SPI
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
static void  RocSpiWriteData(uint8_t Data)
{
    HAL_StatusTypeDef WriteStatus;

    while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

    WriteStatus = HAL_SPI_Transmit(&hspi1, &Data, 1, ROC_TFT_LCD_WRITE_TIME_OUT);
    if(HAL_OK != WriteStatus)
    {
        ROC_LOGE("SPI write data is in error(%d)", WriteStatus);
    }
}

/*********************************************************************************
 *  Description:
 *              Select register with SPI communication
 *
 *  Parameter:
 *              Reg: the written LCD register
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
static void RocTftLcdWriteReg(uint8_t Reg)
{
   ROC_TFT_LCD_RS_CLR();

   RocSpiWriteData(Reg);
}

/*********************************************************************************
 *  Description:
 *              Write 8 bit data to LCD register
 *
 *  Parameter:
 *              Data: the data written to LCD register
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
static void RocTftLcdWriteDat(uint8_t Data)
{
    ROC_TFT_LCD_RS_SET();

    RocTftLcdSpiDataFormatSet(ROC_TFT_LCD_SPI_DAT_8_BIT);
    RocSpiWriteData(Data);
}

/*********************************************************************************
 *  Description:
 *              Write 16 bit data to LCD register
 *
 *  Parameter:
 *              Data: the data written to LCD register
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
static void RocTftLcdWrite16Dat(uint16_t Data)
{
    RocTftLcdWriteDat(Data >> 8);
    RocTftLcdWriteDat(Data);
}

/*********************************************************************************
 *  Description:
 *              Write command to LCD register
 *
 *  Parameter:
 *              Cmd: the command written to LCD register
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
//static void RocTftLcdRegWriteCmd(uint8_t Reg,uint16_t Data)
//{
//    RocTftLcdWriteReg(Reg);
//    RocTftLcdWrite16Dat(Data);
//}

/*********************************************************************************
 *  Description:
 *              Reset TFT LCD
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
static void RocTftLcdReset(void)
{
    ROC_TFT_LCD_RST_CLR();
    HAL_Delay(100);
    ROC_TFT_LCD_RST_SET();
    HAL_Delay(50);
}

/*********************************************************************************
 *  Description:
 *              Init TFT LCD register
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
static void RocTftLcdRegInit(void)
{
    RocTftLcdReset();

    RocTftLcdWriteReg(0x11);
    RocTftLcdWriteDat(0x00);

    RocTftLcdWriteReg(0xCF);
    RocTftLcdWriteDat(0X00);
    RocTftLcdWriteDat(0XC1);
    RocTftLcdWriteDat(0X30);

    RocTftLcdWriteReg(0xED);
    RocTftLcdWriteDat(0X64);
    RocTftLcdWriteDat(0X03);
    RocTftLcdWriteDat(0X12);
    RocTftLcdWriteDat(0X81);

    RocTftLcdWriteReg(0xE8);
    RocTftLcdWriteDat(0X85);
    RocTftLcdWriteDat(0X11);
    RocTftLcdWriteDat(0X78);

    RocTftLcdWriteReg(0xF6);
    RocTftLcdWriteDat(0X01);
    RocTftLcdWriteDat(0X30);
    RocTftLcdWriteDat(0X00);

    RocTftLcdWriteReg(0xCB);
    RocTftLcdWriteDat(0X39);
    RocTftLcdWriteDat(0X2C);
    RocTftLcdWriteDat(0X00);
    RocTftLcdWriteDat(0X34);
    RocTftLcdWriteDat(0X05);

    RocTftLcdWriteReg(0xF7);
    RocTftLcdWriteDat(0X20);

    RocTftLcdWriteReg(0xEA);
    RocTftLcdWriteDat(0X00);
    RocTftLcdWriteDat(0X00);

    RocTftLcdWriteReg(0xC0);
    RocTftLcdWriteDat(0X20);

    RocTftLcdWriteReg(0xC1);
    RocTftLcdWriteDat(0X11);

    RocTftLcdWriteReg(0xC5);
    RocTftLcdWriteDat(0X31);
    RocTftLcdWriteDat(0X3C);

    RocTftLcdWriteReg(0xC7);
    RocTftLcdWriteDat(0XA9);

    RocTftLcdWriteReg(0x3A);
    RocTftLcdWriteDat(0X55);

    RocTftLcdWriteReg(0x36);

#if ROC_TFT_LCD_HORIZONTAL
    RocTftLcdWriteDat(0xE8);
#else
    RocTftLcdWriteDat(0x48);
#endif

    RocTftLcdWriteReg(0xB1);
    RocTftLcdWriteDat(0X00);
    RocTftLcdWriteDat(0X18);

    RocTftLcdWriteReg(0xB4);
    RocTftLcdWriteDat(0X00);
    RocTftLcdWriteDat(0X00);

    RocTftLcdWriteReg(0xF2);
    RocTftLcdWriteDat(0X00);

    RocTftLcdWriteReg(0x26);
    RocTftLcdWriteDat(0X01);

    RocTftLcdWriteReg(0xE0);
    RocTftLcdWriteDat(0X0F);
    RocTftLcdWriteDat(0X17);
    RocTftLcdWriteDat(0X14);
    RocTftLcdWriteDat(0X09);
    RocTftLcdWriteDat(0X0C);
    RocTftLcdWriteDat(0X06);
    RocTftLcdWriteDat(0X43);
    RocTftLcdWriteDat(0X75);
    RocTftLcdWriteDat(0X36);
    RocTftLcdWriteDat(0X08);
    RocTftLcdWriteDat(0X13);
    RocTftLcdWriteDat(0X05);
    RocTftLcdWriteDat(0X10);
    RocTftLcdWriteDat(0X0B);
    RocTftLcdWriteDat(0X08);


    RocTftLcdWriteReg(0xE1);
    RocTftLcdWriteDat(0X00);
    RocTftLcdWriteDat(0X1F);
    RocTftLcdWriteDat(0X23);
    RocTftLcdWriteDat(0X03);
    RocTftLcdWriteDat(0X0E);
    RocTftLcdWriteDat(0X04);
    RocTftLcdWriteDat(0X39);
    RocTftLcdWriteDat(0X25);
    RocTftLcdWriteDat(0X4D);
    RocTftLcdWriteDat(0X06);
    RocTftLcdWriteDat(0X0D);
    RocTftLcdWriteDat(0X0B);
    RocTftLcdWriteDat(0X33);
    RocTftLcdWriteDat(0X37);
    RocTftLcdWriteDat(0X0F);

    RocTftLcdWriteReg(0x29);	
}

/*********************************************************************************
 *  Description:
 *              Set TFT LCD X and Y position
 *
 *  Parameter:
 *              Xpos: X position
 *              Ypos: Y position
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
static void RocTftLcdSetXY(uint16_t Xpos, uint16_t Ypos)
{
    RocTftLcdWriteReg(0x2A);
    RocTftLcdWrite16Dat(Xpos);
    RocTftLcdWriteReg(0x2B);
    RocTftLcdWrite16Dat(Ypos);
    RocTftLcdWriteReg(0x2c);
}

/*********************************************************************************
 *  Description:
 *              Set TFT LCD display region
 *
 *  Parameter:
 *              XStart: X start position
 *              YStart: Y start position
 *              XEnd:   X end position
 *              YEnd:   Y end position
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
static void RocTftLcdSetRegion(uint16_t XStart, uint16_t YStart, uint16_t XEnd, uint16_t YEnd)
{
    RocTftLcdWriteReg(0x2A);
    RocTftLcdWrite16Dat(XStart);
    RocTftLcdWrite16Dat(XEnd);
    RocTftLcdWriteReg(0x2B);
    RocTftLcdWrite16Dat(YStart);
    RocTftLcdWrite16Dat(YEnd);
    RocTftLcdWriteReg(0x2c);
}

/*********************************************************************************
 *  Description:
 *              Draw a point at (X, Y) position on TFT LCD display
 *
 *  Parameter:
 *              X:      X position
 *              Y:      Y position
 *              Color:  the colour of point
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
void RocTftLcdDrawPoint(uint16_t X, uint16_t Y, uint16_t Color)
{
    RocTftLcdSetXY(X, Y);
    RocTftLcdWrite16Dat(Color);
}

/*********************************************************************************
 *  Description:
 *              Clear all TFT LCD region with background colour
 *
 *  Parameter:
 *              BakColor: the background colour of TFT LCD
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
void RocTftLcdAllClear(uint16_t BakColor)
{
    unsigned int i;

    RocTftLcdSetRegion(0, 0, ROC_TFT_LCD_X_MAX_PIXEL - 1, ROC_TFT_LCD_Y_MAX_PIXEL - 1);

    ROC_TFT_LCD_RS_SET();

    for(i = 0; i < ROC_TFT_LCD_X_MAX_PIXEL * ROC_TFT_LCD_Y_MAX_PIXEL; i++)
    {
        //RocTftLcdWrite16Dat(BakColor);
        RocSpiWriteData(BakColor>>8);
        RocSpiWriteData(BakColor);
    }
}

/*********************************************************************************
 *  Description:
 *              Clear the TFT LCD selected region with background colour
 *
 *  Parameter:
 *              BakColor: the background colour of TFT LCD
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
static void RocTftLcdRegionClear(uint16_t XStart, uint16_t YStart, uint16_t XEnd, uint16_t YEnd, uint16_t BakColor)
{
    unsigned int i;

    RocTftLcdSetRegion(XStart, YStart, XEnd, YEnd);

    ROC_TFT_LCD_RS_SET();

    for(i = 0; i < (XEnd - XStart) * (YEnd - YStart); i++)
    {
        //RocTftLcdWrite16Dat(BakColor);
        RocSpiWriteData(BakColor>>8);
        RocSpiWriteData(BakColor);
    }
}

/*********************************************************************************
 *  Description:
 *              Convert GBR format reading from ILI93xx IC to LCD RGB format
 *
 *  Parameter:
 *              GbrDat: the GBR colour data
 *
 *  Return:
 *              The RGB colour data
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
static uint16_t RocTftLcdBgr2Rgb(uint16_t GbrDat)
{
    uint16_t RDat = 0;
    uint16_t GDat = 0;
    uint16_t BDat = 0;
    uint16_t RgbDat = 0;

    BDat = (GbrDat >> 0) & 0x1f;
    GDat = (GbrDat >> 5) & 0x3f;
    RDat = (GbrDat >> 11) & 0x1f;
    RgbDat = (BDat << 11) + (GDat << 5) + (RDat << 0);

    return(RgbDat);
}

/*********************************************************************************
 *  Description:
 *              Draw a circle on TFT LCD
 *
 *  Parameter:
 *              X: the X position
 *              Y: the Y position
 *              R: the radius of the circle
 *              Color: the color of the circle
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
void RocTftLcdDrawCircle(uint16_t X, uint16_t Y, uint16_t R, uint16_t Color)
{
    /* Bresenham algorithm */
    uint16_t A;
    uint16_t B;
    int32_t C;

    A = 0;
    B = R;
    C = 3 - 2 * R;

    while(A < B)
    {
        RocTftLcdDrawPoint(X + A, Y + B, Color);     // 7
        RocTftLcdDrawPoint(X - A, Y + B, Color);     // 6
        RocTftLcdDrawPoint(X + A, Y - B, Color);     // 2
        RocTftLcdDrawPoint(X - A, Y - B, Color);     // 3
        RocTftLcdDrawPoint(X + B, Y + A, Color);     // 8
        RocTftLcdDrawPoint(X - B, Y + A, Color);     // 5
        RocTftLcdDrawPoint(X + B, Y - A, Color);     // 1
        RocTftLcdDrawPoint(X - B, Y - A, Color);     // 4

        if( C < 0)
        {
            C = C + 4 * A + 6;
        }
        else
        {
            C= C + 4 * (A - B) + 10;
            B -= 1;
        }

        A += 1;
    } 

    if(A == B)
    {
        RocTftLcdDrawPoint(X + A, Y + B, Color);
        RocTftLcdDrawPoint(X + A, Y + B, Color);
        RocTftLcdDrawPoint(X + A, Y - B, Color);
        RocTftLcdDrawPoint(X - A, Y - B, Color);
        RocTftLcdDrawPoint(X + B, Y + A, Color);
        RocTftLcdDrawPoint(X - B, Y + A, Color);
        RocTftLcdDrawPoint(X + B, Y - A, Color);
        RocTftLcdDrawPoint(X - B, Y - A, Color);
    }
} 

/*********************************************************************************
 *  Description:
 *              Draw a line on TFT LCD
 *
 *  Parameter:
 *              XStart: X start position
 *              YStart: Y start position
 *              XEnd:   X end position
 *              YEnd:   Y end position
 *              Color:  the color of the line
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
void RocTftLcdDrawLine(uint16_t XStart, uint16_t YStart,uint16_t XEnd, uint16_t YEnd,uint16_t Color)
{
    /* Bresenham algorithm */
    int32_t Dx;             // difference in x's
    int32_t Dy;             // difference in y's
    int32_t Dx2;            // Dx, Dy * 2
    int32_t Dy2;
    int32_t XInc;           // amount in pixel space to move during drawing
    int32_t YInc;           // amount in pixel space to move during drawing
    int32_t Error;          // the discriminant i.e. error i.e. decision variable
    int32_t Index;          // used for looping	

    RocTftLcdSetXY(XStart,YStart);

    Dx = XEnd - XStart;
    Dy = YEnd - YStart;

    if(Dx >= 0)
    {
        XInc = 1;
    }
    else
    {
        XInc = -1;
        Dx = -Dx;
    }

    if (Dy>=0)
    {
        YInc = 1;
    }
    else
    {
        YInc = -1;
        Dy = -Dy;
    } 

    Dx2 = Dx << 1;
    Dy2 = Dy << 1;

    if(Dx > Dy)
    {
        // initialize error term
        Error = Dy2 - Dx;

        // draw the line
        for(Index = 0; Index <= Dx; Index++)
        {
            RocTftLcdDrawPoint(XStart,YStart,Color);

            // test if error has overflowed
            if(Error >= 0)
            {
                Error -= Dx2;

                // move to next line
                YStart += YInc;
            }

            // adjust the error term
            Error += Dy2;

            // move to the next pixel
            XStart += XInc;
        }
    }
    else
    {
        // initialize error term
        Error = Dx2 - Dy;

        // draw the line
        for(Index = 0; Index <= Dy; Index++)
        {
            // set the pixel
            RocTftLcdDrawPoint(XStart,YStart,Color);

            // test if error overflowed
            if(Error >= 0)
            {
                Error-=Dy2;

                // move to next line
                XStart += XInc;
            }

            // adjust the error term
            Error += Dx2;

            // move to the next pixel
            YStart += YInc;
        }
    }
}

/*********************************************************************************
 *  Description:
 *              Draw a rectangle on TFT LCD
 *
 *  Parameter:
 *              X:      X position
 *              Y:      Y position
 *              W:      the rectangle width
 *              H:      the rectangle height
 *              Color:  the color of the rectangle
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
void RocTftLcdDrawRectangle(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, uint16_t Color)
{
    RocTftLcdDrawLine(X, Y, X + W, Y, 0xEF7D);
    RocTftLcdDrawLine(X + W-1, Y + 1, X + W - 1,Y + 1 + H, 0x2965);
    RocTftLcdDrawLine(X, Y + H, X + W, Y + H, 0x2965);
    RocTftLcdDrawLine(X, Y,X, Y + H, 0xEF7D);
    RocTftLcdDrawLine(X + 1, Y + 1, X + 1 + W - 2, Y + 1 + H - 2, Color);
}

/*********************************************************************************
 *  Description:
 *              Draw a rectangle on TFT LCD
 *
 *  Parameter:
 *              X:      X position
 *              Y:      Y position
 *              W:      the rectangle width
 *              H:      the rectangle height
 *              Color:  the color of the rectangle
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
void RocTftLcdDrawRectangle2(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, uint8_t Mode)
{
    if(Mode == 0)
    {
        RocTftLcdDrawLine(X, Y, X + W, Y, 0xEF7D);
        RocTftLcdDrawLine(X + W - 1, Y + 1, X + W - 1, Y + 1 + H, 0x2965);
        RocTftLcdDrawLine(X, Y + H, X + W, Y + H, 0x2965);
        RocTftLcdDrawLine(X, Y, X, Y + H, 0xEF7D);
    }
    if(Mode == 1)
    {
        RocTftLcdDrawLine(X, Y, X + W, Y, 0x2965);
        RocTftLcdDrawLine(X + W - 1, Y + 1, X + W - 1, Y + 1 + H, 0xEF7D);
        RocTftLcdDrawLine(X, Y + H, X + W, Y + H, 0xEF7D);
        RocTftLcdDrawLine(X, Y, X, Y + H, 0x2965);
    }
    if(Mode == 2)
    {
        RocTftLcdDrawLine(X, Y, X + W, Y, 0xFFFF);
        RocTftLcdDrawLine(X + W - 1, Y + 1, X + W - 1, Y + 1 + H, 0xFFFF);
        RocTftLcdDrawLine(X, Y + H, X + W, Y + H, 0xFFFF);
        RocTftLcdDrawLine(X, Y, X, Y + H, 0xFFFF);
    }
}

/*********************************************************************************
 *  Description:
 *              Draw a button on TFT LCD
 *
 *  Parameter:
 *              XStart: X start position
 *              YStart: Y start position
 *              XEnd:   X end position
 *              YEnd:   Y end position
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
static void RocTftLcdDisplayButtonDown(uint16_t XStart, uint16_t YStart, uint16_t XEnd, uint16_t YEnd)
{
    RocTftLcdDrawLine(XStart, YStart, XEnd, YStart, ROC_TFT_LCD_COLOR_GRAY_2);              //H
    RocTftLcdDrawLine(XStart + 1,YStart + 1, XEnd, YStart + 1, ROC_TFT_LCD_COLOR_GRAY_1);   //H
    RocTftLcdDrawLine(XStart, YStart, XStart, YEnd, ROC_TFT_LCD_COLOR_GRAY_2);              //V
    RocTftLcdDrawLine(XStart + 1,YStart + 1, XStart + 1, YEnd, ROC_TFT_LCD_COLOR_GRAY_1);   //V
    RocTftLcdDrawLine(XStart, YEnd, XEnd, YEnd, ROC_TFT_LCD_COLOR_WHITE);                   //H
    RocTftLcdDrawLine(XEnd,  YStart, XEnd, YEnd, ROC_TFT_LCD_COLOR_WHITE);                  //V
}

/*********************************************************************************
 *  Description:
 *              Draw a button on TFT LCD
 *
 *  Parameter:
 *              XStart: X start position
 *              YStart: Y start position
 *              XEnd:   X end position
 *              YEnd:   Y end position
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
static void RocTftLcdDisplayButtonUp(uint16_t XStart,uint16_t YStart,uint16_t XEnd,uint16_t YEnd)
{
    RocTftLcdDrawLine(XStart, YStart, XEnd, YStart, ROC_TFT_LCD_COLOR_WHITE);           //H
    RocTftLcdDrawLine(XStart, YStart, XStart, YEnd, ROC_TFT_LCD_COLOR_WHITE);           //V

    RocTftLcdDrawLine(XStart + 1, YEnd - 1, XEnd,YEnd - 1, ROC_TFT_LCD_COLOR_GRAY_1);   //H
    RocTftLcdDrawLine(XStart, YEnd, XEnd, YEnd, ROC_TFT_LCD_COLOR_GRAY_2);              //H
    RocTftLcdDrawLine(XEnd - 1, YStart + 1, XEnd - 1, YEnd, ROC_TFT_LCD_COLOR_GRAY_1);  //V
    RocTftLcdDrawLine(XEnd, YStart, XEnd, YEnd, ROC_TFT_LCD_COLOR_GRAY_2);              //V
}

/*********************************************************************************
 *  Description:
 *              Show a string on TFT LCD
 *
 *  Parameter:
 *              X:    X position
 *              Y:    Y position
 *              Fc:   font colour
 *              Bc:   background colour
 *              pStr: the pointer to string
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
void RocTftLcdDrawGbk16Str(uint16_t X, uint16_t Y, uint16_t Fc, uint16_t Bc, uint8_t *pStr)
{
    uint16_t i = 0;
    uint16_t j = 0;
    uint16_t k = 0;
    uint16_t x0 = 0;

    x0 = X;

    while(*pStr)
    {
        if((*pStr) < 128)
        {
            k = *pStr;

            if(k == 13)
            {
                X = x0;
                Y += 16;
            }
            else
            {
                if(k > 32)
                {
                    k -= 32;
                }
                else
                {
                    k=0;
                }

                for(i = 0; i < 16; i++)
                {
                    for(j = 0; j < 8; j++)
                    {
                        if(g_Ascii16[k * 16 + i] & (0x80 >> j))
                        {
                            RocTftLcdDrawPoint(X + j, Y + i, Fc);
                        }
                        else
                        {
                            if(Fc != Bc)
                            {
                                RocTftLcdDrawPoint(X + j, Y + i, Bc);
                            }
                        }
                    }
                }

                X += 8;
            }

            pStr++;
        }
        else
        {
            for(k = 0; k < ROC_TFT_LCD_HZ16_NUM; k++)
            {
                if((g_Hz16[k].Index[0] == *(pStr)) && (g_Hz16[k].Index[1] == *(pStr + 1)))
                {
                    for(i = 0; i < 16; i++)
                    {
                        for(j = 0; j < 8; j++)
                        {
                            if(g_Hz16[k].Msk[i * 2] & (0x80 >> j))
                            {
                                RocTftLcdDrawPoint(X + j, Y + i, Fc);
                            }
                            else
                            {
                                if(Fc != Bc)
                                {
                                    RocTftLcdDrawPoint(X + j, Y + i, Bc);
                                }
                            }
                        }

                        for(j = 0; j < 8; j++)
                        {
                            if(g_Hz16[k].Msk[i * 2 + 1] & (0x80 >> j))
                            {
                                RocTftLcdDrawPoint(X + j + 8, Y + i, Fc);
                            }
                            else
                            {
                                if(Fc != Bc)
                                {
                                    RocTftLcdDrawPoint(X + j + 8, Y + i, Bc);
                                }
                            }
                        }
                    }
                }

                pStr += 2;
                X += 16;
            }
        }
    }
}

/*********************************************************************************
 *  Description:
 *              Show a string on TFT LCD
 *
 *  Parameter:
 *              X:    X position
 *              Y:    Y position
 *              Fc:   font colour
 *              Bc:   background colour
 *              pStr: the pointer to string
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
void RocTftLcdDrawGbk24Str(uint16_t X, uint16_t Y, uint16_t Fc, uint16_t Bc, uint8_t *pStr)
{
    uint16_t i = 0;
    uint16_t j = 0;
    uint16_t k = 0;

    while(*pStr)
    {
        if(*pStr < 0x80)
        {
            k = *pStr;

            if (k > 32)
            {
                k -= 32;
            }
            else
            {
                k = 0;
            }

            for(i = 0; i < 16; i++)
            {
                for(j = 0; j < 8; j++)
                {
                    if(g_Ascii16[k * 16 + i] & (0x80 >> j))
                    {
                        RocTftLcdDrawPoint(X + j, Y + i, Fc);
                    }
                    else
                    {
                        if(Fc != Bc)
                        {
                            RocTftLcdDrawPoint(X + j, Y + i, Bc);
                        }
                    }
                }
            }

            pStr++;
            X += 8;
        }
        else
        {
            for(k = 0; k < ROC_TFT_LCD_HZ24_NUM; k++)
            {
                if((g_Hz24[k].Index[0] == *(pStr)) && (g_Hz24[k].Index[1] == *(pStr + 1)))
                {
                    for(i = 0; i < 24; i++)
                    {
                        for(j = 0; j < 8; j++)
                        {
                            if(g_Hz24[k].Msk[i*3] & (0x80 >> j))
                            {
                                RocTftLcdDrawPoint(X + j, Y + i, Fc);
                            }
                            else
                            {
                                if(Fc!= Bc)
                                {
                                    RocTftLcdDrawPoint(X + j, Y + i, Bc);
                                }
                            }
                        }

                        for(j = 0; j < 8; j++)
                        {
                            if(g_Hz24[k].Msk[i * 3 + 1] & (0x80 >> j))
                            {
                                RocTftLcdDrawPoint(X + j + 8, Y + i, Fc);
                            }
                            else
                            {
                                if(Fc != Bc)
                                {
                                    RocTftLcdDrawPoint(X + j + 8, Y + i, Bc);
                                }
                            }
                        }

                        for(j=0;j<8;j++)
                        {
                            if(g_Hz24[k].Msk[i * 3 + 2] & (0x80 >> j))
                            {
                                RocTftLcdDrawPoint(X + j + 16, Y + i, Fc);
                            }
                            else
                            {
                                if(Fc != Bc)
                                {
                                    RocTftLcdDrawPoint(X + j + 16, Y + i, Bc);
                                }
                            }
                        }
                    }
                }
            }

            pStr += 2;
            X += 24;
        }
    }
}

/*********************************************************************************
 *  Description:
 *              Double data to string data
 *
 *  Parameter:
 *              DoubleData: the double data
 *              pString:    the pointer to the string memory
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.20)
**********************************************************************************/
ROC_RESULT RocDoubleDatToStringDat(double DoubleData, uint8_t *pString)
{
    uint8_t     i = 0;
    double      DecimalData;
    int32_t     IntegerData;

    if(DoubleData >= 100000)
    {
        return RET_ERROR;
    }

    IntegerData = (int32_t)DoubleData;
    DecimalData = DoubleData - IntegerData;

    if(IntegerData >= 10000)
    {
        pString[i] = 48 + IntegerData / 10000;
        IntegerData = IntegerData % 10000;
        i++;
    }

    if(IntegerData >= 1000)
    {
        pString[i] = 48 + IntegerData / 1000;
        IntegerData = IntegerData % 1000;
        i++;
    }
    else if(IntegerData < 1000 && i != 0)
    {
        pString[i] = 0 + 48;
        i++;
    }

    if(IntegerData >= 100)
    {
        pString[i] = 48 + IntegerData / 100;
        IntegerData = IntegerData % 100;
        i++;
    }
    else if(IntegerData < 100 && i != 0)
    {
        pString[i] = 0 + 48;
        i++;
    }

    if(IntegerData >= 10)
    {
        pString[i] = 48 + IntegerData / 10;
        IntegerData = IntegerData % 10;
        i++;
    }
    else if(IntegerData < 10 && i != 0)
    {
        pString[i] = 48;
        i++;
    }

    pString[i] = 48 + IntegerData;

    if(DecimalData >= 0.000001)
    {
        i++;

        pString[i]='.';

        i++;

        IntegerData = (int)(DecimalData * 1000000);
        pString[i] = 48 + IntegerData / 100000;
        IntegerData = IntegerData % 100000;

        if(IntegerData > 0)
        {
            i++;

            pString[i] = 48 + IntegerData / 10000;
            IntegerData = IntegerData % 10;
        }

        if(IntegerData > 0)
        {
            i++;

            pString[i] = 48 + IntegerData / 1000;
            IntegerData = IntegerData % 10;
        }

        if(IntegerData > 0)
        {
            i++;

            pString[i] = 48 + IntegerData / 100;
            IntegerData = IntegerData % 10;
        }

        if(IntegerData > 0)
        {
            i++;

            pString[i] = 48 + IntegerData / 10;
            IntegerData = IntegerData % 10;
        }

        if(IntegerData >= 0)
        {
            i++;

            pString[i] = 48 + IntegerData;
        }
    }

    i++;

    pString[i]='\0';

    return RET_OK;
}

/*********************************************************************************
 *  Description:
 *              Show a error massage on TFT LCD
 *
 *  Parameter:
 *              pStr: the pointer to errot massage string
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
void RocTftLcdShowErrorMsg(uint8_t *pStr)
{
    RocTftLcdDrawGbk24Str(120, 210, ROC_TFT_LCD_COLOR_RED, ROC_TFT_LCD_COLOR_YELLOW, pStr);
}

/*********************************************************************************
 *  Description:
 *              Show a double number on TFT LCD
 *
 *  Parameter:
 *              X:    X position
 *              Y:    Y position
 *              Fc:   font colour
 *              Bc:   background colour
 *              Num:  the display double number
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
void RocTftLcdDrawGbk16Num(uint16_t X, uint16_t Y, uint16_t Fc, uint16_t Bc, double Num)
{
    ROC_RESULT Ret = RET_OK;
    uint8_t NumStr[ROC_TFT_LCD_SUPPORT_NUM_LEN + 1];

    if(ROC_TFT_LCD_SUPPORT_MAX_NUM < Num
    || ROC_TFT_LCD_SUPPORT_MIN_NUM > Num)
    {
        ROC_LOGE("Input data is out of the TFT LCD show range(%lf)", Num);
        RocTftLcdShowErrorMsg("DATA ERROR!");
    }
    else
    {
        Ret = RocDoubleDatToStringDat(Num, NumStr);
        if(RET_OK == Ret)
        {
            RocTftLcdRegionClear(X, Y, X + ROC_TFT_LCD_SUPPORT_NUM_LEN * ROC_TFT_LCD_WIDTH_GBK_16, Y + ROC_TFT_LCD_HEIGHT_GBK_16, Bc);
            RocTftLcdDrawGbk16Str(X, Y, Fc, Bc, NumStr);
        }
        else
        {
            ROC_LOGE("Double to string convertion is in error!");
            RocTftLcdShowErrorMsg("CONVERT ERROR!");
        }
    }
}

/*********************************************************************************
 *  Description:
 *              Show a double number on TFT LCD
 *
 *  Parameter:
 *              X:    X position
 *              Y:    Y position
 *              Fc:   font colour
 *              Bc:   background colour
 *              Num:  the display double number
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
void RocTftLcdDrawGbk24Num(uint16_t X, uint16_t Y, uint16_t Fc, uint16_t Bc, double Num)
{
    ROC_RESULT Ret = RET_OK;
    uint8_t NumStr[ROC_TFT_LCD_SUPPORT_NUM_LEN + 1];

    if(ROC_TFT_LCD_SUPPORT_MAX_NUM < Num
    || ROC_TFT_LCD_SUPPORT_MIN_NUM > Num)
    {
        ROC_LOGE("Input data is out of the TFT LCD show range(%lf)", Num);
        RocTftLcdShowErrorMsg("DATA ERROR!");
    }
    else
    {
        Ret = RocDoubleDatToStringDat(Num, NumStr);
        if(RET_OK == Ret)
        {
            RocTftLcdRegionClear(X, Y, X + ROC_TFT_LCD_SUPPORT_NUM_LEN * ROC_TFT_LCD_WIDTH_GBK_24, Y + ROC_TFT_LCD_HEIGHT_GBK_24, Bc);
            RocTftLcdDrawGbk24Str(X, Y, Fc, Bc, NumStr);
        }
        else
        {
            ROC_LOGE("Double to string convertion is in error!");
            RocTftLcdShowErrorMsg("CONVERT ERROR!");
        }
    }
}

/*********************************************************************************
 *  Description:
 *              Draw a button on TFT LCD
 *
 *  Parameter:
 *              X:   X position
 *              Y:   Y position
 *              Fc:  font colour
 *              Bc:  background colour
 *              Num: the num to display
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
void RocTftLcdDrawTubeNum(uint16_t X, uint16_t Y, uint16_t Fc, uint16_t Bc, uint16_t Num)
{
    uint16_t i = 0;
    uint16_t j = 0;
    uint16_t k = 0;
    uint16_t c = 0;

    for(i = 0; i < 32; i++)
    {
        for(j = 0; j < 4; j++)
        {
            c = *(g_Sz32 + Num * 32 * 4 + i * 4 + j);

            for(k = 0; k < 8; k++)
            {
                if(c & (0x80 >> k))
                {
                    RocTftLcdDrawPoint(X + j * 8 + k, Y+i, Fc);
                }
                else
                {
                    if(Fc != Bc)
                    {
                        RocTftLcdDrawPoint(X + j * 8 + k, Y + i, Bc);
                    }
                }
            }
        }
    }
}

/*********************************************************************************
 *  Description:
 *              Draw a menu test on TFT LCD
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
static void RocTftLcdMainMenuTest(void)
{
    RocTftLcdAllClear(ROC_TFT_LCD_COLOR_GRAY_0);

    RocTftLcdDrawGbk16Str(16,2,ROC_TFT_LCD_COLOR_BLUE,ROC_TFT_LCD_COLOR_GRAY_0,"全动电子技术");
    RocTftLcdDrawGbk16Str(16,20,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0,"液晶测试程序");

    RocTftLcdBgr2Rgb(0);
    RocTftLcdDisplayButtonDown(0,0,0,0);
    RocTftLcdDrawGbk24Str(0,0,0,0,0);

    RocTftLcdDisplayButtonUp(15,38,113,58); //XStart,YStart,XEnd,YEnd
    RocTftLcdDrawGbk16Str(16,40,ROC_TFT_LCD_COLOR_GREEN,ROC_TFT_LCD_COLOR_GRAY_0,"颜色填充测试");

    RocTftLcdDisplayButtonUp(15,68,113,88); //XStart,YStart,XEnd,YEnd
    RocTftLcdDrawGbk16Str(16,70,ROC_TFT_LCD_COLOR_BLUE,ROC_TFT_LCD_COLOR_GRAY_0,"文字显示测试");

    RocTftLcdDisplayButtonUp(15,98,113,118); //XStart,YStart,XEnd,YEnd
    RocTftLcdDrawGbk16Str(16,100,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0,"图片显示测试");;

    //RocTftLcdDrawGbk16Str(16,120,ROC_TFT_LCD_COLOR_BLUE,ROC_TFT_LCD_COLOR_GRAY_0,"Welcome");
    RocTftLcdDrawGbk16Str(16,140,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0, "Welcome");

    RocTftLcdDrawTubeNum(100,125,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0,g_DisplayNum[5]);
    HAL_Delay(1000);
    RocTftLcdDrawTubeNum(100,125,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0,g_DisplayNum[4]);
    HAL_Delay(1000);
    RocTftLcdDrawTubeNum(100,125,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0,g_DisplayNum[3]);
    HAL_Delay(1000);
    RocTftLcdDrawTubeNum(100,125,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0,g_DisplayNum[2]);
    HAL_Delay(1000);
    RocTftLcdDrawTubeNum(100,125,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0,g_DisplayNum[1]);
    HAL_Delay(1000);
    RocTftLcdDrawTubeNum(100,125,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0,g_DisplayNum[0]);	
}

/*********************************************************************************
 *  Description:
 *              Draw a rectangle on TFT LCD
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
static void RocTftLcdNumTest(void)
{
    uint8_t i=0;

    RocTftLcdAllClear(ROC_TFT_LCD_COLOR_GRAY_0);
    RocTftLcdDrawGbk16Str(16,20,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0,"Num Test");
    HAL_Delay(1000);
    RocTftLcdAllClear(ROC_TFT_LCD_COLOR_GRAY_0);

    for(i = 0; i < 10; i++)
    {
        RocTftLcdDrawTubeNum((i % 3) * 40, 32 * (i / 3) + 30, ROC_TFT_LCD_COLOR_RED, ROC_TFT_LCD_COLOR_GRAY_0, g_DisplayNum[i + 1]);
        HAL_Delay(100);
    }
}
/*********************************************************************************
 *  Description:
 *              Chinese and english font display test on TFT LCD
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
static void RocTftLcdFontTest(void)
{
    RocTftLcdAllClear(ROC_TFT_LCD_COLOR_GRAY_0);
    RocTftLcdDrawGbk16Str(16,10,ROC_TFT_LCD_COLOR_BLUE,ROC_TFT_LCD_COLOR_GRAY_0,"文字显示测试");

    HAL_Delay(1000);
    RocTftLcdAllClear(ROC_TFT_LCD_COLOR_GRAY_0);
    RocTftLcdDrawGbk16Str(16,30,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0,"全动电子技术");
    RocTftLcdDrawGbk16Str(16,50,ROC_TFT_LCD_COLOR_BLUE,ROC_TFT_LCD_COLOR_GRAY_0,"专注液晶批发");
    RocTftLcdDrawGbk16Str(16,70,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0, "全程技术支持");
    RocTftLcdDrawGbk16Str(0,100,ROC_TFT_LCD_COLOR_BLUE,ROC_TFT_LCD_COLOR_GRAY_0,"Tel:15989313508");
    RocTftLcdDrawGbk16Str(0,130,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0, "www.qdtech.net");	
    HAL_Delay(1500);
}

/*********************************************************************************
 *  Description:
 *              TFT LCD colour test
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
static void RocTftLcdColorTest(void)
{
    uint8_t i=1;

    RocTftLcdAllClear(ROC_TFT_LCD_COLOR_GRAY_0);

    RocTftLcdDrawGbk16Str(20,10,ROC_TFT_LCD_COLOR_BLUE,ROC_TFT_LCD_COLOR_GRAY_0,"颜色填充测试");
    HAL_Delay(1200);

    while(i--)
    {
        RocTftLcdAllClear(ROC_TFT_LCD_COLOR_WHITE); HAL_Delay(500);
        RocTftLcdAllClear(ROC_TFT_LCD_COLOR_BLACK); HAL_Delay(500);
        RocTftLcdAllClear(ROC_TFT_LCD_COLOR_RED);   HAL_Delay(500);
        RocTftLcdAllClear(ROC_TFT_LCD_COLOR_GREEN); HAL_Delay(500);
        RocTftLcdAllClear(ROC_TFT_LCD_COLOR_BLUE);  HAL_Delay(500);
    }
}

/*********************************************************************************
 *  Description:
 *              Show a picture on TFT LCD
 *
 *  Parameter:
 *              pStr: the pointer to string
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
static void RocTftLcdShowImage(const unsigned char *pStr)
{
    int32_t i,j,k;
    uint16_t picH,picL;

    RocTftLcdAllClear(ROC_TFT_LCD_COLOR_GRAY_0);
    RocTftLcdDrawGbk16Str(16,10,ROC_TFT_LCD_COLOR_BLUE,ROC_TFT_LCD_COLOR_GRAY_0, "图片显示测试");
    HAL_Delay(1000);

    RocTftLcdAllClear(ROC_TFT_LCD_COLOR_GRAY_0);

    for(k = 0; k < ROC_TFT_LCD_Y_MAX_PIXEL / 40; k++)
    {
        for(j = 0; j < ROC_TFT_LCD_X_MAX_PIXEL / 40; j++)
        {
            RocTftLcdSetRegion(40 * j, 40 * k, 40 * j + 39, 40 * k + 39);

            for(i = 0; i < 40 * 40; i++)
            {
                picL = *(pStr + i * 2);
                picH = *(pStr + i * 2 + 1);

                RocTftLcdWrite16Dat(picH << 8 | picL);
            }
        }
    }
}

/*********************************************************************************
 *  Description:
 *              TFT demo test
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
static void RocTftLcdTestDemo(void)
{
    RocTftLcdRegInit();
    RocTftLcdMainMenuTest();
    RocTftLcdColorTest();
    RocTftLcdNumTest();
    RocTftLcdFontTest();
    RocTftLcdShowImage(g_ImageQQ);
    HAL_Delay(1500);
}

/*********************************************************************************
 *  Description:
 *              TFT LCD init
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
ROC_RESULT RocTftLcdInit(void)
{
    uint16_t i = 0;
    ROC_RESULT Ret = RET_OK;

    RocTftLcdSpiSpeedSet(ROC_TFT_LCD_SPI_DAT_HIGH_SPEED);

    RocTftLcdRegInit();
    RocTftLcdAllClear(ROC_TFT_LCD_COLOR_DEFAULT_BAK);

    if(RET_OK != Ret)
    {
        RocTftLcdTestDemo();
    }

    for(i = 0; i < 48; i++)
    {
        RocTftLcdDrawLine(0, i * 5, 320, i * 5, ROC_TFT_LCD_COLOR_WHITE);
    }

    if(RET_OK != Ret)
    {
        ROC_LOGE("Remote usart init is in error!");
    }

    return Ret;
}



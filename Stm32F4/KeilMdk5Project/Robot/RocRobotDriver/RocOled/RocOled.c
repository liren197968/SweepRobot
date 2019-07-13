/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/05/02      1.0
********************************************************************************/
#include <string.h>

#include "stm32f4xx_hal.h"

#include "RocFont.h"

#include "RocLog.h"
#include "RocOled.h"
#include "RocTftLcd.h"

/*********************************************************************************
 *  Description:
 *              Write a byte data with SPI DMA communication
 *
 *  Parameter:
 *              Dat:    the data written to SPI
 *              DatLen: the data length
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.07.13)
**********************************************************************************/
static void  RocOledSpiDmaWriteData(uint8_t *Dat, uint16_t DatLen)
{
    HAL_StatusTypeDef WriteStatus;

    while(HAL_SPI_GetState(ROC_OLED_SPI_CHANNEL) != HAL_SPI_STATE_READY);

    WriteStatus = HAL_SPI_Transmit_DMA(ROC_OLED_SPI_CHANNEL, Dat, DatLen);
    if(HAL_OK != WriteStatus)
    {
        ROC_LOGE("SPI write data is in error(%d)", WriteStatus);
    }
}

/*********************************************************************************
 *  Description:
 *              Send data to OLED data bus
 *
 *  Parameter:
 *              Dat: the written data
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.07.13)
**********************************************************************************/
static void RocOledWriteBus(uint8_t Dat)
{
    RocOledSpiDmaWriteData(&Dat, 1);
}

/*********************************************************************************
 *  Description:
 *              Send data to OLED register
 *
 *  Parameter:
 *              Dat: the written data
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.07.13)
**********************************************************************************/
static void RocOledWriteDat(uint8_t Dat)
{
    ROC_OLED_DC_SET();
    RocOledWriteBus(Dat);
}

/*********************************************************************************
 *  Description:
 *              Send command to OLED data bus
 *
 *  Parameter:
 *              Cmd: the written command
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.07.13)
**********************************************************************************/
static void RocOledWriteCmd(uint8_t Cmd)
{
    ROC_OLED_DC_CLR();
    RocOledWriteBus(Cmd);
}

/*********************************************************************************
 *  Description:
 *              Set OLED display position
 *
 *  Parameter:
 *              X: the x position
 *              Y: the Y position
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.07.13)
**********************************************************************************/
static void RocOledSetPos(uint8_t X, uint8_t Y)
{ 
    RocOledWriteCmd(0xB0 + Y);
    RocOledWriteCmd(((X & 0xF0) >> 4) | 0x10);
    RocOledWriteCmd((X & 0x0F) | 0x00);
}

/*********************************************************************************
 *  Description:
 *              Fill OLED screen with color data
 *
 *  Parameter:
 *              BmpDat: the color data written to OLED
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.07.13)
**********************************************************************************/
void RocOledFillScreen(uint8_t BmpDat)
{
    uint8_t X, Y;

    for(Y = 0; Y < 8; Y++)
    {
        RocOledWriteCmd(0xB0 + Y);
        RocOledWriteCmd(0x01);
        RocOledWriteCmd(0x10);

        for(X = 0; X < ROC_OLED_X_SIZE; X++)
        {
            RocOledWriteDat(BmpDat);
        }
    }
}

/*********************************************************************************
 *  Description:
 *              Init OLED register
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.07.13)
**********************************************************************************/
void RocOledClearScreen(void)
{
    uint8_t X, Y;

    for(Y = 0; Y < 8; Y++)
    {
        RocOledWriteCmd(0xB0 + Y);
        RocOledWriteCmd(0x01);
        RocOledWriteCmd(0x10);

        for(X = 0;X < ROC_OLED_X_SIZE; X++)
        {
            RocOledWriteDat(0);
        }
    }
}

/*********************************************************************************
 *  Description:
 *              Init OLED register
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.07.13)
**********************************************************************************/
static void RocOledRegInit(void)
{
    ROC_OLED_RST_CLR();
    HAL_Delay(50);
    ROC_OLED_RST_SET();

    RocOledWriteCmd(0xae);      //--turn off oled panel
    RocOledWriteCmd(0x00);      //--set low column address
    RocOledWriteCmd(0x10);      //--set high column address
    RocOledWriteCmd(0x40);      //--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    RocOledWriteCmd(0x81);      //--set contrast control register
    RocOledWriteCmd(0xcf);      //--Set SEG Output Current Brightness
    RocOledWriteCmd(0xa1);      //--Set SEG/Column Mapping          0xa0左右反置 0xa1正常
    RocOledWriteCmd(0xc8);      //--Set COM/Row Scan Direction      0xc0上下反置 0xc8正常
    RocOledWriteCmd(0xa6);      //--set normal display
    RocOledWriteCmd(0xa8);      //--set multiplex ratio(1 to 64)
    RocOledWriteCmd(0x3f);      //--1/64 duty
    RocOledWriteCmd(0xd3);      //--set display offset	Shift Mapping RAM Counter (0x00~0x3F)
    RocOledWriteCmd(0x00);      //--not offset
    RocOledWriteCmd(0xd5);      //--set display clock divide ratio/oscillator frequency
    RocOledWriteCmd(0x80);      //--set divide ratio, Set Clock as 100 Frames/Sec
    RocOledWriteCmd(0xd9);      //--set pre-charge period
    RocOledWriteCmd(0xf1);      //--Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    RocOledWriteCmd(0xda);      //--set com pins hardware configuration
    RocOledWriteCmd(0x12);
    RocOledWriteCmd(0xdb);      //--set vcomh
    RocOledWriteCmd(0x40);      //--Set VCOM Deselect Level
    RocOledWriteCmd(0x20);      //-Set Page Addressing Mode (0x00/0x01/0x02)
    RocOledWriteCmd(0x02);      //--
    RocOledWriteCmd(0x8d);      //--set Charge Pump enable/disable
    RocOledWriteCmd(0x14);      //--set(0x10) disable
    RocOledWriteCmd(0xa4);      //--Disable Entire Display On (0xa4/0xa5)
    RocOledWriteCmd(0xa6);      //--Disable Inverse Display On (0xa6/a7) 
    RocOledWriteCmd(0xaf);      //--turn on oled panel
    RocOledFillScreen(0x00);    //--初始清屏
    RocOledSetPos(0,0);
} 

/*********************************************************************************
 *  Description:
 *              Write 6*8 ASCII char to OLED
 *
 *  Parameter:
 *              X: the x position of OLED
 *              Y: the page of OLED(0 ~ 7)
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.07.13)
**********************************************************************************/
void RocOledDrawGbk8Char(uint8_t X, uint8_t Y, uint8_t Char)
{
    uint8_t i, DataTmp;

    DataTmp = Char - 32;

    if(X > 126)
    {
        X = 0;
        Y++;
    }

    RocOledSetPos(X, Y);

    for(i = 0; i < 6; i++)
    {
        RocOledWriteDat(g_OledFont6x8[DataTmp][i]);
    }
}

/*********************************************************************************
 *  Description:
 *              Write 6*8 ASCII string to OLED
 *
 *  Parameter:
 *              X: the x position of OLED
 *              Y: the page of OLED(0 ~ 7)
 *              S: the string data
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.07.13)
**********************************************************************************/
void RocOledDrawGbk8Str(uint8_t     X, uint8_t Y, uint8_t *S)
{
    uint8_t c = 0, i = 0, j = 0;

    while(S[j] != '\0')
    {    
        c = S[j] - 32;

        if(X > 126)
        {
            X = 0;
            Y++;
        }

        RocOledSetPos(X, Y);

        for(i=0; i<6; i++)
        {
          RocOledWriteDat(g_OledFont6x8[c][i]);
        }

        X += 6;
        j++;
    }
}

/*********************************************************************************
 *  Description:
 *              Write 6*8 number to OLED
 *
 *  Parameter:
 *              X: the x position of OLED
 *              Y: the page of OLED(0 ~ 7)
 *              N: the number data
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.07.13)
**********************************************************************************/
void RocOledDrawGbk8Num(uint8_t X, uint8_t Y, float N)
{
    uint8_t     NumStr[ROC_OLED_SUPPORT_NUM_LEN + 1];

    RocDoubleDatToStringDat(N, NumStr);

    RocOledDrawGbk8Str(X, Y, NumStr);
}

/*********************************************************************************
 *  Description:
 *              Write 8*16 ASCII char to OLED
 *
 *  Parameter:
 *              X: the x position of OLED
 *              Y: the page of OLED(0 ~ 7)
 *              Char: the char data
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.07.13)
**********************************************************************************/
void RocOledDrawGbk16Char(uint8_t X, uint8_t Y, uint8_t Char)
{
    uint8_t c = 0, i = 0;

    c = Char - 32;

    if(X > 120)
    {
        X = 0;
        Y++;
    }

    RocOledSetPos(X, Y);

    for(i = 0; i < 8; i++)
    {
        RocOledWriteDat(g_OledFont8x16[c * 16 + i]);
    }

    RocOledSetPos(X, Y + 1);

    for(i = 0; i < 8; i++)
    {
        RocOledWriteDat(g_OledFont8x16[c * 16 + i + 8]);
    }

    X += 8;
}

/*********************************************************************************
 *  Description:
 *              Write 8*16 ASCII string to OLED
 *
 *  Parameter:
 *              X: the x position of OLED
 *              Y: the page of OLED(0 ~ 7)
 *              S: the string data
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.07.13)
**********************************************************************************/
void RocOledDrawGbk16Str(uint8_t X, uint8_t Y, uint8_t *S)
{
    uint8_t c = 0, i = 0, j = 0;

    while (S[j]!='\0')
    {    
        c = S[j] - 32;

        if(X > 120)
        {
            X=0;
            Y++;
        }

        RocOledSetPos(X,Y);

        for(i=0;i<8;i++)
        {
          RocOledWriteDat(g_OledFont8x16[c*16+i]);
        }

        RocOledSetPos(X,Y+1);

        for(i=0;i<8;i++)
        {
          RocOledWriteDat(g_OledFont8x16[c*16+i+8]);
        }

        X += 8;

        j++;
    }
}

/*********************************************************************************
 *  Description:
 *              Write 8*16 number to OLED
 *
 *  Parameter:
 *              X: the x position of OLED
 *              Y: the page of OLED(0 ~ 7)
 *              N: the number data
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.07.13)
**********************************************************************************/
void RocOledDrawGbk16Num(uint8_t X, uint8_t Y, float N)
{
    uint8_t     NumStr[ROC_OLED_SUPPORT_NUM_LEN + 1];

    RocDoubleDatToStringDat(N, NumStr);

    RocOledDrawGbk16Str(X, Y, NumStr);
}

/*********************************************************************************
 *  Description:
 *              OLED init
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.05.02)
**********************************************************************************/
ROC_RESULT RocOledInit(void)
{
    ROC_RESULT Ret = RET_OK;

    RocOledRegInit();

    RocOledDrawGbk8Str(30, 3, "Init Success");

    if(RET_OK != Ret)
    {
        ROC_LOGE("OLED init is in error!");
    }

    return Ret;
}



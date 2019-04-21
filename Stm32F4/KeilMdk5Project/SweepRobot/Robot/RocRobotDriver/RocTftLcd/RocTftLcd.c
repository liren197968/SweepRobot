/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/04/21      1.0
********************************************************************************/
#include <string.h>

#include "gpio.h"
#include "spi.h"

#include "font.h"
#include "Picture.h"

#include "RocLog.h"
#include "RocTftLcd.h"


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
 *              Set SPI speed
 *
 *  Parameter:
 *              Spix: the SPI number
 *              SpeedSet: the control speed
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.21)
**********************************************************************************/
void RocSpiSpeedSet(SPI_TypeDef* Spix,uint8_t SpeedSet)
{
    Spix->CR1 &= 0XFFC7;

    if(SpeedSet == 1)
    {
    	Spix->CR1 |= SPI_BAUDRATEPRESCALER_2;	
    }
    else
    {
    	Spix->CR1 |= SPI_BAUDRATEPRESCALER_32;
    }

    Spix->CR1 |= 1<<6;
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
    RocTftLcdWriteDat(Data>>8);
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
static void RocTftLcdRegWriteCmd(uint8_t Reg,uint16_t Data)
{
    RocTftLcdWriteReg(Reg);
    RocTftLcdWrite16Dat(Data);
}

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

/*************************************************
函数名：LCD_Set_XY
功能：设置lcd显示起始点
入口参数：xy坐标
返回值：无
*************************************************/
void Lcd_SetXY(uint16_t Xpos, uint16_t Ypos)
{	
	RocTftLcdWriteReg(0x2A);
	RocTftLcdWrite16Dat(Xpos);
	RocTftLcdWriteReg(0x2B);
	RocTftLcdWrite16Dat(Ypos);
	RocTftLcdWriteReg(0x2c);	
} 
/*************************************************
函数名：LCD_Set_Region
功能：设置lcd显示区域，在此区域写点数据自动换行
入口参数：xy起点和终点
返回值：无
*************************************************/
//设置显示窗口
void Lcd_SetRegion(uint16_t xStar, uint16_t yStar,uint16_t xEnd,uint16_t yEnd)
{
	RocTftLcdWriteReg(0x2A);
	RocTftLcdWrite16Dat(xStar);
	RocTftLcdWrite16Dat(xEnd);
	RocTftLcdWriteReg(0x2B);
	RocTftLcdWrite16Dat(yStar);
	RocTftLcdWrite16Dat(yEnd);
	RocTftLcdWriteReg(0x2c);
}

	
/*************************************************
函数名：LCD_DrawPoint
功能：画一个点
入口参数：xy坐标和颜色数据
返回值：无
*************************************************/
void RocTftLcdDrawPoint(uint16_t x, uint16_t y, uint16_t Data)
{
	Lcd_SetXY(x,y);
	RocTftLcdWrite16Dat(Data);
}

/*************************************************
函数名：Lcd_Clear
功能：全屏清屏函数
入口参数：填充颜色COLOR
返回值：无
*************************************************/
void RocTftLcdAllClear(uint16_t BakColor)
{	
   unsigned int i;
   Lcd_SetRegion(0,0,ROC_TFT_LCD_X_MAX_PIXEL-1,ROC_TFT_LCD_Y_MAX_PIXEL-1);
   ROC_TFT_LCD_RS_SET();	
   for(i=0;i<ROC_TFT_LCD_X_MAX_PIXEL*ROC_TFT_LCD_Y_MAX_PIXEL;i++)
   {	
	  	//RocTftLcdWrite16Dat(BakColor);
		RocSpiWriteData(BakColor>>8);
		RocSpiWriteData(BakColor);
#endif 
   }   
}


//从ILI93xx读出的数据为GBR格式，而我们写入的时候为RGB格式。
//通过该函数转换
//c:GBR格式的颜色值
//返回值：RGB格式的颜色值
uint16_t LCD_BGR2RGB(uint16_t c)
{
  uint16_t  r,g,b,rgb;   
  b=(c>>0)&0x1f;
  g=(c>>5)&0x3f;
  r=(c>>11)&0x1f;	 
  rgb=(b<<11)+(g<<5)+(r<<0);		 
  return(rgb);

}


void Gui_Circle(uint16_t X,uint16_t Y,uint16_t R,uint16_t fc) 
{//Bresenham算法 
    unsigned short  a,b; 
    int c; 
    a=0; 
    b=R; 
    c=3-2*R; 
    while (a<b) 
    { 
        RocTftLcdDrawPoint(X+a,Y+b,fc);     //        7 
        RocTftLcdDrawPoint(X-a,Y+b,fc);     //        6 
        RocTftLcdDrawPoint(X+a,Y-b,fc);     //        2 
        RocTftLcdDrawPoint(X-a,Y-b,fc);     //        3 
        RocTftLcdDrawPoint(X+b,Y+a,fc);     //        8 
        RocTftLcdDrawPoint(X-b,Y+a,fc);     //        5 
        RocTftLcdDrawPoint(X+b,Y-a,fc);     //        1 
        RocTftLcdDrawPoint(X-b,Y-a,fc);     //        4 

        if(c<0) c=c+4*a+6; 
        else 
        { 
            c=c+4*(a-b)+10; 
            b-=1; 
        } 
       a+=1; 
    } 
    if (a==b) 
    { 
        RocTftLcdDrawPoint(X+a,Y+b,fc); 
        RocTftLcdDrawPoint(X+a,Y+b,fc); 
        RocTftLcdDrawPoint(X+a,Y-b,fc); 
        RocTftLcdDrawPoint(X-a,Y-b,fc); 
        RocTftLcdDrawPoint(X+b,Y+a,fc); 
        RocTftLcdDrawPoint(X-b,Y+a,fc); 
        RocTftLcdDrawPoint(X+b,Y-a,fc); 
        RocTftLcdDrawPoint(X-b,Y-a,fc); 
    } 
	
} 
//画线函数，使用Bresenham 画线算法
void Gui_DrawLine(uint16_t x0, uint16_t y0,uint16_t x1, uint16_t y1,uint16_t Color)   
{
int dx,             // difference in x's
    dy,             // difference in y's
    dx2,            // dx,dy * 2
    dy2, 
    x_inc,          // amount in pixel space to move during drawing
    y_inc,          // amount in pixel space to move during drawing
    error,          // the discriminant i.e. error i.e. decision variable
    index;          // used for looping	


	Lcd_SetXY(x0,y0);
	dx = x1-x0;//计算x距离
	dy = y1-y0;//计算y距离

	if (dx>=0)
	{
		x_inc = 1;
	}
	else
	{
		x_inc = -1;
		dx    = -dx;  
	} 
	
	if (dy>=0)
	{
		y_inc = 1;
	} 
	else
	{
		y_inc = -1;
		dy    = -dy; 
	} 

	dx2 = dx << 1;
	dy2 = dy << 1;

	if (dx > dy)//x距离大于y距离，那么每个x轴上只有一个点，每个y轴上有若干个点
	{//且线的点数等于x距离，以x轴递增画点
		// initialize error term
		error = dy2 - dx; 

		// draw the line
		for (index=0; index <= dx; index++)//要画的点数不会超过x距离
		{
			//画点
			RocTftLcdDrawPoint(x0,y0,Color);
			
			// test if error has overflowed
			if (error >= 0) //是否需要增加y坐标值
			{
				error-=dx2;

				// move to next line
				y0+=y_inc;//增加y坐标值
			} // end if error overflowed

			// adjust the error term
			error+=dy2;

			// move to the next pixel
			x0+=x_inc;//x坐标值每次画点后都递增1
		} // end for
	} // end if |slope| <= 1
	else//y轴大于x轴，则每个y轴上只有一个点，x轴若干个点
	{//以y轴为递增画点
		// initialize error term
		error = dx2 - dy; 

		// draw the line
		for (index=0; index <= dy; index++)
		{
			// set the pixel
			RocTftLcdDrawPoint(x0,y0,Color);

			// test if error overflowed
			if (error >= 0)
			{
				error-=dy2;

				// move to next line
				x0+=x_inc;
			} // end if error overflowed

			// adjust the error term
			error+=dx2;

			// move to the next pixel
			y0+=y_inc;
		} // end for
	} // end else |slope| > 1
}



void Gui_box(uint16_t x, uint16_t y, uint16_t w, uint16_t h,uint16_t bc)
{
	Gui_DrawLine(x,y,x+w,y,0xEF7D);
	Gui_DrawLine(x+w-1,y+1,x+w-1,y+1+h,0x2965);
	Gui_DrawLine(x,y+h,x+w,y+h,0x2965);
	Gui_DrawLine(x,y,x,y+h,0xEF7D);
    Gui_DrawLine(x+1,y+1,x+1+w-2,y+1+h-2,bc);
}
void Gui_box2(uint16_t x,uint16_t y,uint16_t w,uint16_t h, uint8_t mode)
{
	if (mode==0)	{
		Gui_DrawLine(x,y,x+w,y,0xEF7D);
		Gui_DrawLine(x+w-1,y+1,x+w-1,y+1+h,0x2965);
		Gui_DrawLine(x,y+h,x+w,y+h,0x2965);
		Gui_DrawLine(x,y,x,y+h,0xEF7D);
		}
	if (mode==1)	{
		Gui_DrawLine(x,y,x+w,y,0x2965);
		Gui_DrawLine(x+w-1,y+1,x+w-1,y+1+h,0xEF7D);
		Gui_DrawLine(x,y+h,x+w,y+h,0xEF7D);
		Gui_DrawLine(x,y,x,y+h,0x2965);
	}
	if (mode==2)	{
		Gui_DrawLine(x,y,x+w,y,0xffff);
		Gui_DrawLine(x+w-1,y+1,x+w-1,y+1+h,0xffff);
		Gui_DrawLine(x,y+h,x+w,y+h,0xffff);
		Gui_DrawLine(x,y,x,y+h,0xffff);
	}
}


/**************************************************************************************
功能描述: 在屏幕显示一凸起的按钮框
输    入: uint16_t x1,y1,x2,y2 按钮框左上角和右下角坐标
输    出: 无
**************************************************************************************/
void DisplayButtonDown(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
	Gui_DrawLine(x1,  y1,  x2,y1, ROC_TFT_LCD_COLOR_GRAY_2);  //H
	Gui_DrawLine(x1+1,y1+1,x2,y1+1, ROC_TFT_LCD_COLOR_GRAY_1);  //H
	Gui_DrawLine(x1,  y1,  x1,y2, ROC_TFT_LCD_COLOR_GRAY_2);  //V
	Gui_DrawLine(x1+1,y1+1,x1+1,y2, ROC_TFT_LCD_COLOR_GRAY_1);  //V
	Gui_DrawLine(x1,  y2,  x2,y2, ROC_TFT_LCD_COLOR_WHITE);  //H
	Gui_DrawLine(x2,  y1,  x2,y2, ROC_TFT_LCD_COLOR_WHITE);  //V
}

/**************************************************************************************
功能描述: 在屏幕显示一凹下的按钮框
输    入: uint16_t x1,y1,x2,y2 按钮框左上角和右下角坐标
输    出: 无
**************************************************************************************/
void DisplayButtonUp(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
	Gui_DrawLine(x1,  y1,  x2,y1, ROC_TFT_LCD_COLOR_WHITE); //H
	Gui_DrawLine(x1,  y1,  x1,y2, ROC_TFT_LCD_COLOR_WHITE); //V
	
	Gui_DrawLine(x1+1,y2-1,x2,y2-1, ROC_TFT_LCD_COLOR_GRAY_1);  //H
	Gui_DrawLine(x1,  y2,  x2,y2, ROC_TFT_LCD_COLOR_GRAY_2);  //H
	Gui_DrawLine(x2-1,y1+1,x2-1,y2, ROC_TFT_LCD_COLOR_GRAY_1);  //V
    Gui_DrawLine(x2  ,y1  ,x2,y2, ROC_TFT_LCD_COLOR_GRAY_2); //V
}


void Gui_DrawFont_GBK16(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t *s)
{
	unsigned char i,j;
	unsigned short k,x0;
	x0=x;

	while(*s) 
	{	
		if((*s) < 128) 
		{
			k=*s;
			if (k==13) 
			{
				x=x0;
				y+=16;
			}
			else 
			{
				if (k>32) k-=32; else k=0;
	
			    for(i=0;i<16;i++)
				for(j=0;j<8;j++) 
					{
				    	if(asc16[k*16+i]&(0x80>>j))	RocTftLcdDrawPoint(x+j,y+i,fc);
						else 
						{
							if (fc!=bc) RocTftLcdDrawPoint(x+j,y+i,bc);
						}
					}
				x+=8;
			}
			s++;
		}
			
		else 
		{
		

			for (k=0;k<hz16_num;k++) 
			{
			  if ((hz16[k].Index[0]==*(s))&&(hz16[k].Index[1]==*(s+1)))
			  { 
				    for(i=0;i<16;i++)
				    {
						for(j=0;j<8;j++) 
							{
						    	if(hz16[k].Msk[i*2]&(0x80>>j))	RocTftLcdDrawPoint(x+j,y+i,fc);
								else {
									if (fc!=bc) RocTftLcdDrawPoint(x+j,y+i,bc);
								}
							}
						for(j=0;j<8;j++) 
							{
						    	if(hz16[k].Msk[i*2+1]&(0x80>>j))	RocTftLcdDrawPoint(x+j+8,y+i,fc);
								else 
								{
									if (fc!=bc) RocTftLcdDrawPoint(x+j+8,y+i,bc);
								}
							}
				    }
				}
			  }
			s+=2;x+=16;
		} 
		
	}
}

void Gui_DrawFont_GBK24(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t *s)
{
	unsigned char i,j;
	unsigned short k;

	while(*s) 
	{
		if( *s < 0x80 ) 
		{
			k=*s;
			if (k>32) k-=32; else k=0;

		    for(i=0;i<16;i++)
			for(j=0;j<8;j++) 
				{
			    	if(asc16[k*16+i]&(0x80>>j))	
					RocTftLcdDrawPoint(x+j,y+i,fc);
					else 
					{
						if (fc!=bc) RocTftLcdDrawPoint(x+j,y+i,bc);
					}
				}
			s++;x+=8;
		}
		else 
		{

			for (k=0;k<hz24_num;k++) 
			{
			  if ((hz24[k].Index[0]==*(s))&&(hz24[k].Index[1]==*(s+1)))
			  { 
				    for(i=0;i<24;i++)
				    {
						for(j=0;j<8;j++) 
							{
						    	if(hz24[k].Msk[i*3]&(0x80>>j))
								RocTftLcdDrawPoint(x+j,y+i,fc);
								else 
								{
									if (fc!=bc) RocTftLcdDrawPoint(x+j,y+i,bc);
								}
							}
						for(j=0;j<8;j++) 
							{
						    	if(hz24[k].Msk[i*3+1]&(0x80>>j))	RocTftLcdDrawPoint(x+j+8,y+i,fc);
								else {
									if (fc!=bc) RocTftLcdDrawPoint(x+j+8,y+i,bc);
								}
							}
						for(j=0;j<8;j++) 
							{
						    	if(hz24[k].Msk[i*3+2]&(0x80>>j))	
								RocTftLcdDrawPoint(x+j+16,y+i,fc);
								else 
								{
									if (fc!=bc) RocTftLcdDrawPoint(x+j+16,y+i,bc);
								}
							}
				    }
			  }
			}
			s+=2;x+=24;
		}
	}
}
void RocDrawFontDigitalTubeNum(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint16_t num)
{
	unsigned char i,j,k,c;
	//lcd_text_any(x+94+i*42,y+34,32,32,0x7E8,0x0,sz32,knum[i]);
//	w=w/8;

    for(i=0;i<32;i++)
	{
		for(j=0;j<4;j++) 
		{
			c=*(sz32+num*32*4+i*4+j);
			for (k=0;k<8;k++)	
			{
	
		    	if(c&(0x80>>k))	RocTftLcdDrawPoint(x+j*8+k,y+i,fc);
				else {
					if (fc!=bc) RocTftLcdDrawPoint(x+j*8+k,y+i,bc);
				}
			}
		}
	}
}


unsigned char Num[10]={0,1,2,3,4,5,6,7,8,9};
//绘制测试菜单
//2D按键按钮示例
void Redraw_Mainmenu(void)
{

	RocTftLcdAllClear(ROC_TFT_LCD_COLOR_GRAY_0);
	
	Gui_DrawFont_GBK16(16,2,ROC_TFT_LCD_COLOR_BLUE,ROC_TFT_LCD_COLOR_GRAY_0,"全动电子技术");
	Gui_DrawFont_GBK16(16,20,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0,"液晶测试程序");

	DisplayButtonUp(15,38,113,58); //x1,y1,x2,y2
	Gui_DrawFont_GBK16(16,40,ROC_TFT_LCD_COLOR_GREEN,ROC_TFT_LCD_COLOR_GRAY_0,"颜色填充测试");

	DisplayButtonUp(15,68,113,88); //x1,y1,x2,y2
	Gui_DrawFont_GBK16(16,70,ROC_TFT_LCD_COLOR_BLUE,ROC_TFT_LCD_COLOR_GRAY_0,"文字显示测试");

	DisplayButtonUp(15,98,113,118); //x1,y1,x2,y2
	Gui_DrawFont_GBK16(16,100,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0,"图片显示测试");;

	//Gui_DrawFont_GBK16(16,120,ROC_TFT_LCD_COLOR_BLUE,ROC_TFT_LCD_COLOR_GRAY_0,"Welcome");
	Gui_DrawFont_GBK16(16,140,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0, "Welcome");
	
	RocDrawFontDigitalTubeNum(100,125,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0,Num[5]);
	HAL_Delay(1000);
	RocDrawFontDigitalTubeNum(100,125,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0,Num[4]);
	HAL_Delay(1000);
	RocDrawFontDigitalTubeNum(100,125,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0,Num[3]);
	HAL_Delay(1000);
	RocDrawFontDigitalTubeNum(100,125,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0,Num[2]);
	HAL_Delay(1000);
	RocDrawFontDigitalTubeNum(100,125,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0,Num[1]);
	HAL_Delay(1000);
	RocDrawFontDigitalTubeNum(100,125,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0,Num[0]);	
}
//测试数码管字体
void Num_Test(void)
{
	uint8_t i=0;
	RocTftLcdAllClear(ROC_TFT_LCD_COLOR_GRAY_0);
	Gui_DrawFont_GBK16(16,20,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0,"Num Test");
	HAL_Delay(1000);
	RocTftLcdAllClear(ROC_TFT_LCD_COLOR_GRAY_0);

	for(i=0;i<10;i++)
	{
	RocDrawFontDigitalTubeNum((i%3)*40,32*(i/3)+30,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0,Num[i+1]);
	HAL_Delay(100);
	}
	
}
//中英文显示测试
void Font_Test(void)
{
	RocTftLcdAllClear(ROC_TFT_LCD_COLOR_GRAY_0);
	Gui_DrawFont_GBK16(16,10,ROC_TFT_LCD_COLOR_BLUE,ROC_TFT_LCD_COLOR_GRAY_0,"文字显示测试");

	HAL_Delay(1000);
	RocTftLcdAllClear(ROC_TFT_LCD_COLOR_GRAY_0);
	Gui_DrawFont_GBK16(16,30,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0,"全动电子技术");
	Gui_DrawFont_GBK16(16,50,ROC_TFT_LCD_COLOR_BLUE,ROC_TFT_LCD_COLOR_GRAY_0,"专注液晶批发");
	Gui_DrawFont_GBK16(16,70,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0, "全程技术支持");
	Gui_DrawFont_GBK16(0,100,ROC_TFT_LCD_COLOR_BLUE,ROC_TFT_LCD_COLOR_GRAY_0,"Tel:15989313508");
	Gui_DrawFont_GBK16(0,130,ROC_TFT_LCD_COLOR_RED,ROC_TFT_LCD_COLOR_GRAY_0, "www.qdtech.net");	
	HAL_Delay(1500);	
}
//简单刷屏测试
void Color_Test(void)
{
	uint8_t i=1;
	RocTftLcdAllClear(ROC_TFT_LCD_COLOR_GRAY_0);
	
	Gui_DrawFont_GBK16(20,10,ROC_TFT_LCD_COLOR_BLUE,ROC_TFT_LCD_COLOR_GRAY_0,"颜色填充测试");
	HAL_Delay(1200);

	while(i--)
	{
		RocTftLcdAllClear(ROC_TFT_LCD_COLOR_WHITE); HAL_Delay(500);
		RocTftLcdAllClear(ROC_TFT_LCD_COLOR_BLACK); HAL_Delay(500);
		RocTftLcdAllClear(ROC_TFT_LCD_COLOR_RED);	  HAL_Delay(500);
	  	RocTftLcdAllClear(ROC_TFT_LCD_COLOR_GREEN); HAL_Delay(500);
	  	RocTftLcdAllClear(ROC_TFT_LCD_COLOR_BLUE);  HAL_Delay(500);
	}		
}

//文字显示测试
//16位BMP 40X40 QQ图像取模数据
//Image2LCD取模选项设置
//水平扫描
//16位
//40X40
//不包含图像头数据
//自左至右
//自顶至底
//低位在前
void showimage(const unsigned char *p) //显示40*40 QQ图片
{
  	int i,j,k; 
	unsigned char picH,picL; 
	RocTftLcdAllClear(ROC_TFT_LCD_COLOR_GRAY_0);
	Gui_DrawFont_GBK16(16,10,ROC_TFT_LCD_COLOR_BLUE,ROC_TFT_LCD_COLOR_GRAY_0,"图片显示测试");
	HAL_Delay(1000);

	RocTftLcdAllClear(ROC_TFT_LCD_COLOR_GRAY_0);
	for(k=0;k<ROC_TFT_LCD_Y_MAX_PIXEL/40;k++)
	{
	   	for(j=0;j<ROC_TFT_LCD_X_MAX_PIXEL/40;j++)
		{	
			Lcd_SetRegion(40*j,40*k,40*j+39,40*k+39);		//坐标设置
		    for(i=0;i<40*40;i++)
			 {	
			 	picL=*(p+i*2);	//数据低位在前
				picH=*(p+i*2+1);				
				RocTftLcdWrite16Dat(picH<<8|picL);  						
			 }	
		 }
	}		
}
//综合测试函数
static void QDTFT_Test_Demo(void)
{
	RocTftLcdRegInit();
	Redraw_Mainmenu();//绘制主菜单(部分内容由于分辨率超出物理值可能无法显示)
	Color_Test();//简单纯色填充测试
	Num_Test();//数码管字体测试
	Font_Test();//中英文显示测试		
	showimage(gImage_qq);//图片显示示例
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

    RocTftLcdRegInit();
    RocTftLcdAllClear(ROC_TFT_LCD_COLOR_DEFAULT_BAK);

    for(i = 0; i < 48; i++)
    {
        Gui_DrawLine(0, i * 5, 320, i * 5, ROC_TFT_LCD_COLOR_WHITE);
    }

    if(RET_OK != Ret)
    {
        ROC_LOGE("Remote usart init is in error!");
    }

    return Ret;
}



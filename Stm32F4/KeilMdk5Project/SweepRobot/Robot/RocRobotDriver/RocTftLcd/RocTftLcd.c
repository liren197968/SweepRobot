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

/****************************************************************************
* 名    称：void  SPIv_WriteData(uint8_t Data)
* 功    能：STM32_模拟SPI写一个字节数据底层函数
* 入口参数：Data
* 出口参数：无
* 说    明：STM32_模拟SPI读写一个字节数据底层函数
****************************************************************************/
void  SPIv_WriteData(uint8_t Data)
{
    HAL_StatusTypeDef WriteStatus;

    WriteStatus = HAL_SPI_Transmit(&hspi1, &Data, 1, ROC_TFT_LCD_WRITE_TIME_OUT);
    if(HAL_OK != WriteStatus)
    {
        ROC_LOGE("SPI write data is in error(%d)", WriteStatus);
    }

    while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
}

/****************************************************************************
* 名    称：u8 SPI_WriteByte(SPI_TypeDef* SPIx,uint8_t Byte)
* 功    能：STM32_硬件SPI读写一个字节数据底层函数
* 入口参数：SPIx,Byte
* 出口参数：返回总线收到的数据
* 说    明：STM32_硬件SPI读写一个字节数据底层函数
****************************************************************************/
uint8_t SPI_WriteByte(SPI_TypeDef* SPIx,uint8_t Byte)
{
		
}

/****************************************************************************
* 名    称：void SPI_SetSpeed(SPI_TypeDef* SPIx,uint8_t SpeedSet)
* 功    能：设置SPI的速度
* 入口参数：SPIx,SpeedSet
* 出口参数：无
* 说    明：SpeedSet:1,高速;0,低速;
****************************************************************************/
void SPI_SetSpeed(SPI_TypeDef* SPIx,uint8_t SpeedSet)
{
	SPIx->CR1&=0XFFC7;
	if(SpeedSet==1)//高速
	{
		SPIx->CR1|=SPI_BAUDRATEPRESCALER_2;//Fsck=Fpclk/2	
	}
	else//低速
	{
		SPIx->CR1|=SPI_BAUDRATEPRESCALER_32; //Fsck=Fpclk/32
	}
	SPIx->CR1|=1<<6; //SPI设备使能
}

/****************************************************************************
* 名    称：Lcd_WriteIndex(uint8_t Index)
* 功    能：向液晶屏写一个8位指令
* 入口参数：Index   寄存器地址
* 出口参数：无
* 说    明：调用前需先选中控制器，内部函数
****************************************************************************/
void Lcd_WriteIndex(uint8_t Index)
{
   LCD_RS_CLR;
#if USE_HARDWARE_SPI
   SPI_WriteByte(SPI2,Index);
#else
   SPIv_WriteData(Index);
#endif
}

/****************************************************************************
* 名    称：Lcd_WriteData(uint8_t Data)
* 功    能：向液晶屏写一个8位数据
* 入口参数：dat     寄存器数据
* 出口参数：无
* 说    明：向控制器指定地址写入数据，内部函数
****************************************************************************/
void Lcd_WriteData(uint8_t Data)
{
   LCD_RS_SET;
#if USE_HARDWARE_SPI   
   SPI_WriteByte(SPI2,Data);
#else
   SPIv_WriteData(Data);
#endif 
}

/****************************************************************************
* 名    称：void Lcd_WriteData_16Bit(uint16_t Data)
* 功    能：向液晶屏写一个16位数据
* 入口参数：Data
* 出口参数：无
* 说    明：向控制器指定地址写入一个16位数据
****************************************************************************/
void Lcd_WriteData_16Bit(uint16_t Data)
{	
	Lcd_WriteData(Data>>8);
	Lcd_WriteData(Data);	
}

/****************************************************************************
* 名    称：void LCD_WriteReg(uint8_t Index,uint16_t Data)
* 功    能：写寄存器数据
* 入口参数：Index,Data
* 出口参数：无
* 说    明：本函数为组合函数，向Index地址的寄存器写入Data值
****************************************************************************/
void LCD_WriteReg(uint8_t Index,uint16_t Data)
{
	Lcd_WriteIndex(Index);
  	Lcd_WriteData_16Bit(Data);
}

/****************************************************************************
* 名    称：void Lcd_Reset(void)
* 功    能：液晶硬复位函数
* 入口参数：无
* 出口参数：无
* 说    明：液晶初始化前需执行一次复位操作
****************************************************************************/
void Lcd_Reset(void)
{
	LCD_RST_CLR;
	HAL_Delay(100);
	LCD_RST_SET;
	HAL_Delay(50);
}
/****************************************************************************
* 名    称：void Lcd_Init(void)
* 功    能：液晶初始化函数
* 入口参数：无
* 出口参数：无
* 说    明：液晶初始化_ILI9225_176X220
****************************************************************************/
void Lcd_Init(void)
{	
	Lcd_Reset(); //Reset before LCD Init.

	//2.2inch TM2.2-G2.2 Init 20171020 
	Lcd_WriteIndex(0x11);  
	Lcd_WriteData(0x00); 

	Lcd_WriteIndex(0xCF);  
	Lcd_WriteData(0X00); 
	Lcd_WriteData(0XC1); 
	Lcd_WriteData(0X30);

	Lcd_WriteIndex(0xED);
	Lcd_WriteData(0X64); 
	Lcd_WriteData(0X03); 
	Lcd_WriteData(0X12);
	Lcd_WriteData(0X81);

	Lcd_WriteIndex(0xE8);
	Lcd_WriteData(0X85);
	Lcd_WriteData(0X11);
	Lcd_WriteData(0X78);

	Lcd_WriteIndex(0xF6);  
	Lcd_WriteData(0X01); 
	Lcd_WriteData(0X30); 
	Lcd_WriteData(0X00);

	Lcd_WriteIndex(0xCB);  
	Lcd_WriteData(0X39); 
	Lcd_WriteData(0X2C); 
	Lcd_WriteData(0X00);
	Lcd_WriteData(0X34);
	Lcd_WriteData(0X05);

	Lcd_WriteIndex(0xF7);  
	Lcd_WriteData(0X20); 

	Lcd_WriteIndex(0xEA);  
	Lcd_WriteData(0X00); 
	Lcd_WriteData(0X00); 

	Lcd_WriteIndex(0xC0);  
	Lcd_WriteData(0X20); 

	Lcd_WriteIndex(0xC1);  
	Lcd_WriteData(0X11); 

	Lcd_WriteIndex(0xC5);  
	Lcd_WriteData(0X31); 
	Lcd_WriteData(0X3C); 

	Lcd_WriteIndex(0xC7);  
	Lcd_WriteData(0XA9); 

	Lcd_WriteIndex(0x3A);  
	Lcd_WriteData(0X55); 
	
  Lcd_WriteIndex(0x36);  
	#if USE_HORIZONTAL
		 Lcd_WriteData(0xE8);//横屏参数
	#else
		 Lcd_WriteData(0x48);//竖屏参数 
	#endif

	Lcd_WriteIndex(0xB1);  
	Lcd_WriteData(0X00); 
	Lcd_WriteData(0X18); 

	Lcd_WriteIndex(0xB4);  
	Lcd_WriteData(0X00); 
	Lcd_WriteData(0X00); 

	Lcd_WriteIndex(0xF2);  
	Lcd_WriteData(0X00); 

	Lcd_WriteIndex(0x26);  
	Lcd_WriteData(0X01); 

	Lcd_WriteIndex(0xE0);  
	Lcd_WriteData(0X0F); 
	Lcd_WriteData(0X17); 
	Lcd_WriteData(0X14); 
	Lcd_WriteData(0X09); 
	Lcd_WriteData(0X0C); 
	Lcd_WriteData(0X06); 
	Lcd_WriteData(0X43); 
	Lcd_WriteData(0X75); 
	Lcd_WriteData(0X36); 
	Lcd_WriteData(0X08); 
	Lcd_WriteData(0X13); 
	Lcd_WriteData(0X05); 
	Lcd_WriteData(0X10); 
	Lcd_WriteData(0X0B); 
	Lcd_WriteData(0X08); 


	Lcd_WriteIndex(0xE1);  
	Lcd_WriteData(0X00); 
	Lcd_WriteData(0X1F); 
	Lcd_WriteData(0X23); 
	Lcd_WriteData(0X03); 
	Lcd_WriteData(0X0E); 
	Lcd_WriteData(0X04); 
	Lcd_WriteData(0X39); 
	Lcd_WriteData(0X25); 
	Lcd_WriteData(0X4D); 
	Lcd_WriteData(0X06); 
	Lcd_WriteData(0X0D); 
	Lcd_WriteData(0X0B); 
	Lcd_WriteData(0X33); 
	Lcd_WriteData(0X37); 
	Lcd_WriteData(0X0F); 

	Lcd_WriteIndex(0x29);  	
}



/*************************************************
函数名：LCD_Set_XY
功能：设置lcd显示起始点
入口参数：xy坐标
返回值：无
*************************************************/
void Lcd_SetXY(uint16_t Xpos, uint16_t Ypos)
{	
	Lcd_WriteIndex(0x2A);
	Lcd_WriteData_16Bit(Xpos);
	Lcd_WriteIndex(0x2B);
	Lcd_WriteData_16Bit(Ypos);
	Lcd_WriteIndex(0x2c);	
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
	Lcd_WriteIndex(0x2A);
	Lcd_WriteData_16Bit(xStar);
	Lcd_WriteData_16Bit(xEnd);
	Lcd_WriteIndex(0x2B);
	Lcd_WriteData_16Bit(yStar);
	Lcd_WriteData_16Bit(yEnd);
	Lcd_WriteIndex(0x2c);
}

	
/*************************************************
函数名：LCD_DrawPoint
功能：画一个点
入口参数：xy坐标和颜色数据
返回值：无
*************************************************/
void RocLcdDrawPoint(uint16_t x, uint16_t y, uint16_t Data)
{
	Lcd_SetXY(x,y);
	Lcd_WriteData_16Bit(Data);
}

/*************************************************
函数名：Lcd_Clear
功能：全屏清屏函数
入口参数：填充颜色COLOR
返回值：无
*************************************************/
void Lcd_Clear(uint16_t Color)               
{	
   unsigned int i;
   Lcd_SetRegion(0,0,X_MAX_PIXEL-1,Y_MAX_PIXEL-1);
   LCD_RS_SET;	
   for(i=0;i<X_MAX_PIXEL*Y_MAX_PIXEL;i++)
   {	
	  	//Lcd_WriteData_16Bit(Color);
#if USE_HARDWARE_SPI   
		SPI_WriteByte(SPI2,Color>>8);
		SPI_WriteByte(SPI2,Color);
#else
		SPIv_WriteData(Color>>8);
		SPIv_WriteData(Color);
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
        RocLcdDrawPoint(X+a,Y+b,fc);     //        7 
        RocLcdDrawPoint(X-a,Y+b,fc);     //        6 
        RocLcdDrawPoint(X+a,Y-b,fc);     //        2 
        RocLcdDrawPoint(X-a,Y-b,fc);     //        3 
        RocLcdDrawPoint(X+b,Y+a,fc);     //        8 
        RocLcdDrawPoint(X-b,Y+a,fc);     //        5 
        RocLcdDrawPoint(X+b,Y-a,fc);     //        1 
        RocLcdDrawPoint(X-b,Y-a,fc);     //        4 

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
        RocLcdDrawPoint(X+a,Y+b,fc); 
        RocLcdDrawPoint(X+a,Y+b,fc); 
        RocLcdDrawPoint(X+a,Y-b,fc); 
        RocLcdDrawPoint(X-a,Y-b,fc); 
        RocLcdDrawPoint(X+b,Y+a,fc); 
        RocLcdDrawPoint(X-b,Y+a,fc); 
        RocLcdDrawPoint(X+b,Y-a,fc); 
        RocLcdDrawPoint(X-b,Y-a,fc); 
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
			RocLcdDrawPoint(x0,y0,Color);
			
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
			RocLcdDrawPoint(x0,y0,Color);

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
	Gui_DrawLine(x1,  y1,  x2,y1, GRAY2);  //H
	Gui_DrawLine(x1+1,y1+1,x2,y1+1, GRAY1);  //H
	Gui_DrawLine(x1,  y1,  x1,y2, GRAY2);  //V
	Gui_DrawLine(x1+1,y1+1,x1+1,y2, GRAY1);  //V
	Gui_DrawLine(x1,  y2,  x2,y2, WHITE);  //H
	Gui_DrawLine(x2,  y1,  x2,y2, WHITE);  //V
}

/**************************************************************************************
功能描述: 在屏幕显示一凹下的按钮框
输    入: uint16_t x1,y1,x2,y2 按钮框左上角和右下角坐标
输    出: 无
**************************************************************************************/
void DisplayButtonUp(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
	Gui_DrawLine(x1,  y1,  x2,y1, WHITE); //H
	Gui_DrawLine(x1,  y1,  x1,y2, WHITE); //V
	
	Gui_DrawLine(x1+1,y2-1,x2,y2-1, GRAY1);  //H
	Gui_DrawLine(x1,  y2,  x2,y2, GRAY2);  //H
	Gui_DrawLine(x2-1,y1+1,x2-1,y2, GRAY1);  //V
    Gui_DrawLine(x2  ,y1  ,x2,y2, GRAY2); //V
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
				    	if(asc16[k*16+i]&(0x80>>j))	RocLcdDrawPoint(x+j,y+i,fc);
						else 
						{
							if (fc!=bc) RocLcdDrawPoint(x+j,y+i,bc);
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
						    	if(hz16[k].Msk[i*2]&(0x80>>j))	RocLcdDrawPoint(x+j,y+i,fc);
								else {
									if (fc!=bc) RocLcdDrawPoint(x+j,y+i,bc);
								}
							}
						for(j=0;j<8;j++) 
							{
						    	if(hz16[k].Msk[i*2+1]&(0x80>>j))	RocLcdDrawPoint(x+j+8,y+i,fc);
								else 
								{
									if (fc!=bc) RocLcdDrawPoint(x+j+8,y+i,bc);
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
					RocLcdDrawPoint(x+j,y+i,fc);
					else 
					{
						if (fc!=bc) RocLcdDrawPoint(x+j,y+i,bc);
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
								RocLcdDrawPoint(x+j,y+i,fc);
								else 
								{
									if (fc!=bc) RocLcdDrawPoint(x+j,y+i,bc);
								}
							}
						for(j=0;j<8;j++) 
							{
						    	if(hz24[k].Msk[i*3+1]&(0x80>>j))	RocLcdDrawPoint(x+j+8,y+i,fc);
								else {
									if (fc!=bc) RocLcdDrawPoint(x+j+8,y+i,bc);
								}
							}
						for(j=0;j<8;j++) 
							{
						    	if(hz24[k].Msk[i*3+2]&(0x80>>j))	
								RocLcdDrawPoint(x+j+16,y+i,fc);
								else 
								{
									if (fc!=bc) RocLcdDrawPoint(x+j+16,y+i,bc);
								}
							}
				    }
			  }
			}
			s+=2;x+=24;
		}
	}
}
void Gui_DrawFont_Num32(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint16_t num)
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
	
		    	if(c&(0x80>>k))	RocLcdDrawPoint(x+j*8+k,y+i,fc);
				else {
					if (fc!=bc) RocLcdDrawPoint(x+j*8+k,y+i,bc);
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

	Lcd_Clear(GRAY0);
	
	Gui_DrawFont_GBK16(16,2,BLUE,GRAY0,"全动电子技术");
	Gui_DrawFont_GBK16(16,20,RED,GRAY0,"液晶测试程序");

	DisplayButtonUp(15,38,113,58); //x1,y1,x2,y2
	Gui_DrawFont_GBK16(16,40,GREEN,GRAY0,"颜色填充测试");

	DisplayButtonUp(15,68,113,88); //x1,y1,x2,y2
	Gui_DrawFont_GBK16(16,70,BLUE,GRAY0,"文字显示测试");

	DisplayButtonUp(15,98,113,118); //x1,y1,x2,y2
	Gui_DrawFont_GBK16(16,100,RED,GRAY0,"图片显示测试");;

	//Gui_DrawFont_GBK16(16,120,BLUE,GRAY0,"Welcome");
	Gui_DrawFont_GBK16(16,140,RED,GRAY0, "Welcome");
	
	Gui_DrawFont_Num32(100,125,RED,GRAY0,Num[5]);
	HAL_Delay(1000);
	Gui_DrawFont_Num32(100,125,RED,GRAY0,Num[4]);
	HAL_Delay(1000);
	Gui_DrawFont_Num32(100,125,RED,GRAY0,Num[3]);
	HAL_Delay(1000);
	Gui_DrawFont_Num32(100,125,RED,GRAY0,Num[2]);
	HAL_Delay(1000);
	Gui_DrawFont_Num32(100,125,RED,GRAY0,Num[1]);
	HAL_Delay(1000);
	Gui_DrawFont_Num32(100,125,RED,GRAY0,Num[0]);	
}
//测试数码管字体
void Num_Test(void)
{
	uint8_t i=0;
	Lcd_Clear(GRAY0);
	Gui_DrawFont_GBK16(16,20,RED,GRAY0,"Num Test");
	HAL_Delay(1000);
	Lcd_Clear(GRAY0);

	for(i=0;i<10;i++)
	{
	Gui_DrawFont_Num32((i%3)*40,32*(i/3)+30,RED,GRAY0,Num[i+1]);
	HAL_Delay(100);
	}
	
}
//中英文显示测试
void Font_Test(void)
{
	Lcd_Clear(GRAY0);
	Gui_DrawFont_GBK16(16,10,BLUE,GRAY0,"文字显示测试");

	HAL_Delay(1000);
	Lcd_Clear(GRAY0);
	Gui_DrawFont_GBK16(16,30,RED,GRAY0,"全动电子技术");
	Gui_DrawFont_GBK16(16,50,BLUE,GRAY0,"专注液晶批发");
	Gui_DrawFont_GBK16(16,70,RED,GRAY0, "全程技术支持");
	Gui_DrawFont_GBK16(0,100,BLUE,GRAY0,"Tel:15989313508");
	Gui_DrawFont_GBK16(0,130,RED,GRAY0, "www.qdtech.net");	
	HAL_Delay(1500);	
}
//简单刷屏测试
void Color_Test(void)
{
	uint8_t i=1;
	Lcd_Clear(GRAY0);
	
	Gui_DrawFont_GBK16(20,10,BLUE,GRAY0,"颜色填充测试");
	HAL_Delay(1200);

	while(i--)
	{
		Lcd_Clear(WHITE); HAL_Delay(500);
		Lcd_Clear(BLACK); HAL_Delay(500);
		Lcd_Clear(RED);	  HAL_Delay(500);
	  	Lcd_Clear(GREEN); HAL_Delay(500);
	  	Lcd_Clear(BLUE);  HAL_Delay(500);
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
	Lcd_Clear(GRAY0);
	Gui_DrawFont_GBK16(16,10,BLUE,GRAY0,"图片显示测试");
	HAL_Delay(1000);

	Lcd_Clear(GRAY0);
	for(k=0;k<Y_MAX_PIXEL/40;k++)
	{
	   	for(j=0;j<X_MAX_PIXEL/40;j++)
		{	
			Lcd_SetRegion(40*j,40*k,40*j+39,40*k+39);		//坐标设置
		    for(i=0;i<40*40;i++)
			 {	
			 	picL=*(p+i*2);	//数据低位在前
				picH=*(p+i*2+1);				
				Lcd_WriteData_16Bit(picH<<8|picL);  						
			 }	
		 }
	}		
}
//综合测试函数
static void QDTFT_Test_Demo(void)
{
	Lcd_Init();
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

    Lcd_Init();
    Lcd_Clear(BLACK);

    for(i = 0; i < 48; i++)
    {
        Gui_DrawLine(0, i * 5, 320, i * 5, RED);
    }

    if(RET_OK != Ret)
    {
        ROC_LOGE("Remote usart init is in error!");
    }

    return Ret;
}



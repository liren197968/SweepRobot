#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "usart.h"

#include "RocLog.h"


#pragma import(__use_no_semihosting)

FILE __stdout;

/*********************************************************************************
 *  Description:
 *              Printf system call function
 *
 *  Parameter:
 *              None
 *
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
**********************************************************************************/
struct __FILE
{
    int handle;
};

/*********************************************************************************
 *  Description:
 *              Printf system call function
 *
 *  Parameter:
 *              None
 *
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
*********************************************************************************/
void _sys_exit(int x)
{
    x = x;
}

/*********************************************************************************
 *  Description:
 *              Redefine the fputc function
 *
 *  Parameter:
 *              None
 *
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
**********************************************************************************/
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

    return ch;
}

/*********************************************************************************
 *  Description:
 *              Redefine the fgetc function
 *
 *  Parameter:
 *              None
 *
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
**********************************************************************************/
int fgetc(FILE *f)
{
    uint8_t ch = 0;

    HAL_UART_Receive(&huart1, &ch, 1, 0xFFFF);;

    return ch;
}

/*********************************************************************************
 *  Description:
 *              Normal log print, which will be used to show the normal log.
 *
 *  Parameter:
 *              None
 *
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
**********************************************************************************/
ROC_RESULT RocLogI(const char *function, uint32_t line, const char *fmt, ...)
{
    uint32_t    TickTime = 0;
    ROC_RESULT  Ret = RET_OK;
    va_list     arg_ptr = {NULL};
    char        Buffer[ROC_MAX_BUFFER_SIZE] = {0};

    va_start(arg_ptr, fmt);
    vsnprintf(Buffer, ROC_MAX_BUFFER_SIZE, fmt, arg_ptr);

    TickTime = HAL_GetTick();

    Ret = printf("[%010d] %s[%d]: %s \r\n", TickTime, function, line, Buffer);
    if(ROC_ZERO > Ret)
    {
        printf("Print is error!!! \r\n");
    }

    return Ret;
}

/*********************************************************************************
 *  Description:
 *              Green color log print, which will be used to show the warning log.
 *
 *  Parameter:
 *              None
 *
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
**********************************************************************************/
ROC_RESULT RocLogW(const char *function, uint32_t line, const char *fmt, ...)
{
    uint32_t    TickTime = 0;
    ROC_RESULT  Ret = RET_OK;
    va_list     arg_ptr = {NULL};
    char        Buffer[ROC_MAX_BUFFER_SIZE] = {0};

    va_start(arg_ptr, fmt);
    vsnprintf(Buffer, ROC_MAX_BUFFER_SIZE, fmt, arg_ptr);

    TickTime = HAL_GetTick();

    Ret = printf("[%010d] "ROC_FONT_GREEN"[WARN]%s[%d]: %s "ROC_CLOSE_PROPERTY" \r\n", TickTime, function, line, Buffer);
    if(ROC_ZERO > Ret)
    {
        printf("Print is error!!! \r\n");
    }

    return Ret;
}

/*********************************************************************************
 *  Description:
 *              Red log print, which will be used to show the error log.
 *
 *  Parameter:
 *              None
 *
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
**********************************************************************************/
ROC_RESULT RocLogE(const char *function, uint32_t line, const char *fmt, ...)
{
    uint32_t    TickTime = 0;
    ROC_RESULT  Ret = RET_OK;
    va_list     arg_ptr = {NULL};
    char        Buffer[ROC_MAX_BUFFER_SIZE] = {0};

    va_start(arg_ptr, fmt);
    vsnprintf(Buffer, ROC_MAX_BUFFER_SIZE, fmt, arg_ptr);

    TickTime = HAL_GetTick();

    Ret = printf("[%010d] "ROC_FONT_RED"[ERROR]%s[%d]: %s "ROC_CLOSE_PROPERTY" \r\n", TickTime, function, line, Buffer);
    if(ROC_ZERO > Ret)
    {
        printf("Print is error!!! \r\n");
    }

    return Ret;
}

/*********************************************************************************
 *  Description:
 *              Yellow log print, which will be used to show the notified log.
 *
 *  Parameter:
 *              None
 *
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
**********************************************************************************/
ROC_RESULT RocLogN(const char *function, uint32_t line, const char *fmt, ...)
{
    int32_t     Ret = RET_OK;
    uint32_t    TickTime = 0;
    va_list     arg_ptr;
    char        Buffer[ROC_MAX_BUFFER_SIZE] = {0};

    va_start(arg_ptr, fmt);
    vsnprintf(Buffer, ROC_MAX_BUFFER_SIZE, fmt, arg_ptr);

    TickTime = HAL_GetTick();

    Ret = printf("[%010d] "ROC_FONT_YELLOW"[NOTIFY]%s[%d]: %s "ROC_CLOSE_PROPERTY" \r\n", TickTime, function, line, Buffer);
    if(ROC_ZERO > Ret)
    {
        printf("Print is error!!! \r\n");
    }

    return Ret;
}



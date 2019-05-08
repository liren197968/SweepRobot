/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2018/12/20      1.0
********************************************************************************/
#include "usart.h"

#include "RocLog.h"
#include "RocBluetooth.h"


static uint8_t g_BtRecvEnd = ROC_FALSE;
static uint8_t g_BtRxDatLen = ROC_NONE;
static uint8_t g_BtTxBuffer[ROC_BT_TXD_LENGTH] = "Start";
static uint8_t g_BtRxBuffer[ROC_BT_RXD_LENGTH] = {ROC_NONE};


/*********************************************************************************
 *  Description:
 *              Bluetooth USART init
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              The init status
 *
 *  Author:
 *              ROC LiRen(2018.12.20)
**********************************************************************************/
ROC_RESULT RocBluetoothUsart_Init(void)
{
    ROC_RESULT Ret = RET_OK;

    if(HAL_OK != HAL_UART_Transmit_DMA(&huart3, g_BtTxBuffer, 5))
    {
        Ret = RET_ERROR;

        Error_Handler();
    }
    while(HAL_UART_STATE_READY != HAL_UART_GetState(&huart3));

    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    if(HAL_OK != HAL_UART_Receive_DMA(&huart3, g_BtRxBuffer, ROC_BT_RXD_LENGTH))
    {
        Ret = RET_ERROR;

        Error_Handler();
    }

    return Ret;
}

/*********************************************************************************
 *  Description:
 *              USART complete sending callback function
 *
 *  Parameter:
 *              None
 *
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.20)
**********************************************************************************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *Huart)
{
    if(USART2 == Huart->Instance)
    {
        //ROC_LOGI("Remote control send data successfully");
    }
    else if(USART3 == Huart->Instance)
    {
        //ROC_LOGI("Bluetooth send data successfully");
    }
}

/*********************************************************************************
 *  Description:
 *              USART complete receiving callback function
 *
 *  Parameter:
 *              None
 *
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.20)
**********************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *Huart)
{
    if(USART2 == Huart->Instance)
    {
        //ROC_LOGI("Remote control receive data successfully.");
    }else if(USART3 == Huart->Instance)
    {
        //ROC_LOGI("Bluetooth receive data successfully.");
    }
}

/*********************************************************************************
 *  Description:
 *              USART communication in error callback function
 *
 *  Parameter:
 *              None
 *
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.20)
**********************************************************************************/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *Huart)
{
    if(USART2 == Huart->Instance)
    {
        ROC_LOGE("Remote control data error!");
    }
    else if(USART3 == Huart->Instance)
    {
        ROC_LOGE("Bluetooth data error!");
    }
}
/*********************************************************************************
 *  Description:
 *              USART complete sending callback function
 *
 *  Parameter:
 *              None
 *
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.20)
**********************************************************************************/
void RocBluetoothReceiveCallback(UART_HandleTypeDef *Huart)
{
    uint32_t BufferFlag = RESET;
    uint32_t BufferData = RESET;

    if(USART3 == Huart->Instance)
    {
        BufferFlag = __HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE);

        if((BufferFlag != RESET))
        { 
            g_BtRecvEnd = ROC_TRUE;

            __HAL_UART_CLEAR_IDLEFLAG(&huart3);

            HAL_UART_DMAStop(&huart3);

            BufferData = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);

            g_BtRxDatLen = ROC_BT_RXD_LENGTH - BufferData;
            HAL_UART_Receive_DMA(&huart3, g_BtRxBuffer, ROC_BT_RXD_LENGTH);
        }
    }
}

/*********************************************************************************
 *  Description:
 *              Send data with bluetooth
 *
 *  Parameter:
 *              None
 *
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.20)
**********************************************************************************/
void RocBluetoothData_Send(uint8_t *Buff, uint16_t DatLen)
{
    ROC_RESULT Ret = RET_OK;

    while(HAL_UART_GetState(&huart3) == HAL_UART_STATE_BUSY_TX);

    Ret= HAL_UART_Transmit_DMA(&huart3, Buff, DatLen);
    if(HAL_OK != Ret)
    {
        ROC_LOGE("Remote usart transmission is in error(%d)!");
    
        Error_Handler();
    }

    //while(ROC_NONE != huart3.TxXferCount);
}

/*********************************************************************************
 *  Description:
 *              Set the bluetooth control command
 *
 *  Parameter:
 *              None
 *
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.20)
**********************************************************************************/
void RocBluetoothCtrlCmd_Set(uint8_t CtrlCmd)
{
    g_BtRxBuffer[0] = CtrlCmd;
}

/*********************************************************************************
 *  Description:
 *              Get the bluetooth control command
 *
 *  Parameter:
 *              None
 *
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.20)
**********************************************************************************/
uint8_t RocBluetoothCtrlCmd_Get(void)
{
    return g_BtRxBuffer[0];
}

/*********************************************************************************
 *  Description:
 *              Check bluetooth receive is finshed
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              The running state
 *
 *  Author:
 *              ROC LiRen(2018.12.15)
**********************************************************************************/
ROC_RESULT RocBluetoothRecvIsFinshed(void)
{
    if(g_BtRecvEnd == ROC_TRUE)
    {
        ROC_LOGI("Bluetooth receive (%d) data(%s).", g_BtRxDatLen, g_BtRxBuffer);

        RocBluetoothData_Send(g_BtRxBuffer, g_BtRxDatLen);

        //memset(g_BtRxBuffer, 0, g_BtRxDatLen);

        g_BtRecvEnd = ROC_FALSE;

        return ROC_TRUE;
    }
    else
    {
        return ROC_FALSE;
    }
}

/*********************************************************************************
 *  Description:
 *              Bluetooth module init
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              The init status
 *
 *  Author:
 *              ROC LiRen(2018.12.20)
**********************************************************************************/
ROC_RESULT RocBluetoothInit(void)
{
    ROC_RESULT Ret = RET_OK;

    Ret = RocBluetoothUsart_Init();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Bluetooth usart init is in error!");
    }

    if(RET_OK != Ret)
    {
        ROC_LOGE("Bluetooth module init is in error(%d)!", Ret);
    }
    else
    {
        ROC_LOGI("Bluetooth module init is in success.");
    }

    return Ret;
}


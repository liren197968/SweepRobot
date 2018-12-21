#include "usart.h"

#include "RocLog.h"
#include "RocBluetooth.h"


__IO ITStatus g_UsartReadyFlag = RESET;


uint8_t g_BtRecvEnd = ROC_FALSE;
uint8_t g_BtRxDatLen = ROC_NONE;
uint8_t g_BtTxBuffer[ROC_BT_TXD_LENGTH] = "Start";
uint8_t g_BtRxBuffer[ROC_BT_RXD_LENGTH];


/*********************************************************************************
 *  Description:
 *              Bluetooth USART init
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
    if(USART3 == Huart->Instance)
    {
        g_UsartReadyFlag = SET;

        ROC_LOGI("Bluetooth send data successfully.");
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
    if(USART3 == Huart->Instance)
    {
        ROC_LOGI("Bluetooth receive data successfully.");
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
    if(USART3 == Huart->Instance)
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

            BufferData = huart3.Instance->SR;
            BufferData = huart3.Instance->DR;
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
    if(HAL_OK != HAL_UART_Transmit_DMA(&huart3, Buff, DatLen))
    {
        Error_Handler();
    }

    while(SET != g_UsartReadyFlag);

    g_UsartReadyFlag = RESET;
}

/*********************************************************************************
 *  Description:
 *              Bluetooth module init
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
        ROC_LOGE("Bluetooth module init error(%d)!", Ret);
    }
    else
    {
        ROC_LOGI("Bluetooth module init is in success.");
    }

    return Ret;
}


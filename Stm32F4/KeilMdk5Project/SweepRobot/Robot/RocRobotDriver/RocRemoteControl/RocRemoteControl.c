/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 *  File            RemoteControl.c
 *  Author          Liren
 *  Version         1.0
 *  Data            2019/01/20
********************************************************************************/
#include "usb.h"
#include "usart.h"
#include "usb_host.h"
#include "usbh_hid.h"
#include "usbh_hid_parser.h"

#include "RocLog.h"
#include "RocRemoteControl.h"


static uint8_t g_RemoteRecvEnd = ROC_FALSE;
static uint8_t g_RemoteRxDatLen = ROC_NONE;
static uint8_t g_RemoteTxBuffer[ROC_REMOTE_MAX_NUM_LEN_SEND] = {ROC_NONE};
static uint8_t g_RemoteRxBuffer[ROC_REMOTE_MAX_NUM_LEN_SEND] = {ROC_NONE};


/*********************************************************************************
 *  Description:
 *              USB HOST HID mouse demo
 *
 *  Parameter:
 *              The USB host handle type define
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.01.20)
**********************************************************************************/
static void RocUSBHMouseDemoRun(USBH_HandleTypeDef *phost)
{
    HID_MOUSE_Info_TypeDef *m_pinfo;
    static uint8_t button_state[3] = {0};

    m_pinfo = USBH_HID_GetMouseInfo(phost);
    if(m_pinfo != NULL)
    {
        /* Handle Mouse data position */
        USR_MOUSE_ProcessData(&mouse_info);

        if(m_pinfo->buttons[0])
        {
            button_state[0] = 1;
            HID_MOUSE_ButtonPressed(0);
        }
        else if(button_state[0])
        {
            button_state[0] = 0;
            HID_MOUSE_ButtonReleased(0);
        }

        if(m_pinfo->buttons[1])
        {
            button_state[1] = 1;
            HID_MOUSE_ButtonPressed(1);
        }
        else if(button_state[1])
        {
            button_state[1] = 0;
            HID_MOUSE_ButtonReleased(1);
        }

        if(m_pinfo->buttons[2])
        {
            button_state[2] = 1;
            HID_MOUSE_ButtonPressed(2);
        }
        else if(button_state[2])
        {
            button_state[2] = 0;
            HID_MOUSE_ButtonReleased(2);
        }
    }
}

/*********************************************************************************
 *  Description:
 *              USB HOST HID application process run
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.01.20)
**********************************************************************************/
static void RocUsbHostHidProcess(void)
{
    if(Appli_state == APPLICATION_READY)
    {
        HID_MouseMenuProcess();

        RocUSBHMouseDemoRun(&hUsbHostFS);
    }
}

/*********************************************************************************
 *  Description:
 *              USB HOST HID application init
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.01.20)
**********************************************************************************/
static void RocHidApplyInit(void)
{
    HID_TypeTypeDef   DeviceType = HID_UNKNOWN;

    ROC_LOGN("ST Middlewares USB_Host_library started init.");

    HID_MenuInit();

    while(APPLICATION_READY != Appli_state)
    {
        MX_USB_HOST_Process();
    }

    while(HID_MOUSE != DeviceType)
    {
        DeviceType = USBH_HID_GetDeviceType(&hUsbHostFS);
    }
}

/*********************************************************************************
 *  Description:
 *              Init remote USB control
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.20)
**********************************************************************************/
static void RocRemoteUsbControlInit(void)
{
    HID_TypeTypeDef   DeviceType = HID_UNKNOWN;

    ROC_LOGN("ST Middlewares USB_Host_library started init");

    HID_MenuInit();

    while(APPLICATION_READY != Appli_state)
    {
        MX_USB_HOST_Process();
    }

    while(HID_MOUSE != DeviceType)
    {
        DeviceType = USBH_HID_GetDeviceType(&hUsbHostFS);
    }
}

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
ROC_RESULT RocRemoteTransUsartInit(void)
{
    ROC_RESULT Ret = RET_OK;

    if(HAL_OK != HAL_UART_Transmit_DMA(&huart2, g_RemoteTxBuffer, ROC_REMOTE_MAX_NUM_LEN_SEND))
    {
        Ret = RET_ERROR;

        Error_Handler();
    }
    while(HAL_UART_STATE_READY != HAL_UART_GetState(&huart2));

    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

    if(HAL_OK != HAL_UART_Receive_DMA(&huart2, g_RemoteRxBuffer, ROC_REMOTE_MAX_NUM_LEN_SEND))
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
void RocRemoteReceiveCallback(UART_HandleTypeDef *Huart)
{
    uint32_t BufferFlag = RESET;
    uint32_t BufferData = RESET;

    if(USART2 == Huart->Instance)
    {
        BufferFlag = __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE);

        if((BufferFlag != RESET))
        { 
            g_RemoteRecvEnd = ROC_TRUE;

            __HAL_UART_CLEAR_IDLEFLAG(&huart2);

            HAL_UART_DMAStop(&huart2);

            BufferData = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

            g_RemoteRxDatLen = ROC_REMOTE_MAX_NUM_LEN_SEND - BufferData;
            HAL_UART_Receive_DMA(&huart2, g_RemoteRxBuffer, ROC_REMOTE_MAX_NUM_LEN_SEND);
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
void RocRemoteDataTransmit(uint8_t *Buff, uint16_t DatLen)
{
    ROC_RESULT Ret = RET_OK;

    Ret = HAL_UART_Transmit_DMA(&huart2, Buff, DatLen);
    if(HAL_OK != Ret)
    {
        ROC_LOGE("Remote usart transmission is in error(%d)!");

        Error_Handler();
    }

    while(ROC_NONE != huart2.TxXferCount);
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
ROC_RESULT RocRemoteRecvIsFinshed(void)
{
    if(g_RemoteRecvEnd == ROC_TRUE)
    {
        ROC_LOGI("Remote receive (%d) data(%s).", g_RemoteRxDatLen, g_RemoteRxBuffer);

        g_RemoteRecvEnd = ROC_FALSE;

        return ROC_TRUE;
    }
    else
    {
        return ROC_FALSE;
    }
}

/*********************************************************************************
 *  Description:
 *              Double data to string data
 *
 *  Parameter:
 *              DoublueData: the double data
 *              pString:     the pointer to the string memory
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.04.20)
**********************************************************************************/
static ROC_RESULT RocDoubleDatToStringDat(double DoubleData, uint8_t *pString)
{
    uint8_t         i = 0;
    double          DecimalData;
    int32_t         IntegerData;

    IntegerData = (int32_t)DoubleData;
    DecimalData = DoubleData - IntegerData;

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
 *              Send duoble data to PC serial
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.01.20)
**********************************************************************************/
static ROC_RESULT RocDoubleDatSendtoPc(double DoubleData)
{
    ROC_RESULT  Ret = RET_OK;
    uint8_t     TempStr[ROC_REMOTE_MAX_NUM_LEN_SEND];

    Ret = RocDoubleDatToStringDat(DoubleData, TempStr);
    if(RET_OK == Ret)
    {
        RocRemoteDataTransmit(TempStr, ROC_REMOTE_MAX_NUM_LEN_SEND);

        return RET_OK;
    }
    else
    {
        return RET_ERROR;
    }
}

/*********************************************************************************
 *  Description:
 *              Remote control init
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.01.20)
**********************************************************************************/
ROC_RESULT RocRemoteControlInit(void)
{
    ROC_RESULT Ret = RET_OK;

    Ret = RocRemoteTransUsartInit();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Remote usart init is in error!");
    }

    return Ret;
}


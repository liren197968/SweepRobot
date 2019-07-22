/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 *  File            RemoteControl.c
 *  Author          Liren
 *  Version         1.0
 *  Data            2019/01/20
********************************************************************************/
#ifdef ROC_REMOTE_USB_CONTROL
#include "usb.h"
#include "usb_host.h"
#include "usbh_hid.h"
#include "usbh_hid_parser.h"
#endif

#include <string.h>

#include "usart.h"

//#include "lora_ptp.h"

#include "RocLog.h"
#include "RocLed.h"
#include "RocRemoteControl.h"


static uint8_t g_RemoteRecvEnd = ROC_FALSE;
static uint8_t g_RemoteRxDatLen = ROC_NONE;
static uint8_t g_RemoteTxBuffer[ROC_REMOTE_MAX_NUM_LEN_SEND] = {0xFA, 0x00, 0x00, 0x00};
static uint8_t g_RemoteRxBuffer[ROC_REMOTE_MAX_NUM_LEN_SEND] = {ROC_NONE};


#ifdef ROC_REMOTE_USB_CONTROL
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
#endif

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
static ROC_RESULT RocRemoteTransUsartInit(void)
{
    ROC_RESULT Ret = RET_OK;

    if(HAL_OK != HAL_UART_Transmit_DMA(ROC_REMOTE_UART_CHANNEL, g_RemoteTxBuffer, ROC_REMOTE_MAX_NUM_LEN_SEND))
    {
        Ret = RET_ERROR;

        Error_Handler();
    }

    __HAL_UART_ENABLE_IT(ROC_REMOTE_UART_CHANNEL, UART_IT_IDLE);

    if(HAL_OK != HAL_UART_Receive_DMA(ROC_REMOTE_UART_CHANNEL, g_RemoteRxBuffer, ROC_REMOTE_MAX_NUM_LEN_SEND))
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
        BufferFlag = __HAL_UART_GET_FLAG(ROC_REMOTE_UART_CHANNEL, UART_FLAG_IDLE);

        if((BufferFlag != RESET))
        { 
            g_RemoteRecvEnd = ROC_TRUE;

            __HAL_UART_CLEAR_IDLEFLAG(ROC_REMOTE_UART_CHANNEL);

            HAL_UART_DMAStop(ROC_REMOTE_UART_CHANNEL);

            BufferData = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

            g_RemoteRxDatLen = ROC_REMOTE_MAX_NUM_LEN_SEND - BufferData;
            HAL_UART_Receive_DMA(ROC_REMOTE_UART_CHANNEL, g_RemoteRxBuffer, ROC_REMOTE_MAX_NUM_LEN_SEND);
        }
    }
}

/*********************************************************************************
 *  Description:
 *              Send data with remote control
 *
 *  Parameter:
 *              *Buff: the pointer to send buffer
 *              DatLen: the length of send buffer
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

    //while(HAL_UART_GetState(ROC_REMOTE_UART_CHANNEL) != HAL_UART_STATE_READY);

    Ret = HAL_UART_Transmit_DMA(ROC_REMOTE_UART_CHANNEL, Buff, DatLen);
    if(HAL_OK != Ret)
    {
        ROC_LOGE("Remote usart transmission is in error(%d)!", Ret);
    }

    while(ROC_NONE != huart2.TxXferCount);
}

/*********************************************************************************
 *  Description:
 *              Receive data with remote control
 *
 *  Parameter:
 *              DatLen: the length of receive buffer
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2018.12.20)
**********************************************************************************/
uint8_t* RocRemoteDataReceive(void)
{
//    ROC_RESULT Ret = RET_OK;

//    Ret = HAL_UART_Receive_DMA(ROC_REMOTE_UART_CHANNEL, g_RemoteRxBuffer, DatLen);
//    if(HAL_OK != Ret)
//    {
//        ROC_LOGE("Remote usart receive is in error(%d)!", Ret);
//    }

//    while(ROC_TRUE != RocRemoteRecvIsFinshed());

    return g_RemoteRxBuffer;
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
        //ROC_LOGI("Remote receive (%d) data(%s).", g_RemoteRxDatLen, g_RemoteRxBuffer);

        g_RemoteRecvEnd = ROC_FALSE;

        return ROC_TRUE;
    }
    else
    {
        return ROC_FALSE;
    }
}

#if 0
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
 *              ROC LiRen(2019.07.19)
**********************************************************************************/
#define CMD_SIZE    255
#define AT_CMD_TIME_OUT   50

static char command[CMD_SIZE];
static volatile unsigned cmd_index = 0;
FlagStatus IsCmdReceived = RESET;
static uint32_t cmd_rx_time_out = 0;

static void CMD_GetChar( uint8_t* rxChar)
{
    cmd_index %= CMD_SIZE;
    command[cmd_index] = *rxChar;
    cmd_rx_time_out = AT_CMD_TIME_OUT;
    cmd_index++;
}

void StartTransTxRxTask(void)
{
    uint8_t* prxchar = NULL;
    uint16_t len   = 0;
    int8_t   snr = 0;
    int16_t  rssi = 0;
    uint8_t* pchar = NULL;
    uint8_t sf = 0 ;
	uint8_t   current_sf = 0;

    //vcom_ReceiveInit(CMD_GetChar);

    LORA_ptop_config(0x12, 20);

    SX1276SetOpMode( RF_OPMODE_SLEEP );

    RocLedTurnOn(ROC_LED_2);
    RocLedTurnOff(ROC_LED_1);

    for(len = 0; len < CMD_SIZE; len++)
    {
        command[len] = len;
    }

    while(1)
    {

        if (IsCmdReceived == SET)
        {

            if(cmd_index > CMD_SIZE)
            {
                cmd_index = CMD_SIZE;
            }

//			sf = BSP_PB_GetState(BUTTON_MODE);

//			if(sf == 0)
//			{
//				sf = 7;
//			}
//			else
//			{
//				sf = 12;
//			}
            sf = 7;

            RocLedTurnOff(ROC_LED_1);
            cmd_index = CMD_SIZE;
            LORA_ptop_SendMsg(sf, TX_PWR, (uint8_t*)&command[0], cmd_index );
            RocLedTurnOn(ROC_LED_1);

            cmd_index = 0 ;
            IsCmdReceived = RESET;
            memset(command, 0, sizeof(command));
        }
        else
        {
            LORA_ptop_SetInRxMode(0);

            pchar = LORA_ptop_ReceiveMsg(&prxchar, &len, &rssi, &snr, 5);

            if(pchar != 0)
            {

                RocLedTurnOff(ROC_LED_2);
                HAL_Delay(400);
                RocLedTurnOn(ROC_LED_2);

                if(len > CMD_SIZE)
                {
                    len = CMD_SIZE;
                }

                vcom_send_data(pchar, len);
                ROC_LOGN("The receive data is %s", pchar);

                //free(pchar);
            }
        }

        HAL_Delay(10);
    }
}
#endif
static uint8_t RocRobotJoystickCmdGet(void)
{
    uint8_t *RemoteData = NULL;

    RemoteData = RocRemoteDataReceive();

    if(ROC_JOYSTICK_FRAME_HEADER != RemoteData[0])
    {
        return ROC_NONE;
    }
    else if(ROC_JOYSTICK_KEY_HEADER != RemoteData[1])
    {
        return ROC_NONE;
    }
    else
    {
        return RemoteData[3];
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
    uint16_t Dat[4];
#ifdef ROC_REMOTE_USB_CONTROL
    RocRemoteUsbControlInit();
#endif

    Ret = RocRemoteTransUsartInit();
    if(RET_OK != Ret)
    {
        ROC_LOGE("Remote usart init is in error!");
    }

    return Ret;
}


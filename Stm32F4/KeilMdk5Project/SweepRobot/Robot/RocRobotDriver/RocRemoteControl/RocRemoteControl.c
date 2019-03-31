/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 *  File            RemoteControl.c
 *  Author          Liren
 *  Version         1.0
 *  Data            2019/01/20
********************************************************************************/
#include "usb.h"
#include "usb_host.h"
#include "usbh_hid.h"
#include "usbh_hid_parser.h"

#include "RocLog.h"
#include "RocRemoteControl.h"

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

    RocHidApplyInit();

    ROC_LOGW("USB HOST HID device connect success.");

    while(1)
    {
        MX_USB_HOST_Process();

        RocUsbHostHidProcess();
    }

    return Ret;
}


/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/01/20      1.0
********************************************************************************/
#include "stm32f4xx_hal.h"

#include "usart.h"

#include "RocLog.h"
#include "RocCh375.h"


#define CH375_DBG

#define p_dev_descr ((PUSB_DEV_DESCR)RECV_BUFFER)
#define p_cfg_descr ((PUSB_CFG_DESCR_LONG)RECV_BUFFER)


int16_t stallCount;
int8_t usbtype;

/**
 * Class: MeUSBHost
 * \par Description
 * Declaration of Class MeUSBHost.
 */

  bool ch375_online;
  bool device_online;
  bool device_ready;
  uint8_t RECV_BUFFER[ CH375_MAX_DATA_LEN ];


uint8_t endp_out_addr;
uint8_t endp_out_size;
uint8_t endp_in_addr;
uint8_t endp6_mode, endp7_mode;

uint8_t *cmd_buf;
uint8_t *ret_buf;
PUSB_ENDP_DESCR tmpEp = {0};

CH375_USB_DEF   CH375_USB_Info;



/**
 * \par Function
 *    CH375_RD
 * \par Description
 *    Read data from USB.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    Return 0;
 * \par Others
 *    None
 */
uint8_t CH375_RD(void)
{
  HAL_Delay(2); // stupid HAL_Delay, the chip don't got any buffer
  //if(HAL_UART_STATE_READY != HAL_UART_GetState(&huart2))
 {
  uint8_t c = 0;
  HAL_StatusTypeDef ReadStatus;
  ReadStatus = HAL_UART_Receive(&huart2, &c, 1, 1000);
    if(HAL_OK != ReadStatus)
    {
        ROC_LOGI("Read data is in error(%d)!", ReadStatus);
        while(1);
    }
  //c = Bsp_CH375_Read_Data();
#ifdef CH375_DBG
    ROC_LOGI("<< 0x%x", c);
#endif
    return c;
  }
  return 0;
}

/**
 * \par Function
 *    CH375_WR
 * \par Description
 *    Write data to USB device.
 * \param[in]
 *    c - The bytes that wrote to device. 
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void CH375_WR(uint8_t c)
{
  uint16_t d = 0;
  HAL_StatusTypeDef WriteStatus;
  d = c | 0x100;
  WriteStatus = HAL_UART_Transmit(&huart2, (uint8_t *)&c, 1, 1000);
  if(HAL_OK != WriteStatus)
  {
      ROC_LOGI("Write data is in error(%d)!", WriteStatus);
      while(1);
  }

  //Bsp_CH375_Write_Data(c);
  HAL_Delay(2);
#ifdef CH375_DBG
  ROC_LOGI(">> 0x%x", c);
#endif
}

void CH375_WR_Data(uint8_t c)
{
  uint16_t d = 0;
  HAL_StatusTypeDef WriteStatus;
  d = c & 0xFF;
  WriteStatus = HAL_UART_Transmit(&huart2, (uint8_t *)&c, 1, 1000);
  if(HAL_OK != WriteStatus)
  {
      ROC_LOGI("Write data is in error(%d)!", WriteStatus);
      while(1);
  }
  HAL_Delay(2);
#ifdef CH375_DBG
  ROC_LOGI(">> 0x%x", c);
#endif
}

/**
 * \par Function
 *    set_usb_mode
 * \par Description
 *    Set the work mode of USB.
 * \param[in]
 *    mode - The USB's work mode. 
 * \par Output
 *    None
 * \return
 *    Return the data that CH375_RD()'s return.
 * \par Others
 *    None
 */
int16_t set_usb_mode(int16_t mode)
{
  CH375_WR(CMD_SET_USB_MODE);
  CH375_WR_Data(mode);
  endp6_mode=endp7_mode=0x80;
  return CH375_RD();
}

/**
 * \par Function
 *    getIrq
 * \par Description
 *    Get the Interrupt Request of USB.
 * \param[in]
 *    None 
 * \par Output
 *    None
 * \return
 *    Return the data that CH375_RD()'s return.
 * \par Others
 *    None
 */
uint8_t getIrq()
{
    ROC_LOGW("Wait for USB interrupt...");
    while(GPIO_PIN_SET == HAL_GPIO_ReadPin(CH375_INT_GPIO_Port, CH375_INT_Pin));
  CH375_WR(CMD_GET_STATUS);
  HAL_Delay(20);
  return CH375_RD();
}

/**
 * \par Function
 *    toggle_send
 * \par Description
 *    The toggle used to send data.
 * \param[in]
 *    None 
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void toggle_send()
{
#ifdef CH375_DBG
  ROC_LOGI("toggle send: 0x%x", endp7_mode);
#endif
  CH375_WR(CMD_SET_ENDP7);
  CH375_WR_Data( endp7_mode );
  endp7_mode^=0x40;
}

/**
 * \par Function
 *    toggle_recv
 * \par Description
 *    The toggle used to receive data.
 * \param[in]
 *    None 
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void toggle_recv()
{
  CH375_WR( CMD_SET_ENDP6 );
  CH375_WR_Data( endp6_mode );
#ifdef CH375_DBG
  ROC_LOGI("toggle recv: 0x%x", endp6_mode);
#endif
  endp6_mode^=0x40;
}

/**
 * \par Function
 *    issue_token
 * \par Description
 *    USB Host make a token and perform transactions.
 * \param[in]
 *    endp_and_pid - The token that USB Host used.
 * \par Output
 *    None
 * \return
 *    Return the Interrupt Request.
 * \par Others
 *    None
 */
uint8_t issue_token( uint8_t endp_and_pid )
{
  CH375_WR( CMD_ISSUE_TOKEN );
  CH375_WR_Data( endp_and_pid );  /* Bit7~4 for EndPoint No, Bit3~0 for Token PID */
#ifdef CH375_DBG
  ROC_LOGI("issue token: 0x%x", endp_and_pid);
#endif
  HAL_Delay(2);
  return getIrq();
}

/**
 * \par Function
 *    wr_usb_data
 * \par Description
 *    Write data to USB Host.
 * \param[in]
 *    len - The data's length.
  * \param[in]
 *    buf - Data in buffer.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void wr_usb_data( uint8_t len, uint8_t *buf )
{
#ifdef CH375_DBG
  ROC_LOGI("issue token: %d", len);
#endif
  CH375_WR( CMD_WR_USB_DATA7 );
  CH375_WR_Data( len );
  while( len-- ){
    CH375_WR_Data( *buf++ );
  }
}

/**
 * \par Function
 *    rd_usb_data
 * \par Description
 *    Read data from USB Host.
 * \param[in]
 *    len - The data's length.
 * \param[in]
 *    buf - Data in buffer.
 * \par Output
 *    None
 * \return
 *    Return the length of read data.
 * \par Others
 *    None
 */
uint8_t rd_usb_data( uint8_t *buf )
{
  uint8_t i, len;
    HAL_StatusTypeDef ReadStatus;
  CH375_WR( CMD_RD_USB_DATA );
  len=CH375_RD();
#ifdef CH375_DBG
  ROC_LOGI("usb rd: %d", len);
#endif
//    ReadStatus = HAL_UART_Receive(&huart2, buf, len, 1000);
//    if(HAL_OK != ReadStatus)
//    {
//        ROC_LOGE("Read data from USB bus is in error(%d)!", ReadStatus);
//        while(1);
//    }
  for ( i=0; i!=len; i++ ) *buf++=CH375_RD();
  return( len );
}

/**
 * \par Function
 *    get_version
 * \par Description
 *    Get version of USB Host.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    Return the data that CH375_RD()'s return.
 * \par Others
 *    None
 */
int16_t get_version()
{
  CH375_WR(CMD_GET_IC_VER);
  return CH375_RD();
}

/**
 * \par Function
 *    set_freq
 * \par Description
 *    Set frequency of USB Host.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void set_freq(void)
{
    CH375_WR(CMD_SET_USB_SPEED);
    CH375_WR_Data(0x00);
    getIrq();
/*
  CH375_WR(0x0b);
  CH375_WR(0x17);
  CH375_WR(0xd8);
*/
}

/**
 * \par Function
 *    set_addr
 * \par Description
 *    Set address of USB Host.
 * \param[in]
 *    addr - The address of USB Host.
 * \par Output
 *    None
 * \return
 *    Return the number of Interrupt Request.
 * \par Others
 *    None
 */
uint8_t set_addr( uint8_t addr )
{
  uint8_t irq;
  CH375_WR(CMD_SET_ADDRESS);
  CH375_WR_Data(addr);
  irq = getIrq();
  if(irq==USB_INT_SUCCESS){
    CH375_WR(CMD_SET_USB_ADDR);
    CH375_WR_Data(addr);
  }
  return irq;
}

/**
 * \par Function
 *    set_config
 * \par Description
 *    Set config of USB Host.
 * \param[in]
 *    cfg - The config file of USB Host.
 * \par Output
 *    None
 * \return
 *    Return the number of Interrupt Request.
 * \par Others
 *    None
 */
uint8_t set_config(uint8_t cfg)
{
  endp6_mode=endp7_mode=0x80; // reset the sync flags
  CH375_WR(CMD_SET_CONFIG);
  CH375_WR_Data(cfg);
  return getIrq();
}

/**
 * \par Function
 *    clr_stall6
 * \par Description
 *    Clear all stall in USB Host.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    Return the number of Interrupt Request.
 * \par Others
 *    None
 */
uint8_t clr_stall6(void)
{
  CH375_WR( CMD_CLR_STALL );
  CH375_WR_Data( endp_out_addr | 0x80 );
  endp6_mode=0x80;
  return getIrq();
}

/**
 * \par Function
 *    get_desr
 * \par Description
 *    Get description of USB Host.
 * \param[in]
 *    type - The type of description.
 * \par Output
 *    None
 * \return
 *    Return the number of Interrupt Request.
 * \par Others
 *    None
 */
uint8_t get_desr(uint8_t type)
{
  CH375_WR( CMD_GET_DESCR );
  CH375_WR_Data( type );   /* description type, only 1(device) or 2(config) */
  return getIrq();
}

/**
 * \par Function
 *    host_recv
 * \par Description
 *    The USB Host receive data.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    Return the length of data.
 * \par Others
 *    None
 */
uint8_t host_recv()
{
  uint8_t irq;
  toggle_recv();
  irq = issue_token( ( endp_in_addr << 4 ) | DEF_USB_PID_IN );
  if(irq==USB_INT_SUCCESS)
{
    int16_t len = rd_usb_data(RECV_BUFFER);
#ifdef CH375_DBG
    for(int16_t i=0;i<len;i++){
      // point hid device
      ROC_LOGI("0x%x", (int16_t)RECV_BUFFER[i]);
    }
#endif
    stallCount = 0;
    return len;
  }else if(irq==USB_INT_DISCONNECT){
    device_online = false;
    device_ready = false;
#ifdef CH375_DBG
    ROC_LOGI("##### disconn ##### \r\n");
    ROC_LOGI("\r\n");
#endif
    return 0;
  }else{
    clr_stall6();
#ifdef CH375_DBG
    ROC_LOGI("##### stall ##### \r\n");
#endif
    HAL_Delay(10);
    /*
    stallCount++;
    if(stallCount>10){
      device_online = false;
      device_ready = false;
      resetBus();
    }
    */
    return 0;
  }
}

/**
 * \par Function
 *    resetBus
 * \par Description
 *    Reset the data Bus.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void resetBus()
{
  int16_t c;
  c = set_usb_mode(7);
#ifdef CH375_DBG
  ROC_LOGI("set mode 7: 0x%d", c);
#endif
  HAL_Delay(10);
  c = set_usb_mode(6);
#ifdef CH375_DBG
  ROC_LOGI("set mode 6: 0x%d", c);
#endif
  HAL_Delay( 100 );
  while ( getIrq()!=USB_INT_CONNECT );
}

/**
 * \par Function
 *    init
 * \par Description
 *    Init the data Bus.
 * \param[in]
 *    type - The type of data Bus.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void init(int8_t type)
{
  ch375_online = false;
  device_online = false;
  device_ready = false;
  usbtype = type;
}


/**
 * \par Function
 *    initHIDDevice
 * \par Description
 *    Init the HID Device.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    The result of initHIDDevice's action.
 * \par Others
 *    None
 */
int16_t initHIDDevice()
{
  int16_t irq, len, address;

  if(usbtype==USB1_0) set_freq(); //work on a lower freq, necessary for ch375
  ROC_LOGI("start getting description");
  irq = get_desr(1);
#ifdef CH375_DBG
  ROC_LOGI("get des irq: 0x%x", irq);
#endif
  if(irq==USB_INT_SUCCESS){
          ROC_LOGI("start reading description");
      len = rd_usb_data( RECV_BUFFER );
#ifdef CH375_DBG
      ROC_LOGI("descr1 len: %d", len);

      ROC_LOGI(" type: 0x%x", p_dev_descr->bDescriptorType);
#endif
        ROC_LOGI("start setting address");
      irq = set_addr(3);
      if(irq==USB_INT_SUCCESS){
        irq = get_desr(2); // max buf 64byte, todo:config descr overflow
        if(irq==USB_INT_SUCCESS){
          len = rd_usb_data( RECV_BUFFER );
#ifdef CH375_DBG
          ROC_LOGI("descr1 len: 0x%x", len);

          ROC_LOGI(" class: 0x%x", p_cfg_descr->itf_descr.bInterfaceClass);
          ROC_LOGI(" subclass: 0x%x", p_cfg_descr->itf_descr.bInterfaceSubClass);

          ROC_LOGI("num of ep: %d", p_cfg_descr->itf_descr.bNumEndpoints);

          ROC_LOGI("ep0: 0x:%x", p_cfg_descr->endp_descr[0].bLength);
          ROC_LOGI("type: 0x%x", p_cfg_descr->endp_descr[0].bDescriptorType);
#endif
          if(p_cfg_descr->endp_descr[0].bDescriptorType==0x21){ // skip hid des
            tmpEp = (PUSB_ENDP_DESCR)((int8_t*)(&(p_cfg_descr->endp_descr[0]))+p_cfg_descr->endp_descr[0].bLength); // get the real ep position
          }
#ifdef CH375_DBG
          ROC_LOGI("endpoint: 0x%x", tmpEp->bEndpointAddress);
          ROC_LOGI("descriptorType: 0x%x", tmpEp->bDescriptorType);
#endif
          endp_out_addr=endp_in_addr=0;
          address =tmpEp->bEndpointAddress;  /* Address of First EndPoint */
          // actually we only care about the input end points
          if( address&0x80 ){
            endp_in_addr = address&0x0f;  /* Address of IN EndPoint */
          }else{  /* OUT EndPoint */
            endp_out_addr = address&0x0f;
            endp_out_size = p_cfg_descr->endp_descr[0].wMaxPacketSize;
			/* Length of Package for Received Data EndPoint */
            if( endp_out_size == 0 || endp_out_size > 64 )
              endp_out_size = 64;
          }
          // todo: some joystick with more than 2 node
          // just assume every thing is fine, bring the device up
          irq = set_config(p_cfg_descr->cfg_descr.bConfigurationvalue);
          if(irq==USB_INT_SUCCESS){
            CH375_WR( CMD_SET_RETRY );  // set the retry times
            CH375_WR_Data( 0x25 );
            CH375_WR_Data( 0x85 );
            device_ready = true;
            return 1;
          }
        }

      }
  }
  return 0;
}

/**
 * \par Function
 *    probeDevice
 * \par Description
 *    Prode of USB Host Device.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    The result of device's probe.
 * \par Others
 *    None
 */
int16_t probeDevice()
{
  int16_t c;
  if(!ch375_online){
    CH375_WR( CMD_CHECK_EXIST );
    CH375_WR_Data( 0x5A);
    c = CH375_RD(); // should return 0xA5
    if(c!=0xA5) return 0;
    ch375_online = true;
    resetBus();
  }

  c = getIrq();
  if(c!=USB_INT_CONNECT) return 0;
  resetBus(); // reset bus and wait the device online again
  c=0;
  while(c!=USB_INT_CONNECT){
    HAL_Delay(500); // some device may need long time to get ready
    c = getIrq();
#ifdef CH375_DBG
    ROC_LOGI("waiting: %x", c);
#endif
  }
    HAL_Delay(2000);
  if( initHIDDevice()==1)
    device_online=true;

  return c;
}

void parseJoystick(unsigned char *buf)   //Analytic function, print 8 bytes from USB Host
{
  int i = 0;
  for(i = 0; i < 7; i++)
  {
    ROC_LOGI("%d", buf[i]);  //It won't work if you connect to the Makeblock Orion.
  }
  ROC_LOGI("%d\r\n", buf[7]);
  HAL_Delay(10);
}

/*********************************************************************************
 *  Description:
 *              Set the robot walk mode
 *
 *  Parameter:
 *              WalkMode: the expected robot walk mode
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.01.20)
**********************************************************************************/
ROC_RESULT RocCh375Init(void)
{
    ROC_RESULT Ret = RET_OK;

    Bsp_GPIO_Init();

    init(USB1_0);

    HAL_Delay( 200 );
    set_usb_mode( 6 );
    while ( getIrq()!=USB_INT_CONNECT );

    while(1)
    {
        if(!device_online)
        {
            probeDevice();
            HAL_Delay(1000);
            ROC_LOGN("USB HID device is not online!");
        }
        else
        {
            int len = host_recv();
            if(len)
            {
                parseJoystick(RECV_BUFFER);
            }
            HAL_Delay(1000);
        }
    }

    return Ret;
}

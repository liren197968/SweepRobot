#if defined(TARGET_NUCLEO_F401RE)||defined(TARGET_NUCLEO_F411RE)||defined(TARGET_NUCLEO_F446RE)
#include "USBHALHost.h"
#include <algorithm>

#ifdef _USB_DBG
extern RawSerial pc;
//RawSerial pc(USBTX,USBRX);
#include "mydebug.h"
#define USB_DBG(...) do{pc.printf("[%s@%d] ",__PRETTY_FUNCTION__,__LINE__);pc.printf(__VA_ARGS__);pc.puts("\n");} while(0);
#define USB_DBG_HEX(A,B) debug_hex<RawSerial>(pc,A,B)

#else
#define USB_DBG(...) while(0)
#define USB_DBG_HEX(A,B) while(0)
#endif

#undef USB_TEST_ASSERT
void usb_test_assert_internal(const char *expr, const char *file, int line);
#define USB_TEST_ASSERT(EXPR) while(!(EXPR)){usb_test_assert_internal(#EXPR,__FILE__,__LINE__);}

#define USB_TRACE1(A) while(0)

#define USB_INFO(...) do{fprintf(stderr,__VA_ARGS__);fprintf(stderr,"\n");}while(0);

__IO bool attach_done = false;

void delay_ms(uint32_t t)
{
    HAL_Delay(t);
}

// usbh_conf.c
extern HCD_HandleTypeDef hhcd_USB_OTG_FS;

void HAL_HCD_MspInit(HCD_HandleTypeDef* hhcd)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hhcd->Instance==USB_OTG_FS)
  {
    /* Peripheral clock enable */
    __USB_OTG_FS_CLK_ENABLE();
  
    /**USB_OTG_FS GPIO Configuration    
    PA11     ------> USB_OTG_FS_DM
    PA12     ------> USB_OTG_FS_DP 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral interrupt init*/
    /* Sets the priority grouping field */
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
  }
}

// stm32f4xx_it.c
extern "C" {
void HAL_HCD_Connect_Callback(HCD_HandleTypeDef *hhcd)
{
    USB_TRACE1(hhcd);
    attach_done = true;
}

} // extern "C"

USBHALHost* USBHALHost::instHost;

USBHALHost::USBHALHost() {
    instHost = this;
}

void USBHALHost::init() {
    hhcd_USB_OTG_FS.Instance = USB_OTG_FS;
    hhcd_USB_OTG_FS.Init.Host_channels = 8;
    hhcd_USB_OTG_FS.Init.speed = HCD_SPEED_FULL;
    hhcd_USB_OTG_FS.Init.dma_enable = DISABLE;
    hhcd_USB_OTG_FS.Init.phy_itface = HCD_PHY_EMBEDDED;
    hhcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
    hhcd_USB_OTG_FS.Init.low_power_enable = ENABLE;
    hhcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
    hhcd_USB_OTG_FS.Init.use_external_vbus = DISABLE;

    HAL_HCD_Init(&hhcd_USB_OTG_FS);
    HAL_HCD_Start(&hhcd_USB_OTG_FS);

    bool lowSpeed = wait_attach();
    delay_ms(200);
    HAL_HCD_ResetPort(&hhcd_USB_OTG_FS);
    delay_ms(100); // Wait for 100 ms after Reset
    addDevice(NULL, 0, lowSpeed);
}

bool USBHALHost::wait_attach() {
    Timer t;
    t.reset();
    t.start();
    while(!attach_done) {
        if (t.read_ms() > 5*1000) {
            t.reset();
            USB_INFO("Please attach USB device.");
        }
    }
    wait_ms(100);
    return HAL_HCD_GetCurrentSpeed(&hhcd_USB_OTG_FS) == USB_OTG_SPEED_LOW;
}

int USBHALHost::token_setup(USBEndpoint* ep, SETUP_PACKET* setup, uint16_t wLength) {
    const uint8_t ep_addr = 0x00;
    HC hc;
    USBDeviceConnected* dev = ep->getDevice();
    hc.Init(ep_addr, 
        dev->getAddress(),
        dev->getSpeed() ? HCD_SPEED_LOW : HCD_SPEED_FULL,
        EP_TYPE_CTRL, ep->getSize());

    setup->wLength = wLength;
    hc.SubmitRequest((uint8_t*)setup, 8, true); // PID_SETUP
    while(hc.GetURBState() == URB_IDLE);

    switch(hc.GetURBState()) {
        case URB_DONE:
            LastStatus = ACK;
            break;
        default:
            LastStatus = 0xff;
            break;
    }
    ep->setData01(DATA1);
    return 8;
}

int USBHALHost::token_in(USBEndpoint* ep, uint8_t* data, int size, int retryLimit) {
    switch(ep->getType()) {
        case CONTROL_ENDPOINT:
            return token_ctl_in(ep, data, size, retryLimit);
        case INTERRUPT_ENDPOINT:
            return token_int_in(ep, data, size);
        case BULK_ENDPOINT:
            return token_blk_in(ep, data, size, retryLimit);
    }
    return -1;
}

int USBHALHost::token_ctl_in(USBEndpoint* ep, uint8_t* data, int size, int retryLimit) {
    const uint8_t ep_addr = 0x80;
    HC hc;
    USBDeviceConnected* dev = ep->getDevice();
    hc.Init(ep_addr, 
        dev->getAddress(),
        dev->getSpeed() ? HCD_SPEED_LOW : HCD_SPEED_FULL,
        EP_TYPE_CTRL, ep->getSize());

    hc.SetToggle((ep->getData01() == DATA0) ? 0 : 1);

    hc.SubmitRequest(data, size);
    while(hc.GetURBState() == URB_IDLE);

    switch(hc.GetURBState()) {
        case URB_DONE:
            LastStatus = ACK;
            break;
        default:
            LastStatus = 0xff;
            return -1;
    }
    ep->toggleData01();
    return hc.GetXferCount();
}

int USBHALHost::token_int_in(USBEndpoint* ep, uint8_t* data, int size) {
    HC hc;
    USBDeviceConnected* dev = ep->getDevice();
    hc.Init(
        ep->getAddress(),
        dev->getAddress(),
        dev->getSpeed() ? HCD_SPEED_LOW : HCD_SPEED_FULL,
        EP_TYPE_INTR, ep->getSize());

    hc.SetToggle((ep->getData01() == DATA0) ? 0 : 1);

    hc.SubmitRequest(data, size);
    while(hc.GetURBState() == URB_IDLE);
    switch(hc.GetURBState()) {
        case URB_DONE:
            switch(hc.GetState()) {
                case HC_XFRC:
                    LastStatus = ep->getData01();
                    ep->toggleData01();
                    return hc.GetXferCount();
                case HC_NAK:
                    LastStatus = NAK;
                    return -1;
            }
            break;
    }
    LastStatus = STALL;
    return -1;
}

int USBHALHost::token_blk_in(USBEndpoint* ep, uint8_t* data, int size, int retryLimit) {
    HC hc;
    USBDeviceConnected* dev = ep->getDevice();
    hc.Init(
        ep->getAddress(),
        dev->getAddress(),
        HCD_SPEED_FULL,
        EP_TYPE_BULK, ep->getSize());

    hc.SetToggle((ep->getData01() == DATA0) ? 0 : 1);

    int retry = 0;
    do {
        hc.SubmitRequest(data, size);
        while(hc.GetURBState() == URB_IDLE);

        switch(hc.GetURBState()) {
            case URB_DONE:
                switch(hc.GetState()) {
                    case HC_XFRC:
                        LastStatus = ep->getData01();
                        ep->toggleData01();
                        return hc.GetXferCount();
                    case HC_NAK:
                        LastStatus = NAK;
                        if (retryLimit > 0) {
                            delay_ms(1 + 10 * retry);
                        }
                        break;
                    default:
                        break;
                }
                break;
            case URB_STALL:
                LastStatus = STALL;
                return -1;
            default:
                LastStatus = STALL;
                delay_ms(500 + 100 * retry);
                break;
        }
    }while(retry++ < retryLimit);
    return -1;
}

int USBHALHost::token_out(USBEndpoint* ep, const uint8_t* data, int size, int retryLimit) {
    switch(ep->getType()) {
        case CONTROL_ENDPOINT:
            return token_ctl_out(ep, data, size, retryLimit);
        case INTERRUPT_ENDPOINT:
            return token_int_out(ep, data, size);
        case BULK_ENDPOINT:
            return token_blk_out(ep, data, size, retryLimit);
    }
    return -1;
}

int USBHALHost::token_ctl_out(USBEndpoint* ep, const uint8_t* data, int size, int retryLimit) {
    const uint8_t ep_addr = 0x00;
    HC hc;
    USBDeviceConnected* dev = ep->getDevice();
    hc.Init(ep_addr, 
        dev->getAddress(),
        dev->getSpeed() ? HCD_SPEED_LOW : HCD_SPEED_FULL,
        EP_TYPE_CTRL, ep->getSize());

    hc.SetToggle((ep->getData01() == DATA0) ? 0 : 1);

    do {
        hc.SubmitRequest((uint8_t*)data, size);
        while(hc.GetURBState() == URB_IDLE);

        switch(hc.GetURBState()) {
            case URB_DONE:
                LastStatus = ACK;
                ep->toggleData01();
                return size;

            default:
                break;
        }
        delay_ms(1);
    }while(retryLimit-- > 0); 
    return -1;
}

int USBHALHost::token_int_out(USBEndpoint* ep, const uint8_t* data, int size) {
    HC hc;
    USBDeviceConnected* dev = ep->getDevice();
    hc.Init(
        ep->getAddress(),
        dev->getAddress(),
        dev->getSpeed() ? HCD_SPEED_LOW : HCD_SPEED_FULL,
        EP_TYPE_INTR, ep->getSize());

    hc.SetToggle((ep->getData01() == DATA0) ? 0 : 1);

    hc.SubmitRequest((uint8_t*)data, size);
    while(hc.GetURBState() == URB_IDLE);
    if (hc.GetURBState() != URB_DONE) {
        return -1;
    }
    ep->toggleData01();
    return size;
}

int USBHALHost::token_blk_out(USBEndpoint* ep, const uint8_t* data, int size, int retryLimit) {
    HC hc;
    USBDeviceConnected* dev = ep->getDevice();
    hc.Init(
        ep->getAddress(), dev->getAddress(),
        HCD_SPEED_FULL, EP_TYPE_BULK, ep->getSize());

    hc.SetToggle((ep->getData01() == DATA0) ? 0 : 1);

    int retry = 0;
    do {
        hc.SubmitRequest((uint8_t*)data, size);
        while(hc.GetURBState() == URB_IDLE);

        switch(hc.GetURBState()) {
            case URB_DONE:
                switch(hc.GetState()) {
                    case HC_XFRC: // ACK
                        LastStatus = ep->getData01();
                        ep->toggleData01();
                        return size;
                    default:
                        break;
                }
                break;
            case URB_NOTREADY: // HC_NAK
                LastStatus = NAK;
                delay_ms(100 * retry);
                break;
            default:
                LastStatus = STALL;
                return -1;
        }
    } while(retry++ < retryLimit);
    return -1;
}

int USBHALHost::token_iso_in(USBEndpoint* ep, uint8_t* data, int size) {
    HC* hc = ep->getHALData<HC*>();
    if (hc == NULL) {
        hc = new HC;
        ep->setHALData<HC*>(hc);
        USBDeviceConnected* dev = ep->getDevice();
        hc->Init(
            ep->getAddress(), dev->getAddress(),
            HCD_SPEED_FULL, EP_TYPE_ISOC, ep->getSize());
    }
    hc->SubmitRequest(data, size);
    while(hc->GetURBState() == URB_IDLE);
    return hc->GetXferCount();
}

int USBHALHost::multi_token_in(USBEndpoint* ep, uint8_t* data, size_t total, bool block) {
    if (total == 0) {
        return token_in(ep);
    }
    int retryLimit = block ? 10 : 0;
    int read_len = 0;
    for(int n = 0; read_len < total; n++) {
        int size = std::min((int)total-read_len, ep->getSize());
        int result = token_in(ep, data+read_len, size, retryLimit);
        if (result < 0) {
            if (block) {
                return -1;
            }
            if (LastStatus == NAK) {
                if (n == 0) {
                    return -1;
                }
                break;
            }
            return result;
        }
        read_len += result;
        if (result < ep->getSize()) {
            break;
        }
    }
    return read_len;
}

int USBHALHost::multi_token_out(USBEndpoint* ep, const uint8_t* data, size_t total) {
    if (total == 0) {
        return token_out(ep);
    }
    int write_len = 0;
    for(int n = 0; write_len < total; n++) {
        int size = std::min((int)total-write_len, ep->getSize());
        int result = token_out(ep, data+write_len, size);
        if (result < 0) {
            if (LastStatus == NAK) {
                if (n == 0) {
                    return -1;
                }
                break;
            }
            USB_DBG("token_out result=%d %02x", result, LastStatus);
            return result;
        }
        write_len += result;
        if (result < ep->getSize()) {
            break;
        }
    }
    return write_len;
}
void USBHALHost::multi_token_inNB(USBEndpoint* ep, uint8_t* data, int size) {
    USB_TRACE1(size);
    USB_TEST_ASSERT(ep->getState() != USB_TYPE_PROCESSING);
    ep->setBuffer(data, size);
    ep->setState(USB_TYPE_PROCESSING);
}

USB_TYPE USBHALHost::multi_token_inNB_result(USBEndpoint* ep) {
    USB_TEST_ASSERT(ep->getState() == USB_TYPE_PROCESSING);
    uint8_t* buf = ep->getBufStart();
    int size = ep->getBufSize();
    int result = multi_token_in(ep, buf, size, false);
    USB_TRACE1(result);
    if (result < 0) {
        return USB_TYPE_PROCESSING;
    }
    ep->setLengthTransferred(result);
    ep->setState(USB_TYPE_IDLE);
    return USB_TYPE_OK;

}

void USBHALHost::setToggle(USBEndpoint* ep, uint8_t toggle) {
    USB_TEST_ASSERT(toggle == 1);
    ep->setData01(toggle == 0 ? DATA0 : DATA1);
}

uint8_t HC::slot = 0x00;

HC::HC() {
    static const int start = 1;
    uint8_t mask = (1<<start);
    for(int i = start; i < 8; i++, mask <<= 1) {
        if (!(slot & mask)) {
            slot |= mask;
            _ch = i;
            return;
        }
    }
    _ch = 0; // ERROR!!!
}

HC::HC(int ch) {
    _ch = ch;
    slot |= (1<<_ch);
}

HC::~HC() {
    slot &= ~(1<<_ch);
}

HAL_StatusTypeDef HC::Init(uint8_t epnum, uint8_t dev_address, uint8_t speed, uint8_t ep_type, uint16_t mps) {
    _ep_addr = epnum;
    _ep_type = ep_type;
    return HAL_HCD_HC_Init(&hhcd_USB_OTG_FS, _ch,
                           epnum, dev_address, speed, ep_type, mps);
}

HAL_StatusTypeDef HC::SubmitRequest(uint8_t* pbuff, uint16_t length, bool setup) {
    uint8_t direction = (_ep_addr & 0x80) ? DIR_IN : DIR_OUT;
    if (_ep_type == EP_TYPE_CTRL) {
        HCD_HCTypeDef* hc = &hhcd_USB_OTG_FS.hc[_ch];
        if (setup) {
            hc->data_pid = HC_PID_SETUP;
            hc->toggle_out = 0;
        } else {
            if (direction == DIR_IN) {
                if (hc->toggle_in == 0) {
                    hc->data_pid = HC_PID_DATA0;
                } else {
                    hc->data_pid = HC_PID_DATA1;
                }
            } else { // OUT
                if (hc->toggle_out == 0) {
                    hc->data_pid = HC_PID_DATA0;
                } else {
                    hc->data_pid = HC_PID_DATA1;
                }
            }
        }
        hc->xfer_buff = pbuff;
        hc->xfer_len  = length;
        hc->urb_state = URB_IDLE;
        hc->xfer_count = 0;
        hc->ch_num = _ch;
        hc->state = HC_IDLE;
  
        return USB_HC_StartXfer(hhcd_USB_OTG_FS.Instance, hc, 0);
    }
    return HAL_HCD_HC_SubmitRequest(&hhcd_USB_OTG_FS, _ch,
                                    direction, _ep_type, 0, pbuff, length, 0);
}

HCD_URBStateTypeDef HC::GetURBState() {
    return HAL_HCD_HC_GetURBState(&hhcd_USB_OTG_FS, _ch);
}

HCD_HCStateTypeDef HC::GetState() {
    return HAL_HCD_HC_GetState(&hhcd_USB_OTG_FS, _ch);
}

uint32_t HC::GetXferCount() {
    return HAL_HCD_HC_GetXferCount(&hhcd_USB_OTG_FS, _ch);
}

void HC::SetToggle(uint8_t toggle) {
    if (_ep_addr & 0x80) { // IN
        hhcd_USB_OTG_FS.hc[_ch].toggle_in = toggle;
    } else { // OUT
        hhcd_USB_OTG_FS.hc[_ch].toggle_out = toggle;
    }
}

#endif




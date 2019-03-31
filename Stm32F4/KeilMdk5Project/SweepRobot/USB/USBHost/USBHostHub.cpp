#include "USBHost.h"

#define PORT_CONNECTION   0
#define PORT_ENABLE       1
#define PORT_SUSPEND      2
#define PORT_OVER_CURRENT 3
#define PORT_RESET        4
#define PORT_POWER        8
#define PORT_LOW_SPEED    9

#define C_PORT_CONNECTION   16
#define C_PORT_ENABLE       17
#define C_PORT_SUSPEND      18
#define C_PORT_OVER_CURRENT 19
#define C_PORT_RESET        20

bool USBHost::Hub(USBDeviceConnected* dev) {
    USB_INFO("New HUB: VID:%04x PID:%04x [dev: %p]", dev->getVid(), dev->getPid(), dev);
    HubDescriptor hubdesc;
    // get HUB descriptor
    int rc = controlRead(dev, 
                        USB_DEVICE_TO_HOST | USB_REQUEST_TYPE_CLASS,
                        GET_DESCRIPTOR,
                        0x29 << 8, 0, reinterpret_cast<uint8_t*>(&hubdesc), 
                        sizeof(HubDescriptor));
    USB_TEST_ASSERT(rc == USB_TYPE_OK);
    if (rc != USB_TYPE_OK) {
        return false;
    }
    USB_DBG_HEX((uint8_t*)&hubdesc, sizeof(hubdesc));

    uint32_t status;
    rc = controlRead( dev,
                      0xa0, 0, 0, 0, reinterpret_cast<uint8_t*>(&status), 4);
    USB_TEST_ASSERT(rc == USB_TYPE_OK);
    if (rc != USB_TYPE_OK) {
        return false;
    }
    USB_DBG("HUB STATUS: %08X\n", status);

    for(int i = 1; i <= hubdesc.bNbrPorts; i++) {
        SetPortPower(dev, i); // power on
        wait_ms(hubdesc.bPwrOn2PwrGood*2);
        uint32_t status;
        GetPortStatus(dev, i, &status);
        USB_DBG("port: %d status: %08X\n", i, status);
        if (status & 0x010000) { // Connect Status Change, has changed
            USB_TEST_ASSERT(status & 0x000001);
            ClearPortFeature(dev, C_PORT_CONNECTION, i);
            int lowSpeed = 0;
            if (status & 0x0200) {
                lowSpeed = 1;
            }
            PortReset(dev, i);
            if (!addDevice(dev, i, lowSpeed)) {
                ClearPortPower(dev, i); // power off
            }
        } else {
            ClearPortPower(dev, i); // power off
        }
    }
    return false;
}


int USBHost::SetPortPower(USBDeviceConnected* dev, int port)
{
    return SetPortFeature(dev, PORT_POWER, port);
}

int USBHost::ClearPortPower(USBDeviceConnected* dev, int port)
{
    return ClearPortFeature(dev, PORT_POWER, port);
}

int USBHost::SetPortFeature(USBDeviceConnected* dev, int feature, int index)
{
    return controlWrite(dev, 0x23, SET_FEATURE,feature,index,0,0);
}

int USBHost::ClearPortFeature(USBDeviceConnected* dev, int feature, int index)
{
    return controlWrite(dev, 0x23, CLEAR_FEATURE,feature,index,0,0);
}

int USBHost::SetPortReset(USBDeviceConnected* dev, int port)
{
    return SetPortFeature(dev, PORT_RESET, port);
}

int USBHost::GetPortStatus(USBDeviceConnected* dev, int port, uint32_t* status)
{
    return controlRead(dev, 0xa3, GET_STATUS, 0, port, (uint8_t*)status, 4);
}

int USBHost::PortReset(USBDeviceConnected* dev, int port)
{
    USB_DBG("%p port=%d\n", this, port);
    USB_TEST_ASSERT(port >= 1);
    SetPortReset(dev, port);
    // wait reset
    for(int i = 0; i < 100; i++) {
        uint32_t status;    
        GetPortStatus(dev, port, &status);
        USB_DBG("RESET port: %d status: %08X\n", port, status);
        if (status & 0x100000) { // Reset change , Reset complete
            return USB_TYPE_OK;
        }
        wait_ms(5);
     }
     return USB_TYPE_ERROR;
}


/* mbed USBHost Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "USBHost.h"

#define USB_TRACE1(A) while(0)
#undef USB_TEST_ASSERT
void usb_test_assert_internal(const char *expr, const char *file, int line);
#define USB_TEST_ASSERT(EXPR) while(!(EXPR)){usb_test_assert_internal(#EXPR,__FILE__,__LINE__);}

USBHost* USBHost::inst = NULL;

USBHost* USBHost::getHostInst() {
    if (inst == NULL) {
        inst = new USBHost();
        inst->init();
    }
    return inst;
}

void USBHost::poll()
{
    if (inst) {
        inst->task();
    }
}

USBHost::USBHost() {
}

/* virtual */ bool USBHost::addDevice(USBDeviceConnected* parent, int port, bool lowSpeed) {
    USBDeviceConnected* dev = new USBDeviceConnected;
    USBEndpoint* ep = new USBEndpoint(dev);
    dev->init(0, port, lowSpeed);
    dev->setAddress(0);
    dev->setEpCtl(ep);
    uint8_t desc[18];
    wait_ms(100);

    int rc = controlRead(dev, 0x80, GET_DESCRIPTOR, 1<<8, 0, desc, 8);
    USB_TEST_ASSERT(rc == USB_TYPE_OK);
    if (rc != USB_TYPE_OK) {
        USB_ERR("ADD DEVICE FAILD");
    }
    USB_DBG_HEX(desc, 8);
    DeviceDescriptor* dev_desc = reinterpret_cast<DeviceDescriptor*>(desc);
    ep->setSize(dev_desc->bMaxPacketSize);

    int new_addr = USBDeviceConnected::getNewAddress();
    rc = controlWrite(dev, 0x00, SET_ADDRESS, new_addr, 0, NULL, 0);
    USB_TEST_ASSERT(rc == USB_TYPE_OK);
    dev->setAddress(new_addr);
    wait_ms(100);

    rc = controlRead(dev, 0x80, GET_DESCRIPTOR, 1<<8, 0, desc, sizeof(desc));
    USB_TEST_ASSERT(rc == USB_TYPE_OK);
    USB_DBG_HEX(desc, sizeof(desc));

    dev->setVid(dev_desc->idVendor);
    dev->setPid(dev_desc->idProduct);
    dev->setClass(dev_desc->bDeviceClass);
    USB_INFO("parent:%p port:%d speed:%s VID:%04x PID:%04x class:%02x addr:%d",
        parent, port, (lowSpeed ? "low " : "full"), dev->getVid(), dev->getPid(), dev->getClass(),
        dev->getAddress());

    DeviceLists.push_back(dev);

    if (dev->getClass() == HUB_CLASS) {
        const int config = 1;
        int rc = controlWrite(dev, 0x00, SET_CONFIGURATION, config, 0, NULL, 0);
        USB_TEST_ASSERT(rc == USB_TYPE_OK);
        wait_ms(100);
        Hub(dev);
    }
    return true;
}

// enumerate a device with the control USBEndpoint
USB_TYPE USBHost::enumerate(USBDeviceConnected * dev, IUSBEnumerator* pEnumerator)
{
    if (dev->getClass() == HUB_CLASS) { // skip hub class
        return USB_TYPE_OK;
    }
    uint8_t desc[18];
    USB_TYPE rc = controlRead(dev, 0x80, GET_DESCRIPTOR, 1<<8, 0, desc, sizeof(desc));
    USB_TEST_ASSERT(rc == USB_TYPE_OK);
    USB_DBG_HEX(desc, sizeof(desc));
    if (rc != USB_TYPE_OK) {
        return rc;
    }
    DeviceDescriptor* dev_desc = reinterpret_cast<DeviceDescriptor*>(desc);
    dev->setClass(dev_desc->bDeviceClass);
    pEnumerator->setVidPid(dev->getVid(), dev->getPid());

    rc = controlRead(dev, 0x80, GET_DESCRIPTOR, 2<<8, 0, desc, 4);
    USB_TEST_ASSERT(rc == USB_TYPE_OK);
    USB_DBG_HEX(desc, 4);

    int TotalLength = desc[2]|desc[3]<<8;
    uint8_t* buf = new uint8_t[TotalLength];
    rc = controlRead(dev, 0x80, GET_DESCRIPTOR, 2<<8, 0, buf, TotalLength);
    USB_TEST_ASSERT(rc == USB_TYPE_OK);
    //USB_DBG_HEX(buf, TotalLength);

    // Parse the configuration descriptor
    parseConfDescr(dev, buf, TotalLength, pEnumerator);
    delete[] buf;
    // only set configuration if not enumerated before
    if (!dev->isEnumerated()) {
        USB_DBG("Set configuration 1 on dev: %p", dev);
        // sixth step: set configuration (only 1 supported)
        int config = 1;
        USB_TYPE res = controlWrite(dev, 0x00, SET_CONFIGURATION, config, 0, NULL, 0);
        if (res != USB_TYPE_OK) {
            USB_ERR("SET CONF FAILED");
            return res;
        }
        // Some devices may require this delay
        wait_ms(100);
        dev->setEnumerated();
        // Now the device is enumerated!
        USB_DBG("dev %p is enumerated", dev);
    }
    return USB_TYPE_OK;
}

// this method fills the USBDeviceConnected object: class,.... . It also add endpoints found in the descriptor.
void USBHost::parseConfDescr(USBDeviceConnected * dev, uint8_t * conf_descr, uint32_t len, IUSBEnumerator* pEnumerator)
{
    uint32_t index = 0;
    uint32_t len_desc = 0;
    uint8_t id = 0;
    USBEndpoint * ep = NULL;
    uint8_t intf_nb = 0;
    bool parsing_intf = false;
    uint8_t current_intf = 0;
    EndpointDescriptor* ep_desc;

    while (index < len) {
        len_desc = conf_descr[index];
        id = conf_descr[index+1];
        USB_DBG_HEX(conf_descr+index, len_desc);
        switch (id) {
            case CONFIGURATION_DESCRIPTOR:
                USB_DBG("dev: %p has %d intf", dev, conf_descr[4]);
                dev->setNbIntf(conf_descr[4]);
                break;
            case INTERFACE_DESCRIPTOR:
                if(pEnumerator->parseInterface(conf_descr[index + 2], conf_descr[index + 5], conf_descr[index + 6], conf_descr[index + 7])) {
                    intf_nb++;
                    current_intf = conf_descr[index + 2];
                    dev->addInterface(current_intf, conf_descr[index + 5], conf_descr[index + 6], conf_descr[index + 7]);
                    USB_DBG("ADD INTF %d on device %p: class: %d, subclass: %d, proto: %d", current_intf, dev, conf_descr[index + 5],conf_descr[index + 6],conf_descr[index + 7]);
                    parsing_intf = true;
                } else {
                    parsing_intf = false;
                }
                break;
            case ENDPOINT_DESCRIPTOR:
                ep_desc = reinterpret_cast<EndpointDescriptor*>(conf_descr+index);
                if (parsing_intf && (intf_nb <= MAX_INTF) ) {
                    ENDPOINT_TYPE type = (ENDPOINT_TYPE)(ep_desc->bmAttributes & 0x03);
                    ENDPOINT_DIRECTION dir = (ep_desc->bEndpointAddress & 0x80) ? IN : OUT;
                    if(pEnumerator->useEndpoint(current_intf, type, dir)) {
                        ep = new USBEndpoint(dev);
                        ep->init(type, dir, ep_desc->wMaxPacketSize, ep_desc->bEndpointAddress);
                        USB_DBG("ADD USBEndpoint %p, on interf %d on device %p", ep, current_intf, dev);
                        dev->addEndpoint(current_intf, ep);
                    }
                }
                break;
            case HID_DESCRIPTOR:
                //lenReportDescr = conf_descr[index + 7] | (conf_descr[index + 8] << 8);
                break;
            default:
                break;
        }
        index += len_desc;
    }
}

USB_TYPE USBHost::controlRead(USBDeviceConnected* dev, uint8_t requestType, uint8_t request, uint32_t value, uint32_t index, uint8_t * buf, uint32_t len) {
    USBEndpoint* ep = dev->getEpCtl();
    SETUP_PACKET setup(requestType, request, value, index, len);

    int result = token_setup(ep, &setup, len); // setup stage
    USB_TRACE1(result);
    if (result < 0) {
        return USB_TYPE_ERROR;
    }

    int read_len = multi_token_in(ep, buf, len); // data stage
    USB_TRACE1(read_len);
    if (read_len < 0) {
        return USB_TYPE_ERROR;
    }

    setToggle(ep, 1); // DATA1
    result = multi_token_out(ep); // status stage
    USB_TRACE1(result);
    if (result < 0) {
        return USB_TYPE_ERROR;
    }
    ep->setLengthTransferred(read_len);
    return USB_TYPE_OK;
}

USB_TYPE USBHost::controlWrite(USBDeviceConnected* dev, uint8_t requestType, uint8_t request, uint32_t value, uint32_t index, uint8_t * buf, uint32_t len) {
    USBEndpoint* ep = dev->getEpCtl();
    SETUP_PACKET setup(requestType, request, value, index, len);

    int result = token_setup(ep, &setup, len); // setup stage
    USB_TRACE1(result);
    if (result < 0) {
        return USB_TYPE_ERROR;
    }
    int write_len = 0;
    if (buf != NULL) {
        write_len = multi_token_out(ep, buf, len); // data stage
        USB_TRACE1(write_len);
        if (write_len < 0) {
            return USB_TYPE_ERROR;
        }
    }

    setToggle(ep, 1); // DATA1
    result = multi_token_in(ep); // status stage
    USB_TRACE1(result);
    if (result < 0) {
        return USB_TYPE_ERROR;
    }
    ep->setLengthTransferred(write_len);
    return USB_TYPE_OK;
}

USB_TYPE USBHost::bulkRead(USBDeviceConnected* dev, USBEndpoint* ep, uint8_t* buf, uint32_t len, bool blocking) {
    if (blocking == false) {
        ep->setBuffer(buf, len);
        ep_queue.push(ep);
        multi_token_inNB(ep, buf, len);
        return USB_TYPE_PROCESSING;
    }
    int result = multi_token_in(ep, buf, len);
    USB_TRACE1(result);
    if (result < 0) {
        return USB_TYPE_ERROR;
    }
    ep->setLengthTransferred(result);
    return USB_TYPE_OK;
}

USB_TYPE USBHost::bulkWrite(USBDeviceConnected* dev, USBEndpoint* ep, uint8_t* buf, uint32_t len, bool blocking) {
    USB_TEST_ASSERT(blocking);
    int result = multi_token_out(ep, buf, len);
    USB_TRACE1(result);
    if (result < 0) {
        return USB_TYPE_ERROR;
    }
    ep->setLengthTransferred(result);
    return USB_TYPE_OK;
}

USB_TYPE USBHost::interruptRead(USBDeviceConnected* dev, USBEndpoint* ep, uint8_t* buf, uint32_t len, bool blocking) {
    if (blocking == false) {
        ep->setBuffer(buf, len);
        ep_queue.push(ep);
        multi_token_inNB(ep, buf, len);
        return USB_TYPE_PROCESSING;
    }
    int result = multi_token_in(ep, buf, len);
    USB_TRACE1(result);
    if (result < 0) {
        return USB_TYPE_ERROR;
    }
    ep->setLengthTransferred(result);
    return USB_TYPE_OK;
}

USB_TYPE USBHost::interruptWrite(USBDeviceConnected* dev, USBEndpoint* ep, uint8_t* buf, uint32_t len, bool blocking) {
    USB_TEST_ASSERT(blocking);
    int result = multi_token_out(ep, buf, len);
    USB_TRACE1(result);
    if (result < 0) {
        return USB_TYPE_ERROR;
    }
    ep->setLengthTransferred(result);
    return USB_TYPE_OK;
}

USB_TYPE USBHost::isochronousRead(USBDeviceConnected* dev, USBEndpoint* ep, uint8_t* buf, uint32_t len, bool blocking) {
    USB_TEST_ASSERT(blocking);
    isochronousReadNB(ep, buf, len);
    return USB_TYPE_OK;
}

int USBHost::interruptReadNB(USBEndpoint* ep, uint8_t* data, int size) {
    USB_TRACE1(size);
    if (ep->getState() != USB_TYPE_PROCESSING) {
        ep->setState(USB_TYPE_PROCESSING);
        ep->setBuffer(data, size);
        multi_token_inNB(ep, data, size);
    }
    if (multi_token_inNB_result(ep) != USB_TYPE_PROCESSING) {
        return ep->getLengthTransferred();
    }
    return -1;
}

int USBHost::bulkReadNB(USBEndpoint* ep, uint8_t* data, int size) {
    USB_TRACE1(size);
    return interruptReadNB(ep, data, size);
}

int USBHost::isochronousReadNB(USBEndpoint* ep, uint8_t* data, int size) {
    USB_TRACE1(size);
    int result = token_iso_in(ep, data, size);
    if (result >= 0) {
         ep->setLengthTransferred(result);
    }
    return result;
}

void USBHost::task() {
    USBEndpoint* ep = ep_queue.pop();
    if (ep) {
        USB_TEST_ASSERT(ep->getDir() == IN);
        if (multi_token_inNB_result(ep) != USB_TYPE_PROCESSING) {
            ep->call();
        } else {
            ep_queue.push(ep);
        }
    }
}

void usb_test_assert_internal(const char *expr, const char *file, int line){
    error("\n\n%s@%d %s ASSERT!\n\n", file, line, expr);
}



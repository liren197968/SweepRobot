/*
 * mbed USBHost Gamepad driver sample
 * Copyright (c) 2014 Yuuichi Akagawa
 *
 * modified from mbed USBHostMouse
 *
 * mbed USBHost Library
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
 
#include "USBHostGamepad.h"
//#if USBHOST_GAMEPAD

USBHostGamepad::USBHostGamepad() {
    host = USBHost::getHostInst();
    init();
}

void USBHostGamepad::init() {
    dev = NULL;
    int_in = NULL;
    dev_connected = false;
    gamepad_device_found = false;
    gamepad_intf = -1;
}

bool USBHostGamepad::connected() {
    return dev_connected;
}

bool USBHostGamepad::connect() {
    if (dev_connected) {
        return true;
    }

    for (uint8_t i = 0; i < MAX_DEVICE_CONNECTED; i++) {
        if ((dev = host->getDevice(i)) != NULL) {

            if(host->enumerate(dev, this))
                break;

            if (gamepad_device_found) {

                int_in = dev->getEndpoint(gamepad_intf, INTERRUPT_ENDPOINT, IN);
                if (!int_in)
                    break;

                USB_INFO("New Gamepad/Joystick device: VID:%04x PID:%04x [dev: %p - intf: %d]", dev->getVid(), dev->getPid(), dev, gamepad_intf);
#if DEBUG > 3
                //Parse HID Report Descriptor
                parseHidDescr();
#endif
                dev->setName("Gamepad", gamepad_intf);
                host->registerDriver(dev, gamepad_intf, this, &USBHostGamepad::init);

                int_in->attach(this, &USBHostGamepad::rxHandler);
                rxSize = int_in->getSize();
                if (rxSize > sizeof(report))
                    rxSize = sizeof(report);
                host->interruptRead(dev, int_in, report, rxSize, false);

                dev_connected = true;
                return true;
            }
        }
    }
    init();
    return false;
}

void USBHostGamepad::rxHandler() {
    if (dev)
        host->interruptRead(dev, int_in, report, rxSize, false);
        
    /* modified 2016 Racoon */
        
}

/*virtual*/ void USBHostGamepad::setVidPid(uint16_t vid, uint16_t pid)
{
    // we don't check VID/PID for gamepad driver
}

/*virtual*/ bool USBHostGamepad::parseInterface(uint8_t intf_nb, uint8_t intf_class, uint8_t intf_subclass, uint8_t intf_protocol) //Must return true if the interface should be parsed
{
    if ((gamepad_intf == -1) &&
        (intf_class == HID_CLASS) &&
        (intf_subclass == 0x00) &&
        (intf_protocol == 0x00)) {
        gamepad_intf = intf_nb;
        return true;
    }
    return false;
}

/*virtual*/ bool USBHostGamepad::useEndpoint(uint8_t intf_nb, ENDPOINT_TYPE type, ENDPOINT_DIRECTION dir) //Must return true if the endpoint will be used
{
    if (intf_nb == gamepad_intf) {
        if (type == INTERRUPT_ENDPOINT && dir == IN) {
            gamepad_device_found = true;
            return true;
        }
    }
    return false;
}



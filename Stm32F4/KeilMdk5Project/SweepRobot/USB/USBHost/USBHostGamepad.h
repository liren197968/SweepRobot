/* 
 * modified from
 * mbed USBHost Gamepad driver sample
 * Copyright (c) 2014 Yuuichi Akagawa
 *
 * modified from mbed USBHostMouse
 *
 * Copyright (c) 2014 mbed.org, MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef USBHOSTGAMEPAD_H
#define USBHOSTGAMEPAD_H

#include "USBHostConf.h"

//#if USBHOST_GAMEPAD

#include "USBHost.h"
//HID Class Request
#define HID_GET_REPORT                                 0x01
#define HID_GET_IDLE                                   0x02
#define HID_GET_PROTOCOL                               0x03
#define HID_GET_DESCRIPTOR                             0x06
#define HID_SET_REPORT                                 0x09
#define HID_SET_IDLE                                   0x0a
#define HID_SET_PROTOCOL                               0x0b

/** 
 * A class to communicate a USB MIDI device
 */
class USBHostGamepad : public IUSBEnumerator {
public:
    /**
     * Constructor
     */
    USBHostGamepad();

    /**
     * Try to connect a gamepad device
     *
     * @return true if connection was successful
     */
    bool connect();

    /**
    * Check if a gamepad is connected
    *
    * @returns true if a gamepad is connected
    */
    bool connected();

    uint8_t report[8]; /* modified 2016 Racoon */
    int rxSize;
        
protected:
    //From IUSBEnumerator
    virtual void setVidPid(uint16_t vid, uint16_t pid);
    virtual bool parseInterface(uint8_t intf_nb, uint8_t intf_class, uint8_t intf_subclass, uint8_t intf_protocol); //Must return true if the interface should be parsed
    virtual bool useEndpoint(uint8_t intf_nb, ENDPOINT_TYPE type, ENDPOINT_DIRECTION dir); //Must return true if the endpoint will be used

private:
    USBHost * host;
    USBDeviceConnected * dev;
    USBEndpoint * int_in;

    bool dev_connected;
    bool gamepad_device_found;
    int gamepad_intf;

    void rxHandler();
    void init();
};

#endif


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
#pragma once

#include "USBEndpoint.h"
#include "USBHostConf.h"
#include "myvector.h"
#include "mymap.h"

class USBEndpoint;

struct INTERFACE {
    INTERFACE(uint8_t _class, uint8_t _subclass, uint8_t _protocol) {
        intf_class = _class;
        intf_subclass = _subclass;
        intf_protocol = _protocol;
    }
    uint8_t intf_class;
    uint8_t intf_subclass;
    uint8_t intf_protocol;
    myvector<USBEndpoint*>ep; 
}; 

/**
* USBDeviceConnected class
*/
class USBDeviceConnected {
public:

    /**
    * Constructor
    */
    USBDeviceConnected();

    /**
    * Attach an USBEndpoint to this device
    *
    * @param intf_nb interface number
    * @param ep pointeur on the USBEndpoint which will be attached
    * @returns true if successful, false otherwise
    */
    bool addEndpoint(uint8_t intf_nb, USBEndpoint * ep);

    /**
    * Retrieve an USBEndpoint by its TYPE and DIRECTION
    *
    * @param intf_nb the interface on which to lookup the USBEndpoint
    * @param type type of the USBEndpoint looked for
    * @param dir direction of the USBEndpoint looked for
    * @param index the index of the USBEndpoint whitin the interface
    * @returns pointer on the USBEndpoint if found, NULL otherwise
    */
    USBEndpoint * getEndpoint(uint8_t intf_nb, ENDPOINT_TYPE type, ENDPOINT_DIRECTION dir, uint8_t index = 0);

    /**
    * Retrieve an USBEndpoint by its index
    *
    * @param intf_nb interface number
    * @param index index of the USBEndpoint
    * @returns pointer on the USBEndpoint if found, NULL otherwise
    */
    USBEndpoint * getEndpoint(uint8_t intf_nb, uint8_t index);

    /**
    * Add a new interface to this device
    *
    * @param intf_nb interface number
    * @param intf_class interface class
    * @param intf_subclass interface subclass
    * @param intf_protocol interface protocol
    * @returns true if successful, false otherwise
    */
    bool addInterface(uint8_t intf_nb, uint8_t intf_class, uint8_t intf_subclass, uint8_t intf_protocol);

    /**
    * Disconnect the device by calling a callback function registered by a driver
    */
    void disconnect();

    void init(USBDeviceConnected* parent, uint8_t _port, bool _lowSpeed);
    void setAddress(uint8_t addr_) { addr = addr_; };
    void setVid(uint16_t vid_) { vid = vid_; };
    void setPid(uint16_t pid_) { pid = pid_; };
    void setClass(uint8_t device_class_) { device_class = device_class_; }
    void setSubClass(uint8_t device_subclass_) { device_subclass = device_subclass_; };
    void setProtocol(uint8_t pr) { proto = pr; };
    void setEnumerated() { enumerated = true; };
    void setNbIntf(uint8_t nb_intf) {nb_interf = nb_intf; };
    void setSpeed(bool _lowSpeed) { lowSpeed = _lowSpeed; }
    void setName(const char * name_, uint8_t intf_nb) { return; };
    void setEpCtl(USBEndpoint* ep) { ep_ctl = ep; }

    static int getNewAddress() {
        static int addr = 1;
        return addr++;
    }
    uint8_t getAddress() { return addr; };
    uint16_t getVid() { return vid; };
    uint16_t getPid() { return pid; };
    uint8_t getClass() { return device_class; };
    bool getSpeed() { return lowSpeed; }
    bool isEnumerated() { return enumerated; };
    USBEndpoint* getEpCtl() { return ep_ctl; }

private:
    USBDeviceConnected* hub_parent;
    mymap<int,INTERFACE*>intf;
    uint8_t port;
    uint16_t vid;
    uint16_t pid;
    uint8_t addr;
    uint8_t device_class;
    uint8_t device_subclass;
    uint8_t proto;
    bool lowSpeed;
    bool enumerated;
    uint8_t nb_interf;
    USBEndpoint* ep_ctl;
    void init();
};


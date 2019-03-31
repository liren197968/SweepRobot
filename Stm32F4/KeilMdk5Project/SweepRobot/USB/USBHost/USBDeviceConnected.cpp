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

#include "USBDeviceConnected.h"
#include "dbg.h"

USBDeviceConnected::USBDeviceConnected() {
    init();
}

void USBDeviceConnected::init() {
    port = 0;
    vid = 0;
    pid = 0;
    nb_interf = 0;
    enumerated = false;
    device_class = 0;
    device_subclass = 0;
    proto = 0;
    lowSpeed = false;
    hub_parent = NULL;
}

bool USBDeviceConnected::addInterface(uint8_t intf_nb, uint8_t intf_class, uint8_t intf_subclass, uint8_t intf_protocol) {
    USB_DBG("intf_nb=%d", intf_nb);
    if (intf.count(intf_nb) > 0) {
        return false;
    }
    intf[intf_nb] = new INTERFACE(intf_class, intf_subclass, intf_protocol);
    return true;
}

bool USBDeviceConnected::addEndpoint(uint8_t intf_nb, USBEndpoint * ept) {
    if (intf.count(intf_nb) > 0) {
        intf[intf_nb]->ep.push_back(ept);
        return true;
    }
    return false;
}

void USBDeviceConnected::init(USBDeviceConnected* parent, uint8_t port_, bool lowSpeed_) {
    USB_DBG("init dev: %p", this);
    init();
    hub_parent = parent;
    port = port_;
    lowSpeed = lowSpeed_;
}

void USBDeviceConnected::disconnect() {
    //for(int i = 0; i < MAX_INTF; i++) {
    //    intf[i].detach.call();
    //}
    //init();
}


USBEndpoint * USBDeviceConnected::getEndpoint(uint8_t intf_nb, ENDPOINT_TYPE type, ENDPOINT_DIRECTION dir, uint8_t index) {
    USB_DBG("intf_nb=%d", intf_nb);
    USB_TEST_ASSERT(intf.count(intf_nb) > 0);
    INTERFACE* inter = intf[intf_nb];
    for (int i = 0; i < inter->ep.size(); i++) {
        if ((inter->ep[i]->getType() == type) && (inter->ep[i]->getDir() == dir)) {
            if(index) {
                index--;
            } else {
                return inter->ep[i];
            }
        }
    }
    return NULL;
}

USBEndpoint * USBDeviceConnected::getEndpoint(uint8_t intf_nb, uint8_t index) {
    USB_TEST_ASSERT(intf.count(intf_nb) > 0);
    return intf[intf_nb]->ep[index];
}


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
//#include "mbed.h"
#include "USBHALHost.h"
#include "USBDeviceConnected.h"
#include "IUSBEnumerator.h"
#include "USBHostConf.h"
#include "dbg.h"
#include "myvector.h"

/**
* USBHost class
*   This class is a singleton. All drivers have a reference on the static USBHost instance
*/
class USBHost : public USBHALHost {
public:
    /**
    * Static method to create or retrieve the single USBHost instance
    */
    static USBHost* getHostInst();

    /**
    * Control read: setup stage, data stage and status stage
    *
    * @param dev the control read will be done for this device
    * @param requestType request type
    * @param request request
    * @param value value
    * @param index index
    * @param buf pointer on a buffer where will be store the data received
    * @param len length of the transfer
    *
    * @returns status of the control read
    */
    USB_TYPE controlRead(USBDeviceConnected * dev, uint8_t requestType, uint8_t request, uint32_t value, uint32_t index, uint8_t * buf, uint32_t len);

    /**
    * Control write: setup stage, data stage and status stage
    *
    * @param dev the control write will be done for this device
    * @param requestType request type
    * @param request request
    * @param value value
    * @param index index
    * @param buf pointer on a buffer which will be written
    * @param len length of the transfer
    *
    * @returns status of the control write
    */
    USB_TYPE controlWrite(USBDeviceConnected * dev, uint8_t requestType, uint8_t request, uint32_t value, uint32_t index, uint8_t * buf, uint32_t len);

    /**
    * Bulk read
    *
    * @param dev the bulk transfer will be done for this device
    * @param ep USBEndpoint which will be used to read a packet
    * @param buf pointer on a buffer where will be store the data received
    * @param len length of the transfer
    * @param blocking if true, the read is blocking (wait for completion)
    *
    * @returns status of the bulk read
    */
    USB_TYPE bulkRead(USBDeviceConnected * dev, USBEndpoint * ep, uint8_t * buf, uint32_t len, bool blocking = true);

    /**
    * Bulk write
    *
    * @param dev the bulk transfer will be done for this device
    * @param ep USBEndpoint which will be used to write a packet
    * @param buf pointer on a buffer which will be written
    * @param len length of the transfer
    * @param blocking if true, the write is blocking (wait for completion)
    *
    * @returns status of the bulk write
    */
    USB_TYPE bulkWrite(USBDeviceConnected * dev, USBEndpoint * ep, uint8_t * buf, uint32_t len, bool blocking = true);

    /**
    * Interrupt read
    *
    * @param dev the interrupt transfer will be done for this device
    * @param ep USBEndpoint which will be used to write a packet
    * @param buf pointer on a buffer which will be written
    * @param len length of the transfer
    * @param blocking if true, the read is blocking (wait for completion)
    *
    * @returns status of the interrupt read
    */
    USB_TYPE interruptRead(USBDeviceConnected * dev, USBEndpoint * ep, uint8_t * buf, uint32_t len, bool blocking = true);

    /**
    * Interrupt write
    *
    * @param dev the interrupt transfer will be done for this device
    * @param ep USBEndpoint which will be used to write a packet
    * @param buf pointer on a buffer which will be written
    * @param len length of the transfer
    * @param blocking if true, the write is blocking (wait for completion)
    *
    * @returns status of the interrupt write
    */
    USB_TYPE interruptWrite(USBDeviceConnected * dev, USBEndpoint * ep, uint8_t * buf, uint32_t len, bool blocking = true);

    /**
    * Isochronous read
    *
    * @param dev the isochronous transfer will be done for this device
    * @param ep USBEndpoint which will be used to write a packet
    * @param buf pointer on a buffer which will be written
    * @param len length of the transfer
    * @param blocking if true, the read is blocking (wait for completion)
    *
    * @returns status of the interrupt read
    */
    USB_TYPE isochronousRead(USBDeviceConnected* dev, USBEndpoint* ep, uint8_t* buf, uint32_t len, bool blocking = true);

    /**
    * Enumerate a device.
    *
    * @param dev device which will be enumerated
    *
    * @returns status of the enumeration
    */
    USB_TYPE enumerate(USBDeviceConnected * dev, IUSBEnumerator* pEnumerator);

    /**
    * Get a device
    *
    * @param index index of the device which will be returned
    *
    * @returns pointer on the "index" device
    */
    USBDeviceConnected * getDevice(uint8_t index) {
        return index < DeviceLists.size() ? DeviceLists[index] : NULL;
    }

    /**
     *  register a driver into the host associated with a callback function called when the device is disconnected
     *
     *  @param dev device
     *  @param intf interface number
     *  @param tptr pointer to the object to call the member function on
     *  @param mptr pointer to the member function to be called
     */
    template<typename T>
    void registerDriver(USBDeviceConnected * dev, uint8_t intf, T* tptr, void (T::*mptr)(void)) {
    }

    // KL46Z-USBHost extensions
    int interruptReadNB(USBEndpoint* ep, uint8_t* data, int size);
    int bulkReadNB(USBEndpoint*ep, uint8_t* data, int size);
    int isochronousReadNB(USBEndpoint*ep, uint8_t* data, int size);

    /**
     * non-blocking processing
     */
    static void poll();

private:
    USBHost();
    static USBHost* inst;
    virtual bool addDevice(USBDeviceConnected* parent, int port, bool lowSpeed);
    void root_enumeration(USBDeviceConnected* dev);
    void parseConfDescr(USBDeviceConnected* dev, uint8_t* conf_descr, uint32_t len, IUSBEnumerator* pEnumerator);
    myvector<USBDeviceConnected*>DeviceLists;
    void task();
    EndpointQueue ep_queue;

    // USB HUB
    bool Hub(USBDeviceConnected* dev);
    int SetPortPower(USBDeviceConnected* dev, int port);
    int ClearPortPower(USBDeviceConnected* dev, int port);
    int PortReset(USBDeviceConnected* dev, int port);
    int SetPortFeature(USBDeviceConnected* dev, int feature, int index);
    int ClearPortFeature(USBDeviceConnected* dev, int feature, int index);
    int SetPortReset(USBDeviceConnected* dev, int port);
    int GetPortStatus(USBDeviceConnected* dev, int port, uint32_t* status);
};



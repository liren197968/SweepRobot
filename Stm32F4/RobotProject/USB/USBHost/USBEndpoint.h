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
//#include "FunctionPointer.h"
#include "USBHostTypes.h"
#include "USBDeviceConnected.h"

class USBDeviceConnected;

/**
* USBEndpoint class
*/
class USBEndpoint {
public:
    /**
    * Constructor
    */
    USBEndpoint(USBDeviceConnected* _dev) {
        init(CONTROL_ENDPOINT, IN, 8, 0);
        dev = _dev;
        pData = NULL;
    }

    /**
    * Initialize an endpoint
    *
    * @param type endpoint type
    * @param dir endpoint direction
    * @param size endpoint size
    * @param ep_number endpoint number
    */
    void init(ENDPOINT_TYPE _type, ENDPOINT_DIRECTION _dir, uint32_t size, uint8_t ep_number) {
        setState(USB_TYPE_FREE);
        setType(_type);
        dir = _dir;
        MaxPacketSize = size;
        address = ep_number;
        data01_toggle = DATA0;
    }

    void ohci_init(uint8_t frameCount, uint8_t queueLimit) {
        ohci.frameCount = frameCount;
        ohci.queueLimit = queueLimit;
    }

    /**
     *  Attach a member function to call when a transfer is finished
     *
     *  @param tptr pointer to the object to call the member function on
     *  @param mptr pointer to the member function to be called
     */
    template<typename T>
    inline void attach(T* tptr, void (T::*mptr)(void)) {
        if((mptr != NULL) && (tptr != NULL)) {
            rx.attach(tptr, mptr);
        }
    }

    /**
     * Attach a callback called when a transfer is finished
     *
     * @param fptr function pointer
     */
    inline void attach(void (*fptr)(void)) {
        if(fptr != NULL) {
            rx.attach(fptr);
        }
    }

    /**
    * Call the handler associted to the end of a transfer
    */
    inline void call() {
        rx.call();
    };

    void setType(ENDPOINT_TYPE _type) { type = _type; }
    void setState(USB_TYPE st) { state = st; }
    void setDevice(USBDeviceConnected* _dev) { dev = _dev; }
    void setBuffer(uint8_t* buf, int size) { buf_start = buf, buf_size = size; }
    void setLengthTransferred(int len) { transferred = len; };
    void setAddress(int addr) { address = addr; }
    void setSize(int size) { MaxPacketSize = size; }
    void setData01(uint8_t data01) { data01_toggle = data01; }
    void setNextEndpoint(USBEndpoint* ep) { nextEp = ep; };
    template<class T>
    void setHALData(T data) { pData = data; }

    USBDeviceConnected* getDevice() { return dev; }
    ENDPOINT_TYPE getType() { return type; };
    USB_TYPE getState() { return state; }
    int getLengthTransferred() { return transferred; }
    uint8_t *getBufStart() { return buf_start; }
    int getBufSize() { return buf_size; }
    uint8_t getAddress(){ return address; };
    int getSize() { return MaxPacketSize; }
    ENDPOINT_DIRECTION getDir() { return dir; }
    uint8_t getData01() { return data01_toggle; }
    void toggleData01() {
        data01_toggle = (data01_toggle == DATA0) ? DATA1 : DATA0;
    }
    USBEndpoint* nextEndpoint() { return nextEp; };
    template<class T>
    T getHALData() { return reinterpret_cast<T>(pData); }

    struct {
        uint8_t queueLimit;
        uint8_t frameCount; // 1-8
    } ohci;
private:
    USBEndpoint(){}
    ENDPOINT_TYPE type;
    USB_TYPE state;
    ENDPOINT_DIRECTION dir;
    USBDeviceConnected* dev;
    uint8_t data01_toggle; // DATA0,DATA1
    uint8_t address;
    int transferred;
    uint8_t * buf_start;
    int buf_size;
    FunctionPointer rx;
    int MaxPacketSize;
    USBEndpoint* nextEp;
    void* pData;
};

class EndpointQueue {
public:
    EndpointQueue():head(NULL),tail(NULL) {}
    void push(USBEndpoint* ep) {
        if (head) {
            tail->setNextEndpoint(ep);
        } else {
            head = ep;
        }
        tail = ep;
        ep->setNextEndpoint(NULL);
    }
    USBEndpoint* pop() {
        USBEndpoint* ep = head;
        if (ep) {
            head = ep->nextEndpoint();
        }
        return ep;
    }
    bool empty() { return head == NULL; }

private:
    USBEndpoint* head;
    USBEndpoint* tail;
};




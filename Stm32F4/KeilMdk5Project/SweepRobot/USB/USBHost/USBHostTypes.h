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

#ifndef USB_INC_H
#define USB_INC_H

#include <stdint.h>
//#include "mbed.h"
//#include "toolchain.h"

enum USB_TYPE {
    USB_TYPE_OK = 0,

    // completion code
    USB_TYPE_CRC_ERROR = 1,
    USB_TYPE_BIT_STUFFING_ERROR = 2,
    USB_TYPE_DATA_TOGGLE_MISMATCH_ERROR = 3,
    USB_TYPE_STALL_ERROR = 4,
    USB_TYPE_DEVICE_NOT_RESPONDING_ERROR = 5,
    USB_TYPE_PID_CHECK_FAILURE_ERROR = 6,
    USB_TYPE_UNEXPECTED_PID_ERROR = 7,
    USB_TYPE_DATA_OVERRUN_ERROR = 8,
    USB_TYPE_DATA_UNDERRUN_ERROR = 9,
    USB_TYPE_RESERVED = 9,
    USB_TYPE_RESERVED_ = 10,
    USB_TYPE_BUFFER_OVERRUN_ERROR = 12,
    USB_TYPE_BUFFER_UNDERRUN_ERROR = 13,

    // general usb state
    USB_TYPE_DISCONNECTED = 14,
    USB_TYPE_FREE = 15,
    USB_TYPE_IDLE = 16,
    USB_TYPE_PROCESSING = 17,

    USB_TYPE_ERROR = 18,
};


enum ENDPOINT_DIRECTION {
    OUT = 1,
    IN
};

enum ENDPOINT_TYPE {
    CONTROL_ENDPOINT = 0,
    ISOCHRONOUS_ENDPOINT,
    BULK_ENDPOINT,
    INTERRUPT_ENDPOINT
};

#define AUDIO_CLASS     0x01
#define CDC_CLASS       0x02
#define HID_CLASS       0x03
#define MSD_CLASS       0x08
#define HUB_CLASS       0x09
#define SERIAL_CLASS    0x0A

#define  DEVICE_DESCRIPTOR                     (1)
#define  CONFIGURATION_DESCRIPTOR              (2)
#define  INTERFACE_DESCRIPTOR                  (4)
#define  ENDPOINT_DESCRIPTOR                   (5)
#define  HID_DESCRIPTOR                        (33)

//  ----------- Control RequestType Fields  ----------- 
#define  USB_DEVICE_TO_HOST         0x80
#define  USB_HOST_TO_DEVICE         0x00
#define  USB_REQUEST_TYPE_CLASS     0x20
#define  USB_REQUEST_TYPE_STANDARD  0x00
#define  USB_RECIPIENT_DEVICE       0x00
#define  USB_RECIPIENT_INTERFACE    0x01
#define  USB_RECIPIENT_ENDPOINT     0x02

// -------------- USB Standard Requests  -------------- 
#define  GET_STATUS                 0x00
#define  SET_FEATURE                0x03
#define  SET_ADDRESS                0x05
#define  GET_DESCRIPTOR             0x06
#define  SET_CONFIGURATION          0x09
#define  SET_INTERFACE              0x0b
#define  CLEAR_FEATURE              0x01

// -------------- USB Descriptor Length  -------------- 
#define DEVICE_DESCRIPTOR_LENGTH            0x12
#define CONFIGURATION_DESCRIPTOR_LENGTH     0x09

// PID
#define DATA0 0x03
#define DATA1 0x0b
#define ACK   0x02
#define STALL 0x0e
#define NAK   0x0a

#pragma pack(push,1)
typedef struct {
    uint8_t bLength;            
    uint8_t bDescriptorType;    
    uint16_t bcdUSB;            
    uint8_t bDeviceClass;       
    uint8_t bDeviceSubClass;    
    uint8_t bDeviceProtocol;    
    uint8_t bMaxPacketSize;     
    uint16_t idVendor;          
    uint16_t idProduct;         
    uint16_t bcdDevice;         
    uint8_t iManufacturer;      
    uint8_t iProduct;           
    uint8_t iSerialNumber;      
    uint8_t bNumConfigurations; 
} PACKED DeviceDescriptor;

typedef struct {
    uint8_t bLength;               
    uint8_t bDescriptorType;       
    uint16_t wTotalLength;         
    uint8_t bNumInterfaces;        
    uint8_t bConfigurationValue;   
    uint8_t iConfiguration;        
    uint8_t bmAttributes;          
    uint8_t bMaxPower;             
} PACKED ConfigurationDescriptor; 

typedef struct {
    uint8_t bLength;                 
    uint8_t bDescriptorType;   
    uint8_t bInterfaceNumber;  
    uint8_t bAlternateSetting; 
    uint8_t bNumEndpoints;     
    uint8_t bInterfaceClass;   
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;        
} InterfaceDescriptor; 

typedef struct {
    uint8_t bLength;          
    uint8_t bDescriptorType;  
    uint8_t bEndpointAddress; 
    uint8_t bmAttributes;     
    uint16_t wMaxPacketSize;  
    uint8_t bInterval;        
} EndpointDescriptor;

typedef struct {
    uint8_t bDescLength;      
    uint8_t bDescriptorType;  
    uint8_t bNbrPorts;        
    uint16_t wHubCharacteristics;
    uint8_t bPwrOn2PwrGood;   
    uint8_t bHubContrCurrent; 
    uint8_t DeviceRemovable;  
    uint8_t PortPweCtrlMak;   
} HubDescriptor;              
#pragma pack(pop)

#endif


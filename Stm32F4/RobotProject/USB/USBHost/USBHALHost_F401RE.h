#pragma once
#include "mbed.h"
#include "USBHostTypes.h"
#include "USBEndpoint.h"

class HC {
    static const uint8_t DIR_IN  = 1;
    static const uint8_t DIR_OUT = 0;

public:
    HC();
    HC(int ch);
    ~HC();
    HAL_StatusTypeDef Init(uint8_t epnum, uint8_t dev_address, uint8_t speed, uint8_t ep_type, uint16_t mps);
    HAL_StatusTypeDef SubmitRequest(uint8_t* pbuff, uint16_t length, bool setup = false);
    HCD_URBStateTypeDef GetURBState();
    HCD_HCStateTypeDef GetState();
    uint32_t GetXferCount();
    void SetToggle(uint8_t toggle);

    static uint8_t slot;

private:
    int _ch;
    uint8_t _ep_addr;
    uint8_t _ep_type;
};

class USBHALHost {
public:
    uint8_t LastStatus;
    uint8_t prev_LastStatus;

protected:
    USBHALHost();
    void init();
    virtual bool addDevice(USBDeviceConnected* parent, int port, bool lowSpeed) = 0;
    int token_setup(USBEndpoint* ep, SETUP_PACKET* setup, uint16_t wLength = 0);
    int token_iso_in(USBEndpoint* ep, uint8_t* data, int size);
    int multi_token_in(USBEndpoint* ep, uint8_t* data = NULL, size_t total = 0, bool block = true);
    int multi_token_out(USBEndpoint* ep, const uint8_t* data = NULL, size_t total = 0);
    void multi_token_inNB(USBEndpoint* ep, uint8_t* data, int size);
    USB_TYPE multi_token_inNB_result(USBEndpoint* ep);
    void setToggle(USBEndpoint* ep, uint8_t toggle);

private:
    int token_in(USBEndpoint* ep, uint8_t* data = NULL, int size = 0, int retryLimit = 10);
    int token_out(USBEndpoint* ep, const uint8_t* data = NULL, int size = 0, int retryLimit = 10);
    int token_ctl_in(USBEndpoint* ep, uint8_t* data, int size, int retryLimit);
    int token_int_in(USBEndpoint* ep, uint8_t* data, int size);
    int token_blk_in(USBEndpoint* ep, uint8_t* data, int size, int retryLimit);
    int token_ctl_out(USBEndpoint* ep, const uint8_t* data, int size, int retryLimit);
    int token_int_out(USBEndpoint* ep, const uint8_t* data, int size);
    int token_blk_out(USBEndpoint* ep, const uint8_t* data, int size, int retryLimit);
    bool wait_attach();
    static USBHALHost * instHost;
};



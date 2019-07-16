//#include "mbed.h"
#include <stdint.h>

struct SETUP_PACKET {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
    SETUP_PACKET(uint8_t RequestType, uint8_t Request, uint16_t Value, uint16_t Index, uint16_t Length) {
        bmRequestType = RequestType;
        bRequest = Request;
        wValue = Value;
        wIndex = Index;
        wLength = Length;
    }
};

#if defined(TARGET_NUCLEO_F401RE)||defined(TARGET_NUCLEO_F411RE)||defined(TARGET_NUCLEO_F446RE)
#include "USBHALHost_F401RE.h"
#elif defined(TARGET_KL46Z)||defined(TARGET_KL25Z)||defined(TARGET_K64F)
#include "USBHALHost_KL46Z.h"
#elif defined(TARGET_LPC4088)||defined(TARGET_LPC1768)
#include "USBHALHost_LPC4088.h"
#else
//#error "target error"
#endif

#ifndef  CTASSERT
template <bool>struct CtAssert;
template <>struct CtAssert<true> {};
#define CTASSERT(A) CtAssert<A>();
#endif // CTASSERT




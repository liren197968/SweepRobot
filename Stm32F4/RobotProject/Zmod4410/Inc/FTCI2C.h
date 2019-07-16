/*++

Copyright (c) 2005 Future Technology Devices International Ltd.

Module Name:

    ftci2c.h

Abstract:

    API DLL for FT2232H and FT4232H Hi-Speed Dual Device and FT2232D Dual Device setup to simulate the
    Inter-Integrated Circuit(I2C) synchronous serial protocol.
    FTCI2C library definitions

Environment:

    kernel & user mode

Revision History:

    23/03/05    kra   Created.
	  19/10/05	  ana   Removed control write if only 1 bytes to write fixed PAGE_WRITE acknowledge
    03/06/08    kra   Added two new functions ie I2C_SetGPIOs and I2C_GetGPIOs and two 
                      structures ie FTC_INPUT_OUTPUT_PINS and FTC_LOW_HIGH_PINS and two
                      error constants FTC_NULL_INPUT_BUFFER_POINTER and FTC_NULL_INPUT_OUTPUT_BUFFER_POINTER to 
                      control the output states ie high(true) or low(false) of the 4 general purpose higher
                      input/output ACBUS 0-3/GPIOH 0-3) pins
    01/08/08    kra   Added new functions for FT2232H and FT4232H hi-speed devices.
    20/08/08    kra   Added new function I2C_CloseDevice.

--*/


#ifndef FTCI2C_H
#define FTCI2C_H


// The following ifdef block is the standard way of creating macros
// which make exporting from a DLL simpler.  All files within this DLL
// are compiled with the FTCI2C_EXPORTS symbol defined on the command line.
// This symbol should not be defined on any project that uses this DLL.
// This way any other project whose source files include this file see
// FTCI2C_API functions as being imported from a DLL, whereas this DLL
// sees symbols defined with this macro as being exported.

#ifdef RSRFTCI2C_EXPORTS
#define FTCI2C_API __declspec(dllexport)
#else
#define FTCI2C_API __declspec(dllimport)
#endif

typedef DWORD FTC_HANDLE;
typedef ULONG FTC_STATUS;

// Hi-speed device types
enum {
	FT2232H_DEVICE_TYPE = 1,
	FT4232H_DEVICE_TYPE = 2
};

#define NO_WRITE_TYPE 0
#define BYTE_WRITE_TYPE 1
#define PAGE_WRITE_TYPE 2

#define NO_WRITE_READ_TYPE 0
#define BYTE_READ_TYPE 1
#define BLOCK_READ_TYPE 2

#define STANDARD_MODE 1
#define FAST_MODE 2
#define STRETCH_DATA_MODE 4

#define FTC_SUCCESS 0 // FT_OK
#define FTC_INVALID_HANDLE 1 // FT_INVALID_HANDLE
#define FTC_DEVICE_NOT_FOUND 2 //FT_DEVICE_NOT_FOUND
#define FTC_DEVICE_NOT_OPENED 3 //FT_DEVICE_NOT_OPENED
#define FTC_IO_ERROR 4 //FT_IO_ERROR
#define FTC_INSUFFICIENT_RESOURCES 5 // FT_INSUFFICIENT_RESOURCES

#define FTC_FAILED_TO_COMPLETE_COMMAND 20          // cannot change, error code mapped from FT2232c classes
#define FTC_FAILED_TO_SYNCHRONIZE_DEVICE_MPSSE 21  // cannot change, error code mapped from FT2232c classes
#define FTC_INVALID_DEVICE_NAME_INDEX 22           // cannot change, error code mapped from FT2232c classes
#define FTC_NULL_DEVICE_NAME_BUFFER_POINTER 23     // cannot change, error code mapped from FT2232c classes 
#define FTC_DEVICE_NAME_BUFFER_TOO_SMALL 24        // cannot change, error code mapped from FT2232c classes
#define FTC_INVALID_DEVICE_NAME 25                 // cannot change, error code mapped from FT2232c classes
#define FTC_INVALID_LOCATION_ID 26                 // cannot change, error code mapped from FT2232c classes
#define FTC_DEVICE_IN_USE 27                       // cannot change, error code mapped from FT2232c classes
#define FTC_TOO_MANY_DEVICES 28                    // cannot change, error code mapped from FT2232c classes

#define FTC_NULL_CHANNEL_BUFFER_POINTER 29         // cannot change, error code mapped from FT2232h classes 
#define FTC_CHANNEL_BUFFER_TOO_SMALL 30            // cannot change, error code mapped from FT2232h classes
#define FTC_INVALID_CHANNEL 31                     // cannot change, error code mapped from FT2232h classes
#define FTC_INVALID_TIMER_VALUE 32                 // cannot change, error code mapped from FT2232h classes

#define FTC_EXTERNAL_DEVICE_NOT_FOUND 33
#define FTC_INVALID_CLOCK_DIVISOR 34
#define FTC_NULL_INPUT_BUFFER_POINTER 35
#define FTC_NULL_INPUT_OUTPUT_BUFFER_POINTER 36
#define FTC_NULL_CONTROL_DATA_BUFFER_POINTER 37
#define FTC_INVALID_NUMBER_CONTROL_BYTES 38
#define FTC_CONTROL_ACKNOWLEDGE_TIMEOUT 39
#define FTC_NULL_WRITE_DATA_BUFFER_POINTER 40
#define FTC_INVALID_NUMBER_DATA_BYTES_WRITE 41
#define FTC_DATA_ACKNOWLEDGE_TIMEOUT 42
#define FTC_INVALID_WRITE_TYPE 43
#define FTC_NUMBER_BYTES_TOO_SMALL_PAGE_WRITE 44
#define FTC_NULL_PAGE_WRITE_BUFFER_POINTER 45
#define FTC_NULL_READ_DATA_BUFFER_POINTER 46
#define FTC_INVALID_NUMBER_DATA_BYTES_READ 47
#define FTC_INVALID_READ_TYPE 48
#define FTC_INVALID_COMMS_MODE 49
#define FTC_NULL_CLOSE_FINAL_STATE_BUFFER_POINTER 50
#define FTC_NULL_DLL_VERSION_BUFFER_POINTER 51
#define FTC_DLL_VERSION_BUFFER_TOO_SMALL 52
#define FTC_NULL_LANGUAGE_CODE_BUFFER_POINTER 53
#define FTC_NULL_ERROR_MESSAGE_BUFFER_POINTER 54
#define FTC_ERROR_MESSAGE_BUFFER_TOO_SMALL 55
#define FTC_INVALID_LANGUAGE_CODE 56
#define FTC_INVALID_STATUS_CODE 57

#ifdef __cplusplus
extern "C" {
#endif

FTCI2C_API
FTC_STATUS WINAPI I2C_GetNumDevices(LPDWORD lpdwNumDevices);

FTCI2C_API
FTC_STATUS WINAPI I2C_GetNumHiSpeedDevices(LPDWORD lpdwNumHiSpeedDevices);

FTCI2C_API
FTC_STATUS WINAPI I2C_GetDeviceNameLocID(DWORD dwDeviceNameIndex, LPSTR lpDeviceNameBuffer, DWORD dwBufferSize, LPDWORD lpdwLocationID);

FTCI2C_API
FTC_STATUS WINAPI I2C_GetHiSpeedDeviceNameLocIDChannel(DWORD dwDeviceNameIndex, LPSTR lpDeviceNameBuffer, DWORD dwBufferSize, LPDWORD lpdwLocationID, LPSTR lpChannel, DWORD dwChannelBufferSize, LPDWORD lpdwHiSpeedDeviceType);

FTCI2C_API
FTC_STATUS WINAPI I2C_Open(FTC_HANDLE *pftHandle);

FTCI2C_API
FTC_STATUS WINAPI I2C_OpenEx(LPSTR lpDeviceName, DWORD dwLocationID, FTC_HANDLE *pftHandle);

FTCI2C_API
FTC_STATUS WINAPI I2C_OpenHiSpeedDevice(LPSTR lpDeviceName, DWORD dwLocationID, LPSTR lpChannel, FTC_HANDLE *pftHandle);

FTCI2C_API
FTC_STATUS WINAPI I2C_GetHiSpeedDeviceType(FTC_HANDLE ftHandle, LPDWORD lpdwHiSpeedDeviceType);

FTCI2C_API
FTC_STATUS WINAPI I2C_Close(FTC_HANDLE ftHandle);

typedef struct Ft_Close_Final_State_Pins{
  BOOL  bTCKPinState;
  BOOL  bTCKPinActiveState;
  BOOL  bTDIPinState;
  BOOL  bTDIPinActiveState;
  BOOL  bTMSPinState;
  BOOL  bTMSPinActiveState;
}FTC_CLOSE_FINAL_STATE_PINS, *PFTC_CLOSE_FINAL_STATE_PINS;

FTCI2C_API
FTC_STATUS WINAPI I2C_CloseDevice(FTC_HANDLE ftHandle, PFTC_CLOSE_FINAL_STATE_PINS pCloseFinalStatePinsData);

FTCI2C_API
FTC_STATUS WINAPI I2C_InitDevice(FTC_HANDLE ftHandle, DWORD dwClockDivisor);

FTCI2C_API
FTC_STATUS WINAPI I2C_TurnOnDivideByFiveClockingHiSpeedDevice(FTC_HANDLE ftHandle);

FTCI2C_API
FTC_STATUS WINAPI I2C_TurnOffDivideByFiveClockingHiSpeedDevice(FTC_HANDLE ftHandle);

FTCI2C_API
FTC_STATUS WINAPI I2C_TurnOnThreePhaseDataClockingHiSpeedDevice(FTC_HANDLE ftHandle);

FTCI2C_API
FTC_STATUS WINAPI I2C_TurnOffThreePhaseDataClockingHiSpeedDevice(FTC_HANDLE ftHandle);

FTCI2C_API
FTC_STATUS WINAPI I2C_SetDeviceLatencyTimer(FTC_HANDLE ftHandle, BYTE timerValue);

FTCI2C_API
FTC_STATUS WINAPI I2C_GetDeviceLatencyTimer(FTC_HANDLE ftHandle, LPBYTE lpTimerValue);

FTCI2C_API
FTC_STATUS WINAPI I2C_GetClock(DWORD dwClockDivisor, LPDWORD lpdwClockFrequencyHz);

FTCI2C_API
FTC_STATUS WINAPI I2C_GetHiSpeedDeviceClock(DWORD dwClockDivisor, LPDWORD lpdwClockFrequencyHz);

FTCI2C_API
FTC_STATUS WINAPI I2C_SetClock(FTC_HANDLE ftHandle, DWORD dwClockDivisor, LPDWORD lpdwClockFrequencyHz);

FTCI2C_API
FTC_STATUS WINAPI I2C_SetLoopback(FTC_HANDLE ftHandle, BOOL bLoopbackState);

FTCI2C_API
FTC_STATUS WINAPI I2C_SetMode(FTC_HANDLE ftHandle, DWORD dwCommsMode);

typedef struct Ft_Input_Output_Pins{
  BOOL  bPin1InputOutputState;
  BOOL  bPin1LowHighState;
  BOOL  bPin2InputOutputState;
  BOOL  bPin2LowHighState;
  BOOL  bPin3InputOutputState;
  BOOL  bPin3LowHighState;
  BOOL  bPin4InputOutputState;
  BOOL  bPin4LowHighState;
}FTC_INPUT_OUTPUT_PINS, *PFTC_INPUT_OUTPUT_PINS;

FTCI2C_API
FTC_STATUS WINAPI I2C_SetGPIOs(FTC_HANDLE ftHandle, PFTC_INPUT_OUTPUT_PINS pHighInputOutputPinsData);

typedef struct FTH_Input_Output_Pins{
  BOOL  bPin1InputOutputState;
  BOOL  bPin1LowHighState;
  BOOL  bPin2InputOutputState;
  BOOL  bPin2LowHighState;
  BOOL  bPin3InputOutputState;
  BOOL  bPin3LowHighState;
  BOOL  bPin4InputOutputState;
  BOOL  bPin4LowHighState;
  BOOL  bPin5InputOutputState;
  BOOL  bPin5LowHighState;
  BOOL  bPin6InputOutputState;
  BOOL  bPin6LowHighState;
  BOOL  bPin7InputOutputState;
  BOOL  bPin7LowHighState;
  BOOL  bPin8InputOutputState;
  BOOL  bPin8LowHighState;
}FTH_INPUT_OUTPUT_PINS, *PFTH_INPUT_OUTPUT_PINS;

FTCI2C_API
FTC_STATUS WINAPI I2C_SetHiSpeedDeviceGPIOs(FTC_HANDLE ftHandle, BOOL bControlLowInputOutputPins,
                                            PFTC_INPUT_OUTPUT_PINS pLowInputOutputPinsData,
                                            BOOL bControlHighInputOutputPins,
                                            PFTH_INPUT_OUTPUT_PINS pHighInputOutputPinsData);

typedef struct Ft_Low_High_Pins{
  BOOL  bPin1LowHighState;
  BOOL  bPin2LowHighState;
  BOOL  bPin3LowHighState;
  BOOL  bPin4LowHighState;
}FTC_LOW_HIGH_PINS, *PFTC_LOW_HIGH_PINS;

FTCI2C_API
FTC_STATUS WINAPI I2C_GetGPIOs(FTC_HANDLE ftHandle, PFTC_LOW_HIGH_PINS pHighPinsInputData);

typedef struct FTH_Low_High_Pins{
  BOOL  bPin1LowHighState;
  BOOL  bPin2LowHighState;
  BOOL  bPin3LowHighState;
  BOOL  bPin4LowHighState;
  BOOL  bPin5LowHighState;
  BOOL  bPin6LowHighState;
  BOOL  bPin7LowHighState;
  BOOL  bPin8LowHighState;
}FTH_LOW_HIGH_PINS, *PFTH_LOW_HIGH_PINS;

FTCI2C_API
FTC_STATUS WINAPI I2C_GetHiSpeedDeviceGPIOs(FTC_HANDLE ftHandle, BOOL bControlLowInputOutputPins,
                                            PFTC_LOW_HIGH_PINS pLowPinsInputData,
                                            BOOL bControlHighInputOutputPins,
                                            PFTH_LOW_HIGH_PINS pHighPinsInputData);

#define MAX_WRITE_CONTROL_BYTES_BUFFER_SIZE 256    // 256 bytes

typedef BYTE WriteControlByteBuffer[MAX_WRITE_CONTROL_BYTES_BUFFER_SIZE];
typedef WriteControlByteBuffer *PWriteControlByteBuffer;

typedef struct FTC_Page_Write_Data{
  DWORD  dwNumPages;
  DWORD  dwNumBytesPerPage;
}FTC_PAGE_WRITE_DATA, *PFTC_PAGE_WRITE_DATA;

#define MAX_WRITE_DATA_BYTES_BUFFER_SIZE 65536    // 64k bytes

typedef BYTE WriteDataByteBuffer[MAX_WRITE_DATA_BYTES_BUFFER_SIZE];
typedef WriteDataByteBuffer *PWriteDataByteBuffer;

FTCI2C_API
FTC_STATUS WINAPI I2C_Write(FTC_HANDLE ftHandle, PWriteControlByteBuffer pWriteControlBuffer,
                            DWORD dwNumControlBytesToWrite, BOOL bControlAcknowledge, DWORD dwControlAckTimeoutmSecs,
                            BOOL bStopCondition, DWORD dwDataWriteTypes, PWriteDataByteBuffer pWriteDataBuffer, DWORD dwNumDataBytesToWrite,
                            BOOL bDataAcknowledge, DWORD dwDataAckTimeoutmSecs, PFTC_PAGE_WRITE_DATA pPageWriteData);

#define MAX_READ_DATA_BYTES_BUFFER_SIZE 65536    // 64k bytes

typedef BYTE ReadDataByteBuffer[MAX_READ_DATA_BYTES_BUFFER_SIZE];
typedef ReadDataByteBuffer *PReadDataByteBuffer;

FTCI2C_API
FTC_STATUS WINAPI I2C_Read(FTC_HANDLE ftHandle, PWriteControlByteBuffer pWriteControlBuffer,
                           DWORD dwNumControlBytesToWrite, BOOL bControlAcknowledge, DWORD dwControlAckTimeoutmSecs,
                           DWORD dwDataReadTypes, PReadDataByteBuffer pReadDataBuffer, DWORD dwNumDataBytesToRead);

FTCI2C_API
FTC_STATUS WINAPI I2C_GetDllVersion(LPSTR lpDllVersionBuffer, DWORD dwBufferSize);

FTCI2C_API
FTC_STATUS WINAPI I2C_GetErrorCodeString(LPSTR lpLanguage, FTC_STATUS StatusCode,
                                         LPSTR lpErrorMessageBuffer, DWORD dwBufferSize);


#ifdef __cplusplus
}
#endif


#endif  /* FTCI2C_H */
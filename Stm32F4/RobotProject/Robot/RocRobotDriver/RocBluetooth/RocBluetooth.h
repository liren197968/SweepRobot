/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2018/12/20      1.0
********************************************************************************/
#ifndef __ROC_BLUETOOTH_H
#define __ROC_BLUETOOTH_H


#include <stdint.h>

#include "RocError.h"


#define ROC_BT_TXD_LENGTH           100
#define ROC_BT_RXD_LENGTH           100


ROC_RESULT RocBluetoothInit(void);
uint8_t RocBluetoothCtrlCmd_Get(void);
ROC_RESULT RocBluetoothRecvIsFinshed(void);
void RocBluetoothCtrlCmd_Set(uint8_t CtrlCmd);
void RocBluetoothData_Send(uint8_t *Buff, uint16_t DatLen);
void RocBluetoothReceiveCallback(UART_HandleTypeDef *Huart);


#endif


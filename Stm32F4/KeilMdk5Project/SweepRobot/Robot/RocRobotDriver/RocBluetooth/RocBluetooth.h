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


extern uint8_t g_BtRecvEnd;
extern uint8_t g_BtRxDatLen;
extern uint8_t g_BtTxBuffer[ROC_BT_TXD_LENGTH];
extern uint8_t g_BtRxBuffer[ROC_BT_RXD_LENGTH];


ROC_RESULT RocBluetoothInit(void);
void RocBluetoothCtrlCmd_Set(uint8_t CtrlCmd);
uint8_t RocBluetoothCtrlCmd_Get(void);
void RocBluetoothData_Send(uint8_t *Buff, uint16_t DatLen);
void RocBluetoothReceiveCallback(UART_HandleTypeDef *Huart);


#endif


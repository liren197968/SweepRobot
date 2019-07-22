/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/01/20      1.0
********************************************************************************/
#ifndef __ROC_REMOTE_CONTROL_H
#define __ROC_REMOTE_CONTROL_H


#include "RocError.h"


#define ROC_REMOTE_UART_CHANNEL         (&huart2)

#define ROC_REMOTE_MAX_NUM_LEN_SEND     12

#define ROC_JOYSTICK_FRAME_HEADER       0xFA
#define ROC_JOYSTICK_KEY_HEADER         0x01


typedef struct _ROC_REMOTE_CTRL_INPUT_s
{
    double              X;
    double              Y;
    double              Z;
    double              A;
    double              H;

}ROC_REMOTE_CTRL_INPUT_s;


ROC_RESULT RocRemoteControlInit(void);
ROC_RESULT RocRemoteRecvIsFinshed(void);
uint8_t* RocRemoteDataReceive(void);
void RocRemoteDataTransmit(uint8_t *Buff, uint16_t DatLen);
void RocRemoteReceiveCallback(UART_HandleTypeDef *Huart);


#endif


/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2018/12/16      1.0
********************************************************************************/
#ifndef __ROC_ROBOTCONTROL_H
#define __ROC_ROBOTCONTROL_H


#include "RocRemoteControl.h"
#include "RocRobotDhAlgorithm.h"


#define ROC_ROBOT_CONTROL_DEBUG


#define ROC_ROBOT_CTRL_TIME_LCD_TICK    10

#define ROC_ROBOT_CTRL_TRANSFORM_STEP   2
#define ROC_ROBOT_CTRL_TRANSFORM_DELAY  4

#define ROC_ROBOT_CTRL_LEG_LEFT_STEP    20


typedef enum _ROC_ROBOT_RUN_MODE_e
{
    ROC_ROBOT_RUN_MODE_CAR = 0,
    ROC_ROBOT_RUN_MODE_HEXAPOD,
    ROC_ROBOT_RUN_MODE_QUADRUPED,

}ROC_ROBOT_RUN_MODE_e;


typedef enum _ROC_ROBOT_CTRL_CMD_e
{
    ROC_ROBOT_CTRL_CMD_MOSTAND = 'C',
    ROC_ROBOT_CTRL_CMD_FORWARD = 'W',
    ROC_ROBOT_CTRL_CMD_BAKWARD = 'S',
    ROC_ROBOT_CTRL_CMD_LFCLOCK = 'A',
    ROC_ROBOT_CTRL_CMD_RGCLOCK = 'D',
    ROC_ROBOT_CTRL_CMD_LFRWARD = 'Y',
    ROC_ROBOT_CTRL_CMD_RFRWARD = 'G',
    ROC_ROBOT_CTRL_CMD_LBKWARD = 'B',
    ROC_ROBOT_CTRL_CMD_RBKWARD = 'R',
    ROC_ROBOT_CTRL_CMD_NUM = 9,

}ROC_ROBOT_CTRL_CMD_e;


typedef struct _ROC_ROBOT_CTRL_FlAG_s
{
    uint8_t FlagStatus[ROC_ROBOT_CTRL_CMD_NUM];

}ROC_ROBOT_CTRL_FlAG_s;

typedef struct _ROC_ROBOT_CTRL_TIME_s
{
    uint8_t     CtrlTimeIsReady;        // check if the robot control time is ready
    uint8_t     BatTimeIsReady;         // check if the battery checkt time is ready
    uint8_t     LcdTimeIsReady;         // check if the lcd show time is ready

}ROC_ROBOT_CTRL_TIME_s;

typedef struct _ROC_ROBOT_CTRL_s
{
    ROC_ROBOT_CTRL_FlAG_s    CtrlFlag;
    ROC_ROBOT_CTRL_TIME_s    CtrlTime;
    ROC_ROBOT_RUN_MODE_e     RunMode;
    ROC_REMOTE_CTRL_INPUT_s  RemoteCtrl;
    ROC_ROBOT_MOVE_CTRL_s    *MoveCtrl;
    float                    BatVoltage;

}ROC_ROBOT_CTRL_s;


void RocRobotInit(void);
void RocRobotMain(void);


#endif


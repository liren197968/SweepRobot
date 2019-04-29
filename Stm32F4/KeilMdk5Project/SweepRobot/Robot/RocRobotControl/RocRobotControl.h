/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2018/12/16      1.0
********************************************************************************/
#ifndef __ROC_ROBOTCONTROL_H
#define __ROC_ROBOTCONTROL_H


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
    ROC_ROBOT_CTRL_CMD_PARAMET = 'U',
    ROC_ROBOT_CTRL_CMD_TURNLDR = 'Q',
    ROC_ROBOT_CTRL_CMD_TURNRDR = 'E',
    ROC_ROBOT_CTRL_MEASURE_START,
    ROC_ROBOT_CTRL_MEASURE_STOP,

    ROC_ROBOT_CTRL_CMD_NUM = 13,
}ROC_ROBOT_CTRL_CMD_e;

typedef struct _ROC_ROBOT_CTRL_FlAG_s
{
    uint8_t CtrlFlag[ROC_ROBOT_CTRL_CMD_NUM];

}ROC_ROBOT_CTRL_FlAG_s;


void RocRobotInit(void);
void RocRobotMain(void);


#endif


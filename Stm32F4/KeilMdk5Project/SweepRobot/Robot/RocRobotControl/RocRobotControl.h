/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2018/12/16      1.0
********************************************************************************/
#ifndef __ROC_ROBOTCONTROL_H
#define __ROC_ROBOTCONTROL_H


#define ROC_ROBOT_DEFAULT_LEG_STEP                  40
#define ROC_ROBOT_DEFAULT_TURN_ANGLE                20
#define ROC_ROBOT_DEFAULT_FEET_LIFT                 20


typedef enum _ROC_ROBOT_CTRL_CMD_e
{
    ROC_ROBOT_WALK_MODE_CAR = 0,
    ROC_ROBOT_WALK_MODE_HEXAPOD = 1,

}ROC_ROBOT_CTRL_CMD_e;


typedef enum _ROC_ROBOT_WALK_MODE_e
{
    ROC_ROBOT_CTRL_CMD_MOSTAND = 'C',
    ROC_ROBOT_CTRL_CMD_FORWARD = 'W',
    ROC_ROBOT_CTRL_CMD_BAKWARD = 'S',
    ROC_ROBOT_CTRL_CMD_LFCLOCK = 'A',
    ROC_ROBOT_CTRL_CMD_RGCLOCK = 'D',
    ROC_ROBOT_CTRL_CMD_PARAMET = 'U',
    ROC_ROBOT_CTRL_CMD_CARFORD = 'B',
    ROC_ROBOT_CTRL_CMD_CARBAKD = 'R',
    ROC_ROBOT_CTRL_CMD_CARMODE = 'Y',
    ROC_ROBOT_CTRL_CMD_ROTMODE = 'G',
    ROC_ROBOT_CTRL_CMD_TURNLDR = 'Q',
    ROC_ROBOT_CTRL_CMD_TURNRDR = 'E',

    ROC_ROBOT_CTRL_CMD_NUM = 13,
}ROC_ROBOT_WALK_MODE_e;


void RocRobotInit(void);
void RocRobotMain(void);


#endif


/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/01/20      1.0
********************************************************************************/
#ifndef __ROC_REMOTE_CONTROL_H
#define __ROC_REMOTE_CONTROL_H


#include "RocError.h"


typedef struct _ROC_REMOTE_CTRL_INPUT_s
{
    double              X;
    double              Y;
    double              Z;
    double              A;
    double              H;

}ROC_REMOTE_CTRL_INPUT_s;


ROC_RESULT RocRemoteControlInit(void);


#endif

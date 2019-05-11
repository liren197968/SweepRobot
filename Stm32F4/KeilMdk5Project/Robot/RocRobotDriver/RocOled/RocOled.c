/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/05/02      1.0
********************************************************************************/

#include "RocLog.h"
#include "RocOled.h"

/*********************************************************************************
 *  Description:
 *              OLED init
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.05.02)
**********************************************************************************/
ROC_RESULT RocOledInit(void)
{
    ROC_RESULT Ret = RET_OK;

    if(RET_OK != Ret)
    {
        ROC_LOGE("OLED init is in error!");
    }

    return Ret;
}



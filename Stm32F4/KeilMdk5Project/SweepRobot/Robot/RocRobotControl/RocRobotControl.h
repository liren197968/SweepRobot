#ifndef __ROC_ROBOTCONTROL_H
#define __ROC_ROBOTCONTROL_H


#define ROC_ROBOT_DEFAULT_STEP_LENGTH       20

#define ROC_ROBOT_LEG_WIDTH                 118.3
#define ROC_ROBOT_LEG_HEIGHT                69

#define ROC_ROBOT_LEG_LIFT                  90
#define ROC_ROBOT_FEET_LIFT                 90


#define ROC_ROBOT_DOWN_ANGLE                15
#define ROC_ROBOT_ANGLE_TO_RADIAN           3.141593 / 180
#define ROC_ROBOT_ROTATE_ANGLE_TO_PWM       1024 / 220.0

#define ROC_ROBOT_WIDTH                     (ROC_ROBOT_A1 + ROC_ROBOT_A2 * cos(ROC_ROBOT_DOWN_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN) \
                                            + ROC_ROBOT_A4)
#define ROC_ROBOT_HEIGHT                    (ROC_ROBOT_A2 * sin(ROC_ROBOT_DOWN_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN) \
                                            + ROC_ROBOT_LEG_HEIGHT)

#define ROC_ROBOT_FOR_INIT_X                ROC_ROBOT_WIDTH / 2
#define ROC_ROBOT_FOR_INIT_Y                ROC_ROBOT_WIDTH * 0.866
#define ROC_ROBOT_FOR_INIT_Z                -ROC_ROBOT_HEIGHT
#define ROC_ROBOT_MID_INIT_X                ROC_ROBOT_WIDTH
#define ROC_ROBOT_MID_INIT_Y                0
#define ROC_ROBOT_MID_INIT_Z                -ROC_ROBOT_HEIGHT
#define ROC_ROBOT_BAK_INIT_X                ROC_ROBOT_WIDTH / 2
#define ROC_ROBOT_BAK_INIT_Y                ROC_ROBOT_WIDTH * 0.866
#define ROC_ROBOT_BAK_INIT_Z                -ROC_ROBOT_HEIGHT

#define ROC_ROBOT_FOR_HIP_INIT_ANGLE        60
#define ROC_ROBOT_FOR_LEG_INIT_ANGLE        0
#define ROC_ROBOT_FOR_FET_INIT_ANGLE        73.90
#define ROC_ROBOT_MID_HIP_INIT_ANGLE        0
#define ROC_ROBOT_MID_LEG_INIT_ANGLE        0
#define ROC_ROBOT_MID_FET_INIT_ANGLE        73.90
#define ROC_ROBOT_BAK_HIP_INIT_ANGLE        60
#define ROC_ROBOT_BAK_LEG_INIT_ANGLE        0
#define ROC_ROBOT_BAK_FET_INIT_ANGLE        73.90

#define ROC_ROBOT_RIG_FOR_HIP_CENTER        302
#define ROC_ROBOT_RIG_FOR_LEG_CENTER        170
#define ROC_ROBOT_RIG_FOR_FET_CENTER        332
#define ROC_ROBOT_RIG_MID_HIP_CENTER        301
#define ROC_ROBOT_RIG_MID_LEG_CENTER        153
#define ROC_ROBOT_RIG_MID_FET_CENTER        310
#define ROC_ROBOT_RIG_BAK_HIP_CENTER        307
#define ROC_ROBOT_RIG_BAK_LEG_CENTER        180
#define ROC_ROBOT_RIG_BAK_FET_CENTER        310
#define ROC_ROBOT_LEF_FOR_HIP_CENTER        306
#define ROC_ROBOT_LEF_FOR_LEG_CENTER        412
#define ROC_ROBOT_LEF_FOR_FET_CENTER        304
#define ROC_ROBOT_LEF_MID_HIP_CENTER        310
#define ROC_ROBOT_LEF_MID_LEG_CENTER        416
#define ROC_ROBOT_LEF_MID_FET_CENTER        304
#define ROC_ROBOT_LEF_BAK_HIP_CENTER        303
#define ROC_ROBOT_LEF_BAK_LEG_CENTER        400
#define ROC_ROBOT_LEF_BAK_FET_CENTER        310


void RocRobotInit(void);
void RocRobotMain(void);
void RocRobotRemoteControl(void);


#endif


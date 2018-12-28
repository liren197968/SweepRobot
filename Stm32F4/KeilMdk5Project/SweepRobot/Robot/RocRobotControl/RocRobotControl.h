#ifndef __ROC_ROBOTCONTROL_H
#define __ROC_ROBOTCONTROL_H


#define ROC_ROBOT_RIGHT_FRONT_LEG                   1
#define ROC_ROBOT_RIGHT_MIDDLE_LEG                  2
#define ROC_ROBOT_RIGHT_HIND_LEG                    3
#define ROC_ROBOT_LEFT_FRONT_LEG                    4
#define ROC_ROBOT_LEFT_MIDDLE_LEG                   5
#define ROC_ROBOT_LEFT_HIND_LEG                     6


#define ROC_ROBOT_RIG_FRO_HIP_CENTER                302
#define ROC_ROBOT_RIG_FRO_LEG_CENTER                170
#define ROC_ROBOT_RIG_FRO_FET_CENTER                332
#define ROC_ROBOT_RIG_MID_HIP_CENTER                301
#define ROC_ROBOT_RIG_MID_LEG_CENTER                153
#define ROC_ROBOT_RIG_MID_FET_CENTER                310
#define ROC_ROBOT_RIG_HIN_HIP_CENTER                307
#define ROC_ROBOT_RIG_HIN_LEG_CENTER                180
#define ROC_ROBOT_RIG_HIN_FET_CENTER                310
#define ROC_ROBOT_LEF_FRO_HIP_CENTER                306
#define ROC_ROBOT_LEF_FRO_LEG_CENTER                412
#define ROC_ROBOT_LEF_FRO_FET_CENTER                304
#define ROC_ROBOT_LEF_MID_HIP_CENTER                310
#define ROC_ROBOT_LEF_MID_LEG_CENTER                416
#define ROC_ROBOT_LEF_MID_FET_CENTER                304
#define ROC_ROBOT_LEF_HIN_HIP_CENTER                303
#define ROC_ROBOT_LEF_HIN_LEG_CENTER                400
#define ROC_ROBOT_LEF_HIN_FET_CENTER                310


#define ROC_ROBOT_DEFAULT_STEP_LENGTH               25
#define ROC_ROBOT_DEFAULT_FEET_LIFT                 40


#define ROC_ROBOT_CTRL_CMD_MOSTAND                  'C'
#define ROC_ROBOT_CTRL_CMD_FORWARD                  'W'
#define ROC_ROBOT_CTRL_CMD_BAKWARD                  'S'
#define ROC_ROBOT_CTRL_CMD_LFCLOCK                  'A'
#define ROC_ROBOT_CTRL_CMD_RGCLOCK                  'D'
#define ROC_ROBOT_CTRL_CMD_PARAMET                  'Y'

#define ROC_ROBOT_GAIT_FIVE

#if defined(ROC_ROBOT_GAIT_FOUR)
#define ROC_ROBOT_RUN_GAIT_NUM                      6
#elif defined(ROC_ROBOT_GAIT_FIVE)
#define ROC_ROBOT_RUN_GAIT_NUM                      5
#else
#define ROC_ROBOT_RUN_GAIT_NUM                      4
#endif

void RocRobotInit(void);
void RocRobotMain(void);
void RocRobotRemoteControl(void);


#endif


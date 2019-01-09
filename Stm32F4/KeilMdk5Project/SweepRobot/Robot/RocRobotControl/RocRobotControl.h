#ifndef __ROC_ROBOTCONTROL_H
#define __ROC_ROBOTCONTROL_H


//#define ROC_ROBOT_GAIT_DEBUG
//#define ROC_ROBOT_GAIT_FIVE
//#define ROC_ROBOT_CLOSED_LOOP_CONTROL

#define ROC_ROBOT_MATH_CONST_PI                     3.1415926


#define ROC_ROBOT_DH_CONST_D1                       0
#define ROC_ROBOT_DH_CONST_A1                       44
#define ROC_ROBOT_DH_CONST_A2                       74
#define ROC_ROBOT_DH_CONST_A3                       112.38  //83.24


#define ROC_ROBOT_INIT_ANGLE_THET_1                 60
#define ROC_ROBOT_INIT_ANGLE_THET_2                 0
#define ROC_ROBOT_INIT_ANGLE_THET_3                 60
#define ROC_ROBOT_INIT_ANGLE_BETA_1                 150
#define ROC_ROBOT_INIT_ANGLE_BETA_2                 90
#define ROC_ROBOT_INIT_ANGLE_BETA_3                 30


#define ROC_ROBOT_FIRST_STEP_ERROR                  0
#define ROC_ROBOT_SECND_STEP_ERROR                  4
#define ROC_ROBOT_LEFT_FIRST_STEP_ERROR             -2
#define ROC_ROBOT_LEFT_SECND_STEP_ERROR             0
#define ROC_ROBOT_RIGHT_FIRST_STEP_ERROR            -2
#define ROC_ROBOT_RIGHT_SECND_STEP_ERROR            0
#define ROC_ROBOT_PID_CONST_P                       1


#define ROC_ROBOT_LEG_WIDTH                         141.06
#define ROC_ROBOT_LEG_HEIGHT                        110 //80
#define ROC_ROBOT_FEET_WIDTH                        23


#define ROC_ROBOT_INIT_DOWN_ANGLE                   0
#define ROC_ROBOT_ANGLE_TO_RADIAN                   (ROC_ROBOT_MATH_CONST_PI / ROC_SERVO_MAX_ROTATE_ANGLE)
#define ROC_ROBOT_ROTATE_ANGLE_TO_PWM               ((ROC_SERVO_MAX_PWM_VAL - ROC_SERVO_MIN_PWM_VAL) / ROC_SERVO_MAX_ROTATE_ANGLE)


#define ROC_ROBOT_WIDTH                             (ROC_ROBOT_DH_CONST_A1 + ROC_ROBOT_DH_CONST_A2 * cos(ROC_ROBOT_INIT_DOWN_ANGLE \
                                                    * ROC_ROBOT_ANGLE_TO_RADIAN) + ROC_ROBOT_FEET_WIDTH)
#define ROC_ROBOT_HEIGHT                            (ROC_ROBOT_DH_CONST_A2 * sin(ROC_ROBOT_INIT_DOWN_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN) \
                                                    + ROC_ROBOT_LEG_HEIGHT)


#define ROC_ROBOT_FRO_HIP_INIT_ANGLE                60
#define ROC_ROBOT_FRO_LEG_INIT_ANGLE                0
#define ROC_ROBOT_FRO_FET_INIT_ANGLE                78.19//73.96
#define ROC_ROBOT_MID_HIP_INIT_ANGLE                0
#define ROC_ROBOT_MID_LEG_INIT_ANGLE                0
#define ROC_ROBOT_MID_FET_INIT_ANGLE                78.19//73.96
#define ROC_ROBOT_HIN_HIP_INIT_ANGLE                60
#define ROC_ROBOT_HIN_LEG_INIT_ANGLE                0
#define ROC_ROBOT_HIN_FET_INIT_ANGLE                78.19//73.96


#define ROC_ROBOT_FRO_INIT_X                        (ROC_ROBOT_WIDTH * cos(ROC_ROBOT_FRO_HIP_INIT_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN))
#define ROC_ROBOT_FRO_INIT_Y                        (ROC_ROBOT_WIDTH * sin(ROC_ROBOT_FRO_HIP_INIT_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN))
#define ROC_ROBOT_FRO_INIT_Z                        -ROC_ROBOT_HEIGHT
#define ROC_ROBOT_MID_INIT_X                        (ROC_ROBOT_WIDTH * cos(ROC_ROBOT_MID_HIP_INIT_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN))
#define ROC_ROBOT_MID_INIT_Y                        (ROC_ROBOT_WIDTH * sin(ROC_ROBOT_MID_HIP_INIT_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN))
#define ROC_ROBOT_MID_INIT_Z                        -ROC_ROBOT_HEIGHT
#define ROC_ROBOT_HIN_INIT_X                        (ROC_ROBOT_WIDTH * cos(ROC_ROBOT_HIN_HIP_INIT_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN))
#define ROC_ROBOT_HIN_INIT_Y                        (ROC_ROBOT_WIDTH * sin(ROC_ROBOT_HIN_HIP_INIT_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN))
#define ROC_ROBOT_HIN_INIT_Z                        -ROC_ROBOT_HEIGHT


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
#define ROC_ROBOT_LEF_FRO_LEG_CENTER                402
#define ROC_ROBOT_LEF_FRO_FET_CENTER                310
#define ROC_ROBOT_LEF_MID_HIP_CENTER                310
#define ROC_ROBOT_LEF_MID_LEG_CENTER                416
#define ROC_ROBOT_LEF_MID_FET_CENTER                304
#define ROC_ROBOT_LEF_HIN_HIP_CENTER                303
#define ROC_ROBOT_LEF_HIN_LEG_CENTER                400
#define ROC_ROBOT_LEF_HIN_FET_CENTER                310


#define ROC_ROBOT_DEFAULT_LEG_STEP                  25
#define ROC_ROBOT_DEFAULT_LEG_ANGLE                 15
#define ROC_ROBOT_DEFAULT_FEET_LIFT                 30


#define ROC_ROBOT_CTRL_CMD_MOSTAND                  'C'
#define ROC_ROBOT_CTRL_CMD_FORWARD                  'W'
#define ROC_ROBOT_CTRL_CMD_BAKWARD                  'S'
#define ROC_ROBOT_CTRL_CMD_LFCLOCK                  'A'
#define ROC_ROBOT_CTRL_CMD_RGCLOCK                  'D'
#define ROC_ROBOT_CTRL_CMD_PARAMET                  'Y'
#define ROC_ROBOT_CTRL_CMD_CARFORD                  'B'
#define ROC_ROBOT_CTRL_CMD_CARBAKD                  'R'
#define ROC_ROBOT_CTRL_CMD_ROTMODE                  'Y'
#define ROC_ROBOT_CTRL_CMD_CARMODE                  'G'

#if defined(ROC_ROBOT_GAIT_SIX)
#define ROC_ROBOT_RUN_GAIT_NUM                      6
#elif defined(ROC_ROBOT_GAIT_FIVE)
#define ROC_ROBOT_RUN_GAIT_NUM                      5
#else
#define ROC_ROBOT_RUN_GAIT_NUM                      4
#endif


typedef enum {
    ROC_ROBOT_WALK_MODE_CAR = 0,
    ROC_ROBOT_WALK_MODE_HEXAPOD = 1,
}ROC_ROBOT_WALK_MODE_e;



void RocRobotInit(void);
void RocRobotMain(void);
void RocRobotRemoteControl(void);


#endif


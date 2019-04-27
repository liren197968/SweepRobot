/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2018/12/16      1.0
********************************************************************************/
#ifndef __ROC_ROBOT_DH_H
#define __ROC_ROBOT_DH_H


#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "arm_math.h"

#include "RocServo.h"
#include "RocRobotMath.h"


//#define ROC_ROBOT_GAIT_DEBUG
#define ROC_ROBOT_GAIT_QUAD_MODE_ENABLE
#define ROC_ROBOT_CLOSED_LOOP_CONTROL


#define ROC_ROBOT_MATH_CONST_PI                     3.1415926


#define ROC_ROBOT_DH_CONST_D1                       0
#define ROC_ROBOT_DH_CONST_A1                       44
#define ROC_ROBOT_DH_CONST_A2                       74
#define ROC_ROBOT_DH_CONST_A3                       112.38F


#define ROC_ROBOT_FRO_HIP_INIT_ANGLE                60
#define ROC_ROBOT_FRO_LEG_INIT_ANGLE                0
#define ROC_ROBOT_FRO_FET_INIT_ANGLE                78.19F
#define ROC_ROBOT_MID_HIP_INIT_ANGLE                0
#define ROC_ROBOT_MID_LEG_INIT_ANGLE                0
#define ROC_ROBOT_MID_FET_INIT_ANGLE                78.19F
#define ROC_ROBOT_HIN_HIP_INIT_ANGLE                60
#define ROC_ROBOT_HIN_LEG_INIT_ANGLE                0
#define ROC_ROBOT_HIN_FET_INIT_ANGLE                78.19F


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

#define ROC_ROBOT_PID_CONST_P                       2

#define ROC_ROBOT_STEP_ERROR_LOW_LIMIT              -15
#define ROC_ROBOT_STEP_ERROR_HIGH_LIMIT             15


#define ROC_ROBOT_LEG_WIDTH                         141.06F
#define ROC_ROBOT_LEG_HEIGHT                        110
#define ROC_ROBOT_FEET_WIDTH                        23


#define ROC_ROBOT_INIT_DOWN_ANGLE                   0
#define ROC_ROBOT_ANGLE_TO_RADIAN                   (ROC_ROBOT_MATH_CONST_PI / 180)
#define ROC_ROBOT_ROTATE_ANGLE_TO_PWM               ((ROC_SERVO_MAX_PWM_VAL - ROC_SERVO_MIN_PWM_VAL) / ROC_SERVO_MAX_ROTATE_ANGLE)


#define ROC_ROBOT_WIDTH                             (ROC_ROBOT_DH_CONST_A1 + ROC_ROBOT_DH_CONST_A2 * Cos(ROC_ROBOT_INIT_DOWN_ANGLE \
                                                    * ROC_ROBOT_ANGLE_TO_RADIAN) + ROC_ROBOT_FEET_WIDTH)
#define ROC_ROBOT_HEIGHT                            (ROC_ROBOT_DH_CONST_A2 * Sin(ROC_ROBOT_INIT_DOWN_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN) \
                                                    + ROC_ROBOT_LEG_HEIGHT)


#define ROC_ROBOT_FRO_INIT_X                        (ROC_ROBOT_WIDTH * Cos(ROC_ROBOT_FRO_HIP_INIT_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN))
#define ROC_ROBOT_FRO_INIT_Y                        (ROC_ROBOT_WIDTH * Sin(ROC_ROBOT_FRO_HIP_INIT_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN))
#define ROC_ROBOT_FRO_INIT_Z                        -ROC_ROBOT_HEIGHT
#define ROC_ROBOT_MID_INIT_X                        (ROC_ROBOT_WIDTH * Cos(ROC_ROBOT_MID_HIP_INIT_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN))
#define ROC_ROBOT_MID_INIT_Y                        (ROC_ROBOT_WIDTH * Sin(ROC_ROBOT_MID_HIP_INIT_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN))
#define ROC_ROBOT_MID_INIT_Z                        -ROC_ROBOT_HEIGHT
#define ROC_ROBOT_HIN_INIT_X                        (ROC_ROBOT_WIDTH * Cos(ROC_ROBOT_HIN_HIP_INIT_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN))
#define ROC_ROBOT_HIN_INIT_Y                        (ROC_ROBOT_WIDTH * Sin(ROC_ROBOT_HIN_HIP_INIT_ANGLE * ROC_ROBOT_ANGLE_TO_RADIAN))
#define ROC_ROBOT_HIN_INIT_Z                        -ROC_ROBOT_HEIGHT


#define ROC_ROBOT_RIG_FRO_HIP_CENTER                302
#define ROC_ROBOT_RIG_FRO_LEG_CENTER                172
#define ROC_ROBOT_RIG_FRO_FET_CENTER                332
#define ROC_ROBOT_RIG_MID_HIP_CENTER                301
#define ROC_ROBOT_RIG_MID_LEG_CENTER                180
#define ROC_ROBOT_RIG_MID_FET_CENTER                300
#define ROC_ROBOT_RIG_HIN_HIP_CENTER                307
#define ROC_ROBOT_RIG_HIN_LEG_CENTER                180
#define ROC_ROBOT_RIG_HIN_FET_CENTER                310

#define ROC_ROBOT_LEF_FRO_HIP_CENTER                306
#define ROC_ROBOT_LEF_FRO_LEG_CENTER                402
#define ROC_ROBOT_LEF_FRO_FET_CENTER                310
#define ROC_ROBOT_LEF_MID_HIP_CENTER                310
#define ROC_ROBOT_LEF_MID_LEG_CENTER                410
#define ROC_ROBOT_LEF_MID_FET_CENTER                304
#define ROC_ROBOT_LEF_HIN_HIP_CENTER                303
#define ROC_ROBOT_LEF_HIN_LEG_CENTER                400
#define ROC_ROBOT_LEF_HIN_FET_CENTER                310


#define ROC_ROBOT_DEFAULT_LEG_STEP                  40
#define ROC_ROBOT_DEFAULT_TURN_ANGLE                20
#define ROC_ROBOT_DEFAULT_FEET_LIFT                 24


//#define ROC_ROBOT_GAIT_DEBUG
#define ROC_ROBOT_DISPLAY_GAIT_NAMES
//#define ROC_ROBOT_GAIT_QUADMODE                       // We are building for quad support


#define ROC_ROBOT_TRAVEL_DEAD_ZONE                  1   //The deadzone for the analog input from the remote

#define ROC_ROBOT_RUN_SPEED_POWER_ON                (ROC_SERVO_PWM_CYCLE * ROC_SERVO_SPEED_DIV_STP * ROC_SERVO_SPEED_DOWN_MAG * 4)
#define ROC_ROBOT_RUN_SPEED_DEFAULT                 (ROC_SERVO_PWM_CYCLE * ROC_SERVO_SPEED_DIV_STP * ROC_SERVO_SPEED_DOWN_MAG)


typedef struct _ROC_ROBOT_COORD_s
{
    float               X;
    float               Y;
    float               Z;
    float               A;

}ROC_ROBOT_COORD_s;


typedef struct _ROC_ROBOT_IMU_DATA_s
{
    float               Pitch;
    float               Roll;
    float               Yaw;

}ROC_ROBOT_IMU_DATA_s;


typedef enum _ROC_ROBOT_LEG_JOINT_e
{
    ROC_ROBOT_LEG_HIP_JOINT = 0,
    ROC_ROBOT_LEG_KNEE_JOINT,
    ROC_ROBOT_LEG_ANKLE_JOINT,

    ROC_ROBOT_LEG_JOINT_NUM,
}ROC_ROBOT_LEG_JOINT_e;


typedef enum _ROC_ROBOT_LEG_e
{
    ROC_ROBOT_RIG_FRO_LEG = 0,
    ROC_ROBOT_RIG_MID_LEG,
    ROC_ROBOT_RIG_HIN_LEG,
    ROC_ROBOT_LEF_FRO_LEG,
    ROC_ROBOT_LEF_MID_LEG,
    ROC_ROBOT_LEF_HIN_LEG,

    ROC_ROBOT_CNT_LEGS,
}ROC_ROBOT_LEG_e;


typedef enum _ROC_ROBOT_GAIT_TYPE_e
{
    ROC_ROBOT_GAIT_HEXP_MODE_RIPPLE_12 = 0,
    ROC_ROBOT_GAIT_HEXP_MODE_TRIPOD_8,
    ROC_ROBOT_GAIT_HEXP_MODE_TRIPLE_12,
    ROC_ROBOT_GAIT_HEXP_MODE_TRIPLE_16,
    ROC_ROBOT_GAIT_HEXP_MODE_WAVE_24,
    ROC_ROBOT_GAIT_HEXP_MODE_TRIPOD_6,
    ROC_ROBOT_GAIT_HEXP_MODE_CIRCLE_6,

#ifdef ROC_ROBOT_GAIT_QUAD_MODE_ENABLE
    ROC_ROBOT_GAIT_QUAD_MODE_RIPPLE_4,
    ROC_ROBOT_GAIT_QUAD_MODE_SM_RIPPLE_4,
    ROC_ROBOT_GAIT_QUAD_MODE_AMBLE_4,
    ROC_ROBOT_GAIT_QUAD_MODE_SM_AMBLE_4,
#endif

    ROC_ROBOT_GAIT_TYPE_NUM,
}ROC_ROBOT_GAIT_TYPE_e;

typedef enum _ROC_ROBOT_MOVE_STATUS_e
{
    ROC_ROBOT_MOVE_STATUS_POWER_ON = 0,
    ROC_ROBOT_MOVE_STATUS_STANDING,
    ROC_ROBOT_MOVE_STATUS_FORWALKING,       /* Robot is forward walking */
    ROC_ROBOT_MOVE_STATUS_BAKWALKING,       /* Robot is backward walking */
    ROC_ROBOT_MOVE_STATUS_CIRCLING,

    ROC_ROBOT_MOVE_STATUS_NUM,
}ROC_ROBOT_MOVE_STATUS_e;


typedef struct _ROC_PHOENIX_GAIT_s
{
    uint16_t                    NomGaitSpeed;           // Nominal speed of the gait
    uint8_t                     StepsInGait;            // Number of steps in gait
    uint8_t                     NrLiftedPos;            // Number of positions that a single leg is lifted [1-3]
    uint8_t                     FrontDownPos;           // Where the leg should be put down to ground
    uint8_t                     LiftDivFactor;          // Normaly: 2, when NrLiftedPos=5: 4
    uint8_t                     SlidDivFactor;          // Number of steps that a leg is on the floor while walking
    uint8_t                     HalfLiftHeight;         // How high to lift at halfway up.

#ifdef ROC_ROBOT_GAIT_QUAD_MODE_ENABLE
    // Extra information used in the Quad balance mode
    uint32_t                    CogAngleStart;          // COG shifting starting angle
    uint32_t                    CogAngleStep;           // COG Angle Steps in degrees
    uint8_t                     CogRadius;              // COG Radius; the amount the body shifts
    uint8_t                     CogCcw;                 // COG Gait sequence runs counter clock wise
#endif

    uint8_t                     GaitLegNr[ROC_ROBOT_CNT_LEGS];// Init position of the leg

#ifdef ROC_ROBOT_DISPLAY_GAIT_NAMES
    const char*                 PszName;                // The gait name
#endif

}ROC_PHOENIX_GAIT_s;


typedef struct _ROC_PHOENIX_STATE_s
{
    uint8_t                     CtrlTimeIsReady;        // check if the control time is ready
    uint8_t                     BatTimeIsReady;         // check if the battery time is ready

    ROC_ROBOT_MOVE_STATUS_e     MoveStatus;             // True if the robot are walking
    uint8_t                     RobotOn;                // Switch to turn on Phoenix
    uint8_t                     PrevRobotOn;            // Previous loop state

    //Body position
    ROC_ROBOT_COORD_s           BodyCurPos;
    ROC_ROBOT_COORD_s           LegCurPos[ROC_ROBOT_CNT_LEGS];
    ROC_ROBOT_COORD_s           BodyRotOffset;          // Body rotation offset;

    //Body Inverse Kinematics
    ROC_ROBOT_COORD_s           BodyRot;                // X-Pitch, Y-Rotation, Z-Roll

    //[gait]
    ROC_ROBOT_GAIT_TYPE_e       GaitType;               // Gait type
    uint8_t                     GaitStep;               // Actual current step in gait
    uint8_t                     TravelRequest;          //Temp to check if the gait is in motion
    double                      GaitRot[ROC_ROBOT_CNT_LEGS];//Array containing Relative Z rotation corresponding to the Gait
    uint16_t                    LegLiftHeight;          // Current Travel height
    ROC_ROBOT_COORD_s           TravelLength;           // X-Y or Length, Z is rotation
    uint16_t                    TurnLength;             // turn angle for clockwise move

#ifdef TurretRotPin
    // Turret information
    int32_t                     TurretRotAngle;         // Rotation of turrent in 10ths of degree
    int32_t                     TurretTiltAngle;        // The tile for the turret
#endif

    //[Single Leg Control]
    ROC_ROBOT_LEG_e             SelectLegNum;
    ROC_ROBOT_COORD_s           SelectLegCor;
    uint8_t                     SelectLegMode;          // Single leg control mode
    uint8_t                     SelectLegIsAllDown;     // True if the robot legs are all down

#ifdef ROC_ROBOT_CLOSED_LOOP_CONTROL
    //[Balance]
    uint8_t                     BalanceMode;
    ROC_ROBOT_IMU_DATA_s        RefImuAngle;            // IMU reference control angle for robot walking
    ROC_ROBOT_IMU_DATA_s        CurImuAngle;            // Robot current IMU angle when walking
#endif

    //[TIMING]
    uint8_t                     InputTimeDelay;         // Delay that depends on the input to get the "sneaking" effect
    uint32_t                    SpeedControl;           // Adjustible Delay
    uint8_t                     ForceGaitStepCnt;       // New to allow us to force a step even when not moving

#ifdef ROC_ADJUSTABLE_LEG_ANGLES
    uint16_t                    CoxaInitAngle[ROC_ROBOT_CNT_LEGS];
#endif

}ROC_PHOENIX_STATE_s;


typedef struct _ROC_ROBOT_LEG_s
{
    int16_t                     RobotJoint[ROC_ROBOT_LEG_JOINT_NUM];

}ROC_ROBOT_LEG_s;


typedef struct _ROC_ROBOT_SERVO_s
{
    ROC_ROBOT_LEG_s             RobotLeg[ROC_ROBOT_CNT_LEGS];

}ROC_ROBOT_SERVO_s;


typedef struct _ROC_ROBOT_CONTROL_s
{
    ROC_PHOENIX_GAIT_s  CurGait;                        // Definition of the current gait
    ROC_PHOENIX_STATE_s CurState;                       // Definition of the current state
    ROC_ROBOT_SERVO_s   CurServo;                       // Definition of the current servo

}ROC_ROBOT_CONTROL_s;



void RocRobotGaitSeqUpdate(void);
ROC_RESULT RocRobotAlgoCtrlInit(void);
ROC_ROBOT_CONTROL_s *RocRobotCtrlInfoGet(void);
ROC_ROBOT_MOVE_STATUS_e RocRobotMoveStatus_Get(void);
void RocRobotMoveStatus_Set(ROC_ROBOT_MOVE_STATUS_e MoveStatus);
void RocRobotSingleLegSelect(ROC_ROBOT_LEG_e SlecetLegNum);
void RocRobotSingleLegCtrl(ROC_ROBOT_SERVO_s *pRobotServo);
void RocRobotOpenLoopWalkCalculate(ROC_ROBOT_SERVO_s *pRobotServo);
void RocRobotOpenLoopCircleCalculate(ROC_ROBOT_SERVO_s *pRobotServo);
void RocRobotClosedLoopWalkCalculate(ROC_ROBOT_SERVO_s *pRobotServo);
void RocRobotCtrlDeltaMoveCoorInput(double x, double y, double z, double a, double h);


#endif


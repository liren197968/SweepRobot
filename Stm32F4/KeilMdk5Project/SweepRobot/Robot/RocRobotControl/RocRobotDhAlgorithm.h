/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2018/12/16      1.0
********************************************************************************/
#ifndef __ROC_ROBOT_DH_H
#define __ROC_ROBOT_DH_H


#include <stdint.h>


#define ROC_ROBOT_GAIT_DEBUG
#define ROC_ROBOT_DISPLAY_GAIT_NAMES
//#define ROC_ROBOT_GAIT_QUADMODE   // We are building for quad support

#define ROC_ROBOT_TRAVEL_DEAD_ZONE          1   //The deadzone for the analog input from the remote
#define ROC_ROBOT_RUN_SPEED_DEFAULT         90


typedef struct _ROC_ROBOT_COORD_s
{
    double              X;
    double              Y;
    double              Z;
    double              A;

}ROC_ROBOT_COORD_s;


typedef enum _ROC_ROBOT_LEG_JOINT_e
{
    ROC_ROBOT_LEG_HIP_JOINT = 0,
    ROC_ROBOT_LEG_KNEE_JOINT,
    ROC_ROBOT_LEG_ANKLE_JOINT,

    ROC_ROBOT_LEG_JOINT_NUM,
}ROC_ROBOT_LEG_JOINT_e;


#ifdef ROC_ROBOT_GAIT_QUADMODE
typedef enum _ROC_ROBOT_LEG_e
{
    ROC_ROBOT_RIG_FRO_LEG = 0,
    ROC_ROBOT_RIG_HIN_LEG,
    ROC_ROBOT_LEF_FRO_LEG,
    ROC_ROBOT_LEF_HIN_LEG,

    ROC_ROBOT_CNT_LEGS,
}ROC_ROBOT_LEG_e;
#else
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
#endif


typedef enum _ROC_GAIT_TYPE_e
{
    ROC_ROBOT_GAIT_RIPPLE_12 = 0,
    ROC_ROBOT_GAIT_TRIPOD_8,
    ROC_ROBOT_GAIT_TRIPLE_12,
    ROC_ROBOT_GAIT_TRIPLE_16,
    ROC_ROBOT_GAIT_WAVE_24,
    ROC_ROBOT_GAIT_TRIPOD_6,
    ROC_ROBOT_GAIT_CIRCLE_6,

    ROC_ROBOT_GAIT_TYPE_NUM,
}ROC_GAIT_TYPE_e;


//==============================================================================
// Define Gait structure/class - Hopefully allow specific robots to define their
// own gaits and/or define which of the standard ones they want.
//==============================================================================
typedef struct _ROC_PHOENIX_GAIT_s
{
    uint16_t            NomGaitSpeed;           // Nominal speed of the gait
    uint8_t             StepsInGait;            // Number of steps in gait
    uint8_t             NrLiftedPos;            // Number of positions that a single leg is lifted [1-3]
    uint8_t             FrontDownPos;           // Where the leg should be put down to ground
    uint8_t             LiftDivFactor;          // Normaly: 2, when NrLiftedPos=5: 4
    uint8_t             SlidDivFactor;          // Number of steps that a leg is on the floor while walking
    uint8_t             HalfLiftHeight;         // How high to lift at halfway up.

#ifdef ROC_ROBOT_GAIT_QUADMODE
    // Extra information used in the Quad balance mode
    uint32_t            CogAngleStart;          // COG shifting starting angle
    uint32_t            CogAngleStep;           // COG Angle Steps in degrees
    uint8_t             CogRadius;              // COG Radius; the amount the body shifts
    uint8_t             CogCcw;                 // COG Gait sequence runs counter clock wise
#endif

    uint8_t             GaitLegNr[ROC_ROBOT_CNT_LEGS];// Init position of the leg

#ifdef ROC_ROBOT_DISPLAY_GAIT_NAMES
    const char*         PszName;                // The gait name
#endif

}ROC_PHOENIX_GAIT_s;


typedef struct _ROC_PHOENIX_STATE_s
{
    uint8_t             IsWalking;              // True if the robot are walking
    uint8_t             RobotOn;                // Switch to turn on Phoenix
    uint8_t             PrevRobotOn;            // Previous loop state

    //Body position
    ROC_ROBOT_COORD_s   BodyCurPos;
    ROC_ROBOT_COORD_s   LegCurPos[ROC_ROBOT_CNT_LEGS];
    ROC_ROBOT_COORD_s   BodyRotOffset;          // Body rotation offset;

    //Body Inverse Kinematics
    ROC_ROBOT_COORD_s   BodyRot;                // X-Pitch, Y-Rotation, Z-Roll

    //[gait]
    ROC_GAIT_TYPE_e     GaitType;               // Gait type
    uint8_t             GaitStep;               // Actual current step in gait
    uint8_t             TravelRequest;          //Temp to check if the gait is in motion
    double              GaitRot[ROC_ROBOT_CNT_LEGS];//Array containing Relative Z rotation corresponding to the Gait
    uint16_t            LegLiftHeight;          // Current Travel height
    ROC_ROBOT_COORD_s   TravelLength;           // X-Y or Length, Z is rotation
    uint16_t            TurnLength;             // turn angle for clockwise move

#ifdef TurretRotPin
    // Turret information
    int32_t             TurretRotAngle;         // Rotation of turrent in 10ths of degree
    int32_t             TurretTiltAngle;        // The tile for the turret
#endif

    //[Single Leg Control]
    uint8_t             SelectLegNum;
    ROC_ROBOT_COORD_s     SelectLegCor;
    uint8_t             SelectLegMode;          // Single leg control mode

    //[Balance]
    uint8_t             BalanceMode;

    //[TIMING]
    uint8_t             InputTimeDelay;         // Delay that depends on the input to get the "sneaking" effect
    uint32_t            SpeedControl;           // Adjustible Delay
    uint8_t             ForceGaitStepCnt;       // New to allow us to force a step even when not moving

#ifdef ROC_ADJUSTABLE_LEG_ANGLES
    uint16_t            CoxaInitAngle[ROC_ROBOT_CNT_LEGS];
#endif

}ROC_PHOENIX_STATE_s;


typedef struct _ROC_ROBOT_LEG_s
{
    int16_t             RobotJoint[ROC_ROBOT_LEG_JOINT_NUM];

}ROC_ROBOT_LEG_s;


typedef struct _ROC_ROBOT_SERVO_s
{
    ROC_ROBOT_LEG_s     RobotLeg[ROC_ROBOT_CNT_LEGS];

}ROC_ROBOT_SERVO_s;


//==============================================================================
// class ControlState: This is the main structure of data that the Control 
//      manipulates and is used by the main Phoenix Code to make it do what is
//      requested.
//==============================================================================
typedef struct _ROC_ROBOT_CONTROL_s
{
    ROC_PHOENIX_GAIT_s  CurGait;                // Definition of the current gait
    ROC_PHOENIX_STATE_s CurState;               // Definition of the current state
    ROC_ROBOT_SERVO_s   CurServo;               // Definition of the current servo

}ROC_ROBOT_CONTROL_s;


void RocRobotGaitSeqUpdate(void);
ROC_RESULT RocRobotAlgoCtrlInit(void);
ROC_ROBOT_CONTROL_s *RocRobotCtrlInfoGet(void);
void RocRobotOpenLoopWalkCalculate(ROC_ROBOT_SERVO_s *pRobotServo);
void RocRobotOpenLoopCircleCalculate(ROC_ROBOT_SERVO_s *pRobotServo);
void RocRobotCtrlDeltaMoveCoorInput(double x, double y, double z, double a, double h);


#endif


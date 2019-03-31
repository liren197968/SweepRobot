/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2018/12/16      1.0
********************************************************************************/
#ifndef __ROC_ROBOT_DH_H
#define __ROC_ROBOT_DH_H


#include <stdint.h>


#define ROC_ROBOT_DISPLAY_GAIT_NAMES
//#define ROC_ROBOT_GAIT_QUADMODE   // We are building for quad support
#define cTravelDeadZone 4      //The deadzone for the analog input from the remote


#define ROC_ROBOT_RUN_SPEED_DEFAULT     60
#define ROC_ROBOT_PER_LEG_JOINT_NUM     3

typedef struct _ROC_COORD_XYZ_s
{
    double              X;
    double              Y;
    double              Z;

}ROC_COORD_XYZ_s;


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
    ROC_COORD_XYZ_s     BodyCurPos;
    ROC_COORD_XYZ_s     LegCurPos[ROC_ROBOT_CNT_LEGS];
    ROC_COORD_XYZ_s     BodyRotOffset;          // Body rotation offset;

    //Body Inverse Kinematics
    ROC_COORD_XYZ_s     BodyRot;                // X-Pitch, Y-Rotation, Z-Roll

    //[gait]
    uint8_t             GaitType;               // Gait type
    uint8_t             GaitStep;               // Actual current step in gait
    uint8_t             TravelRequest;          //Temp to check if the gait is in motion
    double              GaitRot[ROC_ROBOT_CNT_LEGS];//Array containing Relative Z rotation corresponding to the Gait
    uint16_t            LegLiftHeight;          // Current Travel height
    ROC_COORD_XYZ_s     TravelLength;           // X-Y or Length, Z is rotation

#ifdef TurretRotPin
    // Turret information
    int32_t             TurretRotAngle;         // Rotation of turrent in 10ths of degree
    int32_t             TurretTiltAngle;        // The tile for the turret
#endif

    //[Single Leg Control]
    uint8_t             SelectLegNum;
    ROC_COORD_XYZ_s     SelectLegCor;
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

//==============================================================================
// class ControlState: This is the main structure of data that the Control 
//      manipulates and is used by the main Phoenix Code to make it do what is
//      requested.
//==============================================================================
typedef struct _ROC_ROBOT_CONTROL_s
{
    ROC_PHOENIX_GAIT_s  CurGait;                // Definition of the current gait
    ROC_PHOENIX_STATE_s CurState;               // Definition of the current state

}ROC_ROBOT_CONTROL_s;


void RocRobotGaitSeqUpdate(void);
ROC_RESULT RocRobotAlgoCtrlInit(void);
ROC_ROBOT_CONTROL_s *RocRobotCtrlInfoGet(void);
void RocRobotCtrlDeltaMoveCoorInput(double x, double y, double z, double h);
void RocRobotOpenLoopWalkCalculate(uint16_t *pRobotCtrlPwmVal);
void RocRobotOpenLoopCircleCalculate(float DeltaAngle, float Lift, uint16_t *pRobotCtrlPwmVal);


#endif


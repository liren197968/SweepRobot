#ifndef __ROC_MOTOR_H
#define __ROC_MOTOR_H


#define ROC_MOTOR_MAX_SPEED                 4095
#define ROC_MOTOR_MIN_SPEED                 10
#define ROC_MOTOR_DEFAULT_SPEED             1300
#define ROC_MOTOR_DIFFER_SPEED              300
#define ROC_MOTOR_SERVO_DEFAULT_ANGLE       270


#define ROC_MOTOR_LEFT_ERROR_SPEED          100
#define ROC_MOTOR_RIGT_ERROR_SPEED          100


#define ROC_MOTOR_ENA_PWM_CHANNEL           11
#define ROC_MOTOR_ENB_PWM_CHANNEL           8
#define ROC_MOTOR_SERVO_PWM_CHANNEL         4


#define ROC_MOTOR_STOPPED_ROTATE            0
#define ROC_MOTOR_FORWARD_ROTATE            1
#define ROC_MOTOR_REVERSE_ROTATE            2


ROC_RESULT RocMotorInit(void);
void RocMotorRotateDirectionSet(uint8_t Direction);
ROC_RESULT RocMotorServoTurnAngleSet(uint16_t TurnAngle);
ROC_RESULT RocMotorSpeedSet(uint16_t LeftMotorSpeed, uint16_t RightMotorSpeed);


#endif


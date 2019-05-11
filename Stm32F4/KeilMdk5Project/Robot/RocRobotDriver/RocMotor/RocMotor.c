/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/01/06      1.0
********************************************************************************/
#include "stm32f4xx_hal.h"
#include "gpio.h"

#include "RocLog.h"
#include "RocMotor.h"
#include "RocServo.h"
#include "RocPca9685.h"


/*********************************************************************************
 *  Description:
 *              Set the motor current speed
 *
 *  Parameter:
 *              LeftMotorSpeed: the left motor PWM speed
 *              RightMotorSpeed: the right motor PWM speed
 *
 *  Return:
 *              The PWM output status
 *
 *  Author:
 *              ROC LiRen(2019.01.06)
**********************************************************************************/
ROC_RESULT RocMotorSpeedSet(uint16_t LeftMotorSpeed, uint16_t RightMotorSpeed)
{
    HAL_StatusTypeDef   WriteStatus = HAL_OK;

    if(ROC_MOTOR_MAX_SPEED < LeftMotorSpeed)
    {
        LeftMotorSpeed = ROC_MOTOR_MAX_SPEED;

        ROC_LOGN("The input left motor PWM speed is beyond the max value, be careful!");
    }

    if(ROC_MOTOR_MAX_SPEED < RightMotorSpeed)
    {
        RightMotorSpeed = ROC_MOTOR_MAX_SPEED;

        ROC_LOGN("The input right motor PWM speed is beyond the max value, be careful!");
    }

    WriteStatus = RocPca9685OutPwm(PWM_ADDRESS_H, ROC_MOTOR_ENB_PWM_CHANNEL, 0U, LeftMotorSpeed);
    if(HAL_OK != WriteStatus)
    {
        ROC_LOGE("Left motor output PWM is in error, and motor will stop running!");
    }

    WriteStatus = RocPca9685OutPwm(PWM_ADDRESS_H, ROC_MOTOR_ENA_PWM_CHANNEL, 0U, RightMotorSpeed);
    if(HAL_OK != WriteStatus)
    {
        ROC_LOGE("Right motor output PWM is in error, and motor will stop running!");
    }

    return (ROC_RESULT)WriteStatus;
}

/*********************************************************************************
 *  Description:
 *              Drive car turn direction
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.01.06)
**********************************************************************************/
ROC_RESULT RocMotorServoTurnAngleSet(uint16_t TurnAngle)
{
    HAL_StatusTypeDef   WriteStatus = HAL_OK;

    if(ROC_SERVO_MAX_PWM_VAL < TurnAngle)
    {
        TurnAngle = ROC_SERVO_MAX_PWM_VAL;

        ROC_LOGN("The input motor servo PWM speed is beyond the max value, be careful!");
    }

    WriteStatus = RocPca9685OutPwm(PWM_ADDRESS_H, ROC_MOTOR_SERVO_PWM_CHANNEL, 0U, TurnAngle);
    if(HAL_OK != WriteStatus)
    {
        ROC_LOGE("Motor servo output PWM is in error, and motor will stop running!");
    }

    return (ROC_RESULT)WriteStatus;
}

/*********************************************************************************
 *  Description:
 *              Drive motor stopped
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.01.06)
**********************************************************************************/
static void RocMotorRotateStopped(void)
{
    RocMotorSpeedSet(ROC_MOTOR_MIN_SPEED, ROC_MOTOR_MIN_SPEED);

    HAL_GPIO_WritePin(ROC_MOTOR_GPIO_PORT1, ROC_MOTOR_IN1_PIN | ROC_MOTOR_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ROC_MOTOR_GPIO_PORT2, ROC_MOTOR_IN3_PIN | ROC_MOTOR_IN4_PIN, GPIO_PIN_RESET);
}

/*********************************************************************************
 *  Description:
 *              Drive motor forward
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.01.06)
**********************************************************************************/
static void RocMotorRotateForward(void)
{
    RocMotorSpeedSet(ROC_MOTOR_DEFAULT_SPEED, ROC_MOTOR_DEFAULT_SPEED);

    HAL_GPIO_WritePin(ROC_MOTOR_GPIO_PORT1, ROC_MOTOR_IN2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ROC_MOTOR_GPIO_PORT2, ROC_MOTOR_IN4_PIN, GPIO_PIN_SET);

    HAL_GPIO_WritePin(ROC_MOTOR_GPIO_PORT1, ROC_MOTOR_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ROC_MOTOR_GPIO_PORT2, ROC_MOTOR_IN3_PIN, GPIO_PIN_RESET);
}

/*********************************************************************************
 *  Description:
 *              Drive motor reverse
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.01.06)
**********************************************************************************/
static void RocMotorRotateReverse(void)
{
    RocMotorSpeedSet(ROC_MOTOR_DEFAULT_SPEED + ROC_MOTOR_DIFFER_SPEED, ROC_MOTOR_DEFAULT_SPEED + ROC_MOTOR_DIFFER_SPEED);

    HAL_GPIO_WritePin(ROC_MOTOR_GPIO_PORT1, ROC_MOTOR_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ROC_MOTOR_GPIO_PORT2, ROC_MOTOR_IN3_PIN, GPIO_PIN_SET);

    HAL_GPIO_WritePin(ROC_MOTOR_GPIO_PORT1, ROC_MOTOR_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ROC_MOTOR_GPIO_PORT2, ROC_MOTOR_IN4_PIN, GPIO_PIN_RESET);
}

/*********************************************************************************
 *  Description:
 *              Drive motor forward
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              None
 *
 *  Author:
 *              ROC LiRen(2019.01.06)
**********************************************************************************/
void RocMotorRotateDirectionSet(uint8_t Direction)
{
    switch(Direction)
    {
        case ROC_MOTOR_STOPPED_ROTATE:  RocMotorRotateStopped();
                                        break;

        case ROC_MOTOR_FORWARD_ROTATE:  RocMotorRotateForward();
                                        break;

        case ROC_MOTOR_REVERSE_ROTATE:  RocMotorRotateReverse();
                                        break;

        default:    break;
    }
}

/*********************************************************************************
 *  Description:
 *              Motor driver init
 *
 *  Parameter:
 *              None
 *
 *  Return:
 *              The motor driver init status
 *
 *  Author:
 *              ROC LiRen(2019.01.06)
**********************************************************************************/
ROC_RESULT RocMotorInit(void)
{
    ROC_RESULT Ret = RET_OK;

    //RocMotorRotateDirectionSet(ROC_MOTOR_STOPPED_ROTATE);

    if(RET_OK != Ret)
    {
        ROC_LOGE("Motor speed set is in error!");
    }
    else
    {
        ROC_LOGI("Motor speed set is in success.");
    }

    Ret = RocMotorServoTurnAngleSet(ROC_MOTOR_SERVO_DEFAULT_ANGLE);
    if(RET_OK != Ret)
    {
        ROC_LOGE("Motor servo turn angle set is in error!");
    }
    else
    {
        ROC_LOGI("Motor servo turn angle set is in success.");
    }


    if(RET_OK != Ret)
    {
        ROC_LOGE("Motor driver init is in error!");
    }
    else
    {
        ROC_LOGI("Motor driver init is in success.");
    }

    return Ret;
}



#include "RocPca9685.h"
#include "RocRobotControl.h"


void RocRobotControlInit(void)
{
    RocPca9685Init();
    RocPca9685Enable();

    RocPca9685OutPwm(PWM_ADDRESS_L, 0, 0, 156);
    RocPca9685OutPwm(PWM_ADDRESS_H, 0, 0, 156);
//    RocPca9685OutPwm(PWM_ADDRESS_L, 2, 0, 256);
//    RocPca9685OutPwm(PWM_ADDRESS_L, 3, 0, 256);
}

void RocRobotControlMain(void)
{

}


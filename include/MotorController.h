#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "Esc.h"

class MotorController
{
   public:
    
        MotorController(const uint PIN_MOTOR_1, const uint PIN_MOTOR_2, const uint PIN_MOTOR_3, const uint PIN_MOTOR_4);

        void init();

        bool getIsInit();

    private:
        bool killSwitchOn = false;

        bool isInit = false;

        // Behind right
        Esc motor1;
        // Front right
        Esc motor2;
        // Behind left
        Esc motor3;
        // Front left
        Esc motor4;
};

#endif
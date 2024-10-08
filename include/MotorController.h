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

        void initSpecific(int motor);

        void uninit();

        void uninitSpecific(int motor);

        bool getIsInit(int motor);

        void control(int j1a, int j1b, int j2a, int j2b);

        int motor1Speed, motor2Speed, motor3Speed, motor4Speed;

        uint getMotor1Us();
        uint getMotor2Us();
        uint getMotor3Us();
        uint getMotor4Us();

        uint maxMotorSpeed = 40;

    private:
        static int map(int value, int in_min, int in_max, int out_min, int out_max);

        static int constrain(int value, int minValue, int maxValue);

        bool killSwitchOn = false;

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
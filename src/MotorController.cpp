#include "MotorController.h"

MotorController::MotorController(const uint PIN_MOTOR_1, const uint PIN_MOTOR_2, const uint PIN_MOTOR_3, const uint PIN_MOTOR_4):
    motor1(PIN_MOTOR_1),
    motor2(PIN_MOTOR_2),
    motor3(PIN_MOTOR_3),
    motor4(PIN_MOTOR_4){}

void MotorController::control(int joystickLeftX, int joystickLeftY, int joystickRightX, int joystickRightY){
    const float SENSITIVITY_FACTOR = 0.5f;
    
    // Motor speeds (0 to maxMotorSpeed)
    int throttle = MotorController::map(joystickLeftY, -100, 100, 0, maxMotorSpeed);

    // Direct joystick input (-100 to 100)
    int yaw = joystickLeftX * SENSITIVITY_FACTOR;
    int pitch = joystickRightY * SENSITIVITY_FACTOR;
    int roll = joystickRightX * SENSITIVITY_FACTOR;

    // Calculate motor speed
    motor1Speed = throttle + pitch + roll - yaw; // Front-left motor
    motor2Speed = throttle + pitch - roll + yaw; // Front-right motor
    motor3Speed = throttle - pitch + roll + yaw; // Rear-left motor
    motor4Speed = throttle - pitch - roll - yaw; // Rear-right motor

    motor1Speed = MotorController::constrain(motor1Speed, 0, maxMotorSpeed);
    motor2Speed = MotorController::constrain(motor2Speed, 0, maxMotorSpeed);
    motor3Speed = MotorController::constrain(motor3Speed, 0, maxMotorSpeed);
    motor4Speed = MotorController::constrain(motor4Speed, 0, maxMotorSpeed);

    motor1.setSpeed(motor1Speed);
    motor2.setSpeed(motor2Speed);
    motor3.setSpeed(motor3Speed);
    motor4.setSpeed(motor4Speed);
    return;
}

void MotorController::init(){
    // Check if not already init
    if(motor1.isInit){
        return;
    }

    //Calibrate all motor
    motor1.setSpeedUs(motor1.MAX_US);
    motor2.setSpeedUs(motor2.MAX_US);
    motor3.setSpeedUs(motor3.MAX_US);
    motor4.setSpeedUs(motor4.MAX_US);
    sleep_ms(5000);
    motor1.setSpeedUs(motor1.MIN_US);
    motor2.setSpeedUs(motor2.MIN_US);
    motor3.setSpeedUs(motor3.MIN_US);
    motor4.setSpeedUs(motor4.MIN_US);
    sleep_ms(5000);
    motor1.setSpeedUs(0);
    motor2.setSpeedUs(0);
    motor3.setSpeedUs(0);
    motor4.setSpeedUs(0);
    sleep_ms(2000);
    motor1.setSpeedUs(motor1.MIN_US);
    motor2.setSpeedUs(motor2.MIN_US);
    motor3.setSpeedUs(motor3.MIN_US);
    motor4.setSpeedUs(motor4.MIN_US);
    sleep_ms(1000);
    motor1.isInit = true;
    motor2.isInit = true;
    motor3.isInit = true;
    motor4.isInit = true;
}

uint MotorController::getMotor1Us(){
    return motor1.getCurrentUs();
}

uint MotorController::getMotor2Us(){
    return motor2.getCurrentUs();
}

uint MotorController::getMotor3Us(){
    return motor3.getCurrentUs();
}

uint MotorController::getMotor4Us(){
    return motor4.getCurrentUs();
}

void MotorController::initSpecific(int motor){
    if(motor == 1){
        motor1.calibrate();
    }else if(motor == 2){
        motor2.calibrate();
    }else if(motor == 3){
        motor3.calibrate();
    }else if(motor == 4){
        motor4.calibrate();
    }
}

void MotorController::uninitSpecific(int motor){
    if(motor == 1){
        motor1.stop();
    }else if(motor == 2){
        motor2.stop();
    }else if(motor == 3){
        motor3.stop();
    }else if(motor == 4){
        motor4.stop();
    }
}

void MotorController::uninit(){
    uninitSpecific(1);
    uninitSpecific(2);
    uninitSpecific(3);
    uninitSpecific(4);
}

bool MotorController::getIsInit(int motor){
    if(motor == 1){
        return motor1.isInit;
    }else if(motor == 2){
        return motor2.isInit;
    }else if(motor == 3){
        return motor3.isInit;
    }else if(motor == 4){
        return motor4.isInit;
    }

    return false;
}

int MotorController::map(int value, int in_min, int in_max, int out_min, int out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int MotorController::constrain(int value, int minValue, int maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

#include "MotorController.h"

MotorController::MotorController(const uint PIN_MOTOR_1, const uint PIN_MOTOR_2, const uint PIN_MOTOR_3, const uint PIN_MOTOR_4):
    motor1(PIN_MOTOR_1),
    motor2(PIN_MOTOR_2),
    motor3(PIN_MOTOR_3),
    motor4(PIN_MOTOR_4){}

void MotorController::control(int joystickLeftY, int joystickLeftX, int joystickRightY, int joystickRightX){
    // Motor speeds (0 to max_motor_speed)
    int throttle = MotorController::map(joystickLeftY, -100, 100, 0, MAX_MOTOR_SPEED_P);

    // Direct joystick input for yaw (still -100 to 100 range)
    int yaw = joystickLeftX;

    // Pitch input (-100 to 100 range)
    int pitch = joystickRightY;

    // Roll input (-100 to 100 range)
    int roll = joystickRightX;

    // Calculate motor speed
    int motor1Speed = throttle + pitch + roll - yaw; // Front-left motor
    int motor2Speed = throttle + pitch - roll + yaw; // Front-right motor
    int motor3Speed = throttle - pitch + roll + yaw; // Rear-left motor
    int motor4Speed = throttle - pitch - roll - yaw; // Rear-right motor

    motor1Speed = MotorController::constrain(motor1Speed, 0, MAX_MOTOR_SPEED_P);
    motor2Speed = MotorController::constrain(motor2Speed, 0, MAX_MOTOR_SPEED_P);
    motor3Speed = MotorController::constrain(motor3Speed, 0, MAX_MOTOR_SPEED_P);
    motor4Speed = MotorController::constrain(motor4Speed, 0, MAX_MOTOR_SPEED_P);

    count++;

    printf("Count: %d", count);

    //printf("MOTOR 1: %d. 2: %d. 3: %d. 4: %d.\n", motor1Speed, motor2Speed, motor3Speed, motor4Speed);
    return;
}

void MotorController::init(){
    // Check if not already init
    if(isInit){
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
    isInit = true;
}

void MotorController::uninit(){
    // Check if init
    if(!isInit){
        return;
    }

    //Calibrate all motor
    motor1.setSpeedUs(0);
    motor2.setSpeedUs(0);
    motor3.setSpeedUs(0);
    motor4.setSpeedUs(0);
    sleep_ms(3000);
    isInit = false;
}

bool MotorController::getIsInit(){
    return isInit;
}

int MotorController::map(int value, int in_min, int in_max, int out_min, int out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int MotorController::constrain(int value, int minValue, int maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

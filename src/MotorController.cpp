#include "MotorController.h"

MotorController::MotorController(const uint PIN_MOTOR_1, const uint PIN_MOTOR_2, const uint PIN_MOTOR_3, const uint PIN_MOTOR_4):
    motor1(PIN_MOTOR_1),
    motor2(PIN_MOTOR_2),
    motor3(PIN_MOTOR_3),
    motor4(PIN_MOTOR_4){}

void MotorController::init(){
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

bool MotorController::getIsInit(){
    return isInit;
}

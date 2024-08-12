#include "MPU6050.h"

MPU6050::MPU6050(I2C* i2cc):
    i2c(i2cc)
    {
        isCalibrated = false;
        rateCalibrationRoll=0;
        rateCalibrationPitch=0;
        rateCalibrationYaw=0;
    }

int MPU6050::init(){
    // Assure que la communication est avec le mpu
    uint8_t tempBuf = 0;

    // Démare mpu
    tempBuf = 0x00;
    if(i2c->reg_write(MPU6050_ADDR, 0x6B, &tempBuf, 1) == 0){
        return 1;
    }

    // Active low pass filter
    //tempBuf = 0x03;
    //if(i2c->reg_write(MPU6050_ADDR, 0x1A, &tempBuf, 1) == 0){
    //    return 2;
    //}

    // Sample rate divider
    tempBuf = 0x04;
    if(i2c->reg_write(MPU6050_ADDR, 0x19, &tempBuf, 1) == 0){
        return 2;
    }

    // Change le scale de la sensibilité gyro
    //tempBuf = 0x8;
    tempBuf = 0x00;
    if(i2c->reg_write(MPU6050_ADDR, 0x1B, &tempBuf, 1) == 0){
        return 1;
    }

    // Change le scale de la sensibilité accelero
    tempBuf = 0x00;
    if(i2c->reg_write(MPU6050_ADDR, 0x1C, &tempBuf, 1) == 0){
        return 1;
    }

    // Configure du interupt
    tempBuf = 0x00;
    if(i2c->reg_write(MPU6050_ADDR, 0x37, &tempBuf, 1) == 0){
        return 1;
    }

    // Active le interupt
    tempBuf = 0x01;
    if(i2c->reg_write(MPU6050_ADDR, 0x38, &tempBuf, 1) == 0){
        return 1;
    }
    
    // Set offsets to zero
    if (i2c->reg_write(MPU6050_ADDR, 0x06, 0x00, 1) == 0 ||
        i2c->reg_write(MPU6050_ADDR, 0x07, 0x00, 1) == 0 ||
        i2c->reg_write(MPU6050_ADDR, 0x08, 0x00, 1) == 0 ||
        i2c->reg_write(MPU6050_ADDR, 0x09, 0x00, 1) == 0 ||
        i2c->reg_write(MPU6050_ADDR, 0x0A, 0x00, 1) == 0 ||
        i2c->reg_write(MPU6050_ADDR, 0x0B, 0x00, 1) == 0) {
        return 5;
    }

    return 0;
}

int MPU6050::getDataAccel(){
    uint8_t data[6] = {0};
    if(i2c->reg_read(MPU6050_ADDR, 0x3B, data, 6) == 0){
        return 1;
    }

    // Convert 2 bytes to 16 bit int
    accelX = ((data[0] << 8) | data[1]);
    accelXProcessed = accelX/LSB_ACCEL;
    accelY = ((data[2] << 8) | data[3]);
    accelYProcessed = accelY/LSB_ACCEL;
    accelZ = ((data[4] << 8) | data[5]);
    accelZProcessed = accelZ/LSB_ACCEL;

    return 0;
}

int MPU6050::getDataGyro(){
    uint8_t data[6] = {0};
    if(i2c->reg_read(MPU6050_ADDR, 0x43, data, 6) == 0){
        return 1;
    }

    // Convert 2 bytes to 16 bit int
    gyroX = (int16_t)((data[0] << 8) | data[1]);
    gyroXProcessed = (gyroX / 131.0f) - rateCalibrationRoll;
    gyroY = (int16_t)((data[2] << 8) | data[3]);
    gyroYProcessed = (gyroY / 131.0f) - rateCalibrationPitch;
    gyroZ = (int16_t)((data[4] << 8) | data[5]);
    gyroZProcessed = (gyroZ / 131.0f) - rateCalibrationYaw;

    return 0;
}

int MPU6050::calibrate(){
    float temprateCalibrationRoll, temprateCalibrationPitch, temprateCalibrationYaw = 0;
    for(int i=0; i<1000; i++){
        if(getDataGyro() != 0){
            return 1;
        }
        temprateCalibrationRoll += gyroXProcessed;
        temprateCalibrationPitch += gyroYProcessed;
        temprateCalibrationYaw += gyroZProcessed;
        sleep_ms(1);
    }
    rateCalibrationRoll = temprateCalibrationRoll/1000;
    rateCalibrationPitch = temprateCalibrationPitch/1000;
    rateCalibrationYaw = temprateCalibrationYaw/1000;

    isCalibrated = true;

    return 0;
}
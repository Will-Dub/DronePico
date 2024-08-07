#include "MPU6050.h"

MPU6050::MPU6050(I2C* i2cc):
    i2c(i2cc)
    {
        is_calibrated = false;
        rateCalibrationRoll=0;
        rateCalibrationPitch=0;
        rateCalibrationYaw=0;
    }

int MPU6050::init(){
    // Assure que la communication est avec le mpu
    uint8_t temp_buf = 0;

    // Démare mpu
    temp_buf = 0x00;
    if(i2c->reg_write(MPU6050_ADDR, 0x6B, &temp_buf, 1) == 0){
        return 1;
    }

    // Active low pass filter
    //temp_buf = 0x03;
    //if(i2c->reg_write(MPU6050_ADDR, 0x1A, &temp_buf, 1) == 0){
    //    return 2;
    //}

    // Sample rate divider
    temp_buf = 0x04;
    if(i2c->reg_write(MPU6050_ADDR, 0x19, &temp_buf, 1) == 0){
        return 2;
    }

    // Change le scale de la sensibilité gyro
    //temp_buf = 0x8;
    temp_buf = 0x00;
    if(i2c->reg_write(MPU6050_ADDR, 0x1B, &temp_buf, 1) == 0){
        return 1;
    }

    // Change le scale de la sensibilité accelero
    temp_buf = 0x00;
    if(i2c->reg_write(MPU6050_ADDR, 0x1C, &temp_buf, 1) == 0){
        return 1;
    }

    // Configure du interupt
    temp_buf = 0x00;
    if(i2c->reg_write(MPU6050_ADDR, 0x37, &temp_buf, 1) == 0){
        return 1;
    }

    // Active le interupt
    temp_buf = 0x01;
    if(i2c->reg_write(MPU6050_ADDR, 0x38, &temp_buf, 1) == 0){
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

int MPU6050::get_data_accel(){
    uint8_t data[6] = {0};
    if(i2c->reg_read(MPU6050_ADDR, 0x3B, data, 6) == 0){
        return 1;
    }

    // Convert 2 bytes to 16 bit int
    accelX = ((data[0] << 8) | data[1]);
    accelX_processed = accelX/LSB_ACCEL;
    accelY = ((data[2] << 8) | data[3]);
    accelY_processed = accelY/LSB_ACCEL;
    accelZ = ((data[4] << 8) | data[5]);
    accelZ_processed = accelZ/LSB_ACCEL;

    return 0;
}

int MPU6050::get_data_gyro(){
    uint8_t data[6] = {0};
    if(i2c->reg_read(MPU6050_ADDR, 0x43, data, 6) == 0){
        return 1;
    }

    // Convert 2 bytes to 16 bit int
    gyroX = (int16_t)((data[0] << 8) | data[1]);
    gyroX_processed = (gyroX / 131.0f) - rateCalibrationRoll;
    gyroY = (int16_t)((data[2] << 8) | data[3]);
    gyroY_processed = (gyroY / 131.0f) - rateCalibrationPitch;
    gyroZ = (int16_t)((data[4] << 8) | data[5]);
    gyroZ_processed = (gyroZ / 131.0f) - rateCalibrationYaw;

    return 0;
}

int MPU6050::calibrate(){
    float temp_rateCalibrationRoll, temp_rateCalibrationPitch, temp_rateCalibrationYaw = 0;
    for(int i=0; i<1000; i++){
        if(get_data_gyro() != 0){
            return 1;
        }
        temp_rateCalibrationRoll += gyroX_processed;
        temp_rateCalibrationPitch += gyroY_processed;
        temp_rateCalibrationYaw += gyroZ_processed;
        sleep_ms(1);
    }
    rateCalibrationRoll = temp_rateCalibrationRoll/1000;
    rateCalibrationPitch = temp_rateCalibrationPitch/1000;
    rateCalibrationYaw = temp_rateCalibrationYaw/1000;

    is_calibrated = true;

    return 0;
}
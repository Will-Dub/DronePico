#include "I2C.h"

#ifndef MPU6050_H
#define MPU6050_H

class MPU6050
{
    public:

        int16_t gyroX, gyroY, gyroZ;
        float gyroX_processed, gyroY_processed, gyroZ_processed;

        int16_t accelX, accelY, accelZ;
        float accelX_processed, accelY_processed, accelZ_processed;

        static const uint8_t MPU6050_ADDR = 0x68;
        static const uint8_t REG_DEVID = 0x00;

        //Constante
        static constexpr float LSB_ACCEL = 16384.0;
        static constexpr float DEGREE_GYRO = 131.0;
        static const uint8_t MPU6050_DEVID = 0x83;
        static constexpr float SENSITIVITY_2G = 1.0 / 256;  // (g/LSB)
        static constexpr float EARTH_GRAVITY = 9.80665;

        MPU6050(I2C* i2c);

        int calibrate();

        int get_data_accel();

        int get_data_gyro();

        int init();

    private:
        I2C* i2c;

        bool is_calibrated;

        //Calibrations
        float rateCalibrationRoll, rateCalibrationPitch, rateCalibrationYaw;
};

#endif
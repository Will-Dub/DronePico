#ifndef DRONE_H
#define DRONE_H

#include <cstring>
#include <stdio.h>
#include <sstream>
#include <stdio.h>
#include <sstream>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>

#include "pico/stdlib.h"
#include "I2C.h"
#include "MPU6050.h"
#include "QMC5883L.h"
#include "UART.h"
#include "TinyGPS++.h"
#include "Message.h"

#define PI 3.14159265358979323846
#define RAD180 (180 * PI)

static const int RXPin_GPS = 9, TXPin_GPS = 8;
static const int RXPin_ZERO = 1, TXPin_ZERO = 0;
static const int SdaPin_I2C = 26, SclPin_I2C = 27;
const char MPU6050_DATA_READY_PIN = 17;
const char QMC5883L_DATA_READY_PIN = 16;

class Drone
{
    public:
        Drone();

        void init();

        void start();

        void stop();

        bool getIsZeroConnected();

        bool getIsGpsConnected();

        bool getIsI2CConnected();

        void sensorRead();

        void setDestination(double desiredLatitude, double desiredLongitude, double desiredAltitude, int desiredSpeed);

        void setFlightMode(FlightMode mode);

        void setRotation(float desiredPitch, float desiredRoll, float desiredYaw);

        void setUseMpu6050(bool enable);

        void setUseQmc5883l(bool enable);

        void setUseGps(bool enable);

        void setUseLog(bool enable);

        SensorData getSensorData();

        PositionData getPositionData();

        StatusData getStatusData();

        void sendMessage(Message message);

        std::optional<Message> receiveMessage();

        void setDataReadyQMC5883L();

        void setDataReadyMPU6050();

        bool isNewPositionData();

        bool isNewSensorData();

        void log(const std::string& data, LogType dataType = LogType::LOG_INFO);

    private:
        bool dataReadyMpu6050 = false;
        bool dataReadyQmc5883l = false;
        bool newSensorData = false;
        bool newPositionData = false;

        ModeData modeData;
        PositionData positionData;
        StatusData statusData;
        SensorData sensorData;

        I2C i2c;
        MPU6050 mpu6050;
        QMC5883L qmc5883l;
        UART uartGps;
        UART uartZero;
        TinyGPSPlus gps;

        bool qmc5883lError = false;
        bool mpu6050Error = false;

        const int timeout = 10000;
    private:
        bool isDataReceivedWithinTimeout(uint64_t lastReceivedTime);
};

#endif
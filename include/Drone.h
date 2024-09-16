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
#include "Lora.h"
#include "MPU6050.h"
#include "QMC5883L.h"
#include "UART.h"
#include "TinyGPS++.h"
#include "MotorController.h"

#define PI 3.14159265358979323846
#define RAD180 (180 * PI)

// Data structure
enum class MessageType {
    ControlData,
    ModeData,
    PositionData,
    RequestData,
    StatusData,
    SensorData,
    LogData,
};

enum LogType : uint8_t {
    LOG_INFO,
    LOG_ERROR,
    LOG_CRITICAL
};

enum FlightMode {
    MANUAL,
    STABILIZE,
    ALT_HOLD,
    AUTO
};

struct ModeData {
    // State information
    FlightMode mode;
    bool failSafeTriggered;
    double desiredLatitude, desiredLongitude, desiredAltitude, desiredSpeed;

    // Control parameters
    float desiredPitch, desiredRoll, desiredYaw;
};

struct PositionData {
    double gpsLatitude, gpsLongitude, gpsAltitude, gpsKmph, gpsCourseDeg;
};

struct StatusData {
    bool uartZeroConnected;
    bool uartGpsConnected;
    bool i2cConnected;
    bool loraConnected;

    bool useMotor;
    bool useMpu6050;
    bool useQmc5883l;
    bool useGps;
    bool useLog;
    
    StatusData(
        bool uartZeroConnected = false,
        bool uartGpsConnected = false,
        bool i2cConnected = false,
        bool loraConnected = false,
        bool useMotor = false,
        bool useMpu6050 = true,
        bool useQmc5883l = true,
        bool useGps = true,
        bool useLog = true
    )
        : uartZeroConnected(uartZeroConnected),
          uartGpsConnected(uartGpsConnected),
          i2cConnected(i2cConnected),
          loraConnected(loraConnected),
          useMotor(useMotor),
          useMpu6050(useMpu6050),
          useQmc5883l(useQmc5883l),
          useGps(useGps),
          useLog(useLog) {}
};

struct SensorData {
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    int16_t magX, magY, magZ;
    float pitch, roll, yaw;
};

//Constants
const int RX_PIN_GPS = 5, TX_PIN_GPS = -1;
const int RX_PIN_ZERO = 1, TX_PIN_ZERO = 0;
const int SDA_PIN_I2C = 26, SCL_PIN_I2C = 27;
const int MOTOR_1_PIN = 28, MOTOR_2_PIN = 28, MOTOR_3_PIN = 28, MOTOR_4_PIN = 28;
const char MPU6050_DATA_READY_PIN = 22;
const char QMC5883L_DATA_READY_PIN = 21;
const long LORA_MHZ = 433.425E6;

class Drone
{
    public:
        Drone(uint droneId);

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

        uint getDroneId();

        void SendDataPacketUart(DataPacket dataPacket);

        void SendDataPacketLora(DataPacket dataPacket);

        std::optional<DataPacket> receiveDataPacketUart();

        std::optional<DataPacket> receiveDataPacketLora();

        void setDataReadyQMC5883L();

        void setDataReadyMPU6050();

        bool isNewPositionData();

        bool isNewSensorData();

        void motorInit();

        void motorUninit();

        void log(const std::string& data, LogType logType = LogType::LOG_INFO);

        std::optional<DataPacket> handleDataPacket(DataPacket dataPacket);

    private:
        // New data
        bool dataReadyMpu6050 = false;
        bool dataReadyQmc5883l = false;
        bool newSensorData = false;
        bool newPositionData = false;

        // Data
        ModeData modeData;
        PositionData positionData;
        StatusData statusData;
        SensorData sensorData;

        // Component of the drone
        I2C i2c;
        MPU6050 mpu6050;
        QMC5883L qmc5883l;
        UART uartGps;
        UART uartZero;
        TinyGPSPlus gps;
        Lora lora;
        MotorController motorController;

        // Error
        bool qmc5883lError = false;
        bool mpu6050Error = false;

        uint nextPacketId;

        const uint DRONE_ID;
        const int TIMEOUT = 10000;
    private:
        bool isDataReceivedWithinTimeout(uint64_t lastReceivedTime);
};

#endif
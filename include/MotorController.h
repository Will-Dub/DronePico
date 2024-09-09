#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "Esc.h"

enum class MessageType {
    ControlData,
    ModeData,
    PositionData,
    RequestData,
    StatusData,
    SensorData,
    LogData,
};

//-----------------------------
//All enum used by message type

struct ControlData {
    //Each motor control
    uint8_t motor_pwm[4];
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

enum RequestType {
    STATUS_REQUEST,
    POSITION_REQUEST,
    MODE_REQUEST,
    CONTROL_REQUEST,
    SENSOR_REQUEST,
};

//-----------------------------
//All message type
struct LogData {
    // Log data
    LogType type;
    char message[30];
};

struct RequestData{
    RequestType requestType;
};

class MotorController
{
   public:
    
        MotorController(const uint PIN_MOTOR_1, const uint PIN_MOTOR_2, const uint PIN_MOTOR_3, const uint PIN_MOTOR_4);

        void init();

        void uninit();

        bool getIsInit();

        void control(int j1a, int j1b, int j2a, int j2b);

    private:
        static int map(int value, int in_min, int in_max, int out_min, int out_max);

        static int constrain(int value, int minValue, int maxValue);

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

        const uint MAX_MOTOR_SPEED_P = 100;
};

#endif
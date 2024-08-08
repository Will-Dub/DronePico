#include <stdio.h>
#include <sstream>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include "pico/multicore.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "I2C.h"
#include "MPU6050.h"
#include "QMC5883L.h"
#include "UART.h"
#include "TinyGPS++.h"
#include "Message.h"

#define PI 3.14159265358979323846
#define RAD180 (180 * PI)

static const int RXPin_GPS = 9, TXPin_GPS = 8;
const char MPU6050_DATA_READY_PIN = 17;
const char QMC5883L_DATA_READY_PIN = 16;

const int timeout = 10000;

volatile bool data_ready_qmc5883l = false;
volatile bool data_ready_mpu6050 = false;

/*******************************************************************************
 * Function Definitions
 */

void log(UART* uart_out, const std::string& data, LogType dataType = LogType::LOG_INFO) {
    Message message;
    message.type = MessageType::LogData;
    message.data.log_data.type = dataType;

    std::strncpy(message.data.log_data.message, data.c_str(), sizeof(message.data.log_data.message) - 1);
    message.data.log_data.message[sizeof(message.data.log_data.message) - 1] = '\0';

    uart_out->writeMessage(message);
}

void interrupt_core1(uint gpio, uint32_t events) {
    if(gpio == QMC5883L_DATA_READY_PIN){
        data_ready_qmc5883l = true;
    }
    else if(gpio == MPU6050_DATA_READY_PIN){
        data_ready_mpu6050 = true;
    }
}

bool data_received_within_timeout(uint64_t last_receive_time) {
    uint64_t current_ms = to_us_since_boot(get_absolute_time()) / 1000;
    uint64_t last_ms = last_receive_time / 1000;

    return (current_ms - last_ms <= timeout);
}

/*******************************************************************************
 * Main
 */
int main() {
    stdio_init_all();

    //----------------------------------------------------------------------
    //Variable declaration
    bool error_qmc5883l;
    PositionData position_data = {};
    SensorData sensor_data = {};
    I2C i2c = I2C(i2c1, 26, 27, 100*1000);
    MPU6050 mpu6050(&i2c);
    QMC5883L qmc5883l(&i2c);
    UART uart_zero(uart0, 230400, 1, 0);
    UART uart_gps(uart1, 9600, RXPin_GPS, TXPin_GPS);
    TinyGPSPlus gps;

    //Variable init
    data_ready_mpu6050 = false;
    data_ready_qmc5883l = false;


    //----------------------------------------------------------------------
    //Initialise the sensors
    //MPU 6050 initialisation + calibration
    i2c.setup();

    log(&uart_zero, "INIT MPU6050");
    if(mpu6050.init()){
        log(&uart_zero, "MPU6050 ERREUR DURANT INIT", LogType::LOG_CRITICAL);
    }
    log(&uart_zero, "FIN INIT MPU6050");

    log(&uart_zero, "CALIBRATING MPU6050...");
    if(mpu6050.calibrate()){
        log(&uart_zero, "MPU6050 ERREUR DURANT LA CALIBRATION", LogType::LOG_CRITICAL);
    }
    log(&uart_zero, "CALIBRATING FINISHED MPU6050");

    //QMC6883L initialisation + config
    log(&uart_zero, "CONFIG DEBUT QMC5883L");
    if (!qmc5883l.begin())
    {
        error_qmc5883l = true;
    }
    else
    {
        qmc5883l.setConfig({ QMC5883L::Mode::continuous,
                            QMC5883L::OutputDataRate::odr_10hz,
                            QMC5883L::FullScaleRange::rng_8g,
                            QMC5883L::OverSampleRate::osr_512,
                            false                              });

        if (!qmc5883l.writeConfig())
        {
            error_qmc5883l = true;
        }
        else
        {
            qmc5883l.setCalibration({ {-281.852962, 
                                        -27.985588,
                                        557.383102 },
                                   {{   0.805536,   -0.000364,  -0.011148},
                                    {  -0.000364,   0.819217,   -0.090000},
                                    {  -0.011148,    -0.090000,   0.785799} }});

            error_qmc5883l = false;
        }
    }
    log(&uart_zero, "CONFIG FINI QMC5883L");

    if(error_qmc5883l){
        log(&uart_zero, "CONFIG QMC5883L FAILED", LogType::LOG_CRITICAL);
    }

    //Set interupts
    //QMC5883l
    gpio_set_irq_enabled_with_callback(QMC5883L_DATA_READY_PIN, GPIO_IRQ_EDGE_RISE, true, &interrupt_core1);

    //MPU6050
    gpio_set_irq_enabled_with_callback(MPU6050_DATA_READY_PIN, GPIO_IRQ_EDGE_RISE, true, &interrupt_core1);

    //----------------------------------------------------------------------
    log(&uart_zero, "INIT END");

    //----------------------------------------------------------------------
    //MAIN LOOP
    int messageCount = 0;
    auto startTime = std::chrono::steady_clock::now();

    while (true) {
        bool new_data = false;
        //----------------------------------------------------------------------
        //Read the sensor data

        //Data from qmc5883
        if(data_ready_qmc5883l){
            if (!qmc5883l.readData())
            {
                log(&uart_zero, "QMC error: " + qmc5883l.lastError(), LogType::LOG_ERROR);
            }else{
                new_data = true;
            }
            data_ready_qmc5883l = false;
        }

        //Data from mpu6050
        if(data_ready_mpu6050){
            if(mpu6050.get_data_accel() || mpu6050.get_data_gyro()){
                log(&uart_zero, "MPU6050 ERREUR DURANT LECTURE", LogType::LOG_ERROR);
            }else{
                new_data = true;
            }
            data_ready_mpu6050 = false;
        }

        //Data from gps
        while (uart_is_readable(uart1)) {
            char c = uart_getc(uart1);
            gps.encode(c);
            if (gps.location.isUpdated()) {
                new_data = true;
            }
        }
        
        //----------------------------------------------------------------------
        //Process and store the data
        if(new_data){
            sensor_data.accel_x = mpu6050.accelX_processed;
            sensor_data.accel_y = mpu6050.accelY_processed;
            sensor_data.accel_z = mpu6050.accelZ_processed;
            sensor_data.gyro_x = mpu6050.gyroX_processed;
            sensor_data.gyro_y = mpu6050.gyroY_processed;
            sensor_data.gyro_z = mpu6050.gyroZ_processed;
            sensor_data.mag_x = qmc5883l.calibratedDataX();
            sensor_data.mag_y = qmc5883l.calibratedDataY();
            sensor_data.mag_z = qmc5883l.calibratedDataZ();
            sensor_data.pitch = atan2(sensor_data.accel_x, sqrt(sensor_data.accel_y * sensor_data.accel_y + sensor_data.accel_z * sensor_data.accel_z)) * RAD180;
            sensor_data.roll = atan2(sensor_data.accel_y, sqrt(sensor_data.accel_x * sensor_data.accel_x + sensor_data.accel_z * sensor_data.accel_z)) * RAD180;
            sensor_data.yaw = atan2(sensor_data.mag_y, sensor_data.mag_x) * RAD180;
            
            position_data.gps_latitude = gps.location.lat();
            position_data.gps_longitude = gps.location.lng();
            position_data.gps_altitude = gps.altitude.meters();
            position_data.gps_kmph = gps.speed.kmph();
            position_data.gps_course_deg = gps.course.deg();
        }
        /*
        local_sensor_data.uart_gps_connected = data_received_within_timeout(uart_gps.get_last_receive_time());
        local_sensor_data.i2c_connected = data_received_within_timeout(i2c.get_last_receive_time());
        local_sensor_data.uart_zero_connected = data_received_within_timeout(uart_zero->get_last_receive_time());*/

        //----------------------------------------------------------------------
        //Handle new data from pi zero
        if(uart_zero.isNewDataReceived()){
            std::string data = uart_zero.getReceivedData();
            //uart_zero->write(data);
        }

        /*if (new_data) {
            Message message;
            message.type = MessageType::PositionData;
            message.data.position_data = position_data;

            uart_zero.writeMessage(message);
        }*/

        messageCount++;

        //----------------------------------------------------------------------
        //Process motor and sensor data together

        //----------------------------------------------------------------------
        //Send data to motor

        //----------------------------------------------------------------------
        //Send a copy of the sensor data to the pi zero

        auto currentTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
        if (elapsedTime >= 5) {
            log(&uart_zero, std::to_string(messageCount));
            messageCount = 0;
            startTime = std::chrono::steady_clock::now();
        }
    }
}

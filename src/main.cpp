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

volatile SensorData shared_sensor_data;
volatile bool shared_sensor_data_ready = false;
mutex_t data_mutex;
mutex_t zero_mutex;

/*******************************************************************************
 * Function Definitions
 */

void log(UART* uart_out, const std::string& data, mutex_t* uart_mutex = nullptr, LogType dataType = LogType::LOG_INFO) {
    Message message;
    message.type = MessageType::LogData;
    message.data.log_data.type = dataType;

    std::strncpy(message.data.log_data.message, data.c_str(), sizeof(message.data.log_data.message) - 1);
    message.data.log_data.message[sizeof(message.data.log_data.message) - 1] = '\0';

    if (uart_mutex != nullptr) {
        mutex_enter_blocking(uart_mutex);
    }

    uart_out->writeMessage(message);

    if (uart_mutex != nullptr) {
        mutex_exit(uart_mutex);
    }
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

// Lis les données des sensors et fait les calcul de stabilisation
void readSensorsAndCalculateBasicData(){
    //----------------------------------------------------------------------
    //Variable declaration
    bool error_qmc5883l;
    bool new_data_gps;
    float rateCalibrationRoll, rateCalibrationPitch, rateCalibrationYaw;
    SensorData local_sensor_data;

    //Variable init
    data_ready_mpu6050 = false;
    data_ready_qmc5883l = false;

    //----------------------------------------------------------------------
    //Get arguments
    UART* uart_zero = reinterpret_cast<UART*>(multicore_fifo_pop_blocking());

    //----------------------------------------------------------------------
    //Assign the sensor variable
    //SDA = 12, SCL = 13
    I2C i2c = I2C(i2c1, 26, 27, 100*1000);

    i2c.setup();

    UART uart_gps(uart1, 9600, RXPin_GPS, TXPin_GPS);

    MPU6050 mpu6050(&i2c);

    QMC5883L qmc5883l(&i2c);

    TinyGPSPlus gps;

    //----------------------------------------------------------------------
    //Initialise the sensors
    //MPU 6050 initialisation + calibration

    log(uart_zero, "INIT MPU6050", &zero_mutex);
    if(mpu6050.init()){
        log(uart_zero, "MPU6050 ERREUR DURANT INIT", &zero_mutex, LogType::LOG_CRITICAL);
    }
    log(uart_zero, "FIN INIT MPU6050", &zero_mutex);

    log(uart_zero, "CALIBRATING MPU6050...", &zero_mutex);
    if(mpu6050.calibrate()){
        log(uart_zero, "MPU6050 ERREUR DURANT LA CALIBRATION", &zero_mutex, LogType::LOG_CRITICAL);
    }
    log(uart_zero, "CALIBRATING FINISHED MPU6050", &zero_mutex);

    //QMC6883L initialisation + config
    log(uart_zero, "CONFIG DEBUT QMC5883L", &zero_mutex);
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
    log(uart_zero, "CONFIG FINI QMC5883L", &zero_mutex);

    if(error_qmc5883l){
        log(uart_zero, "CONFIG QMC5883L FAILED", &zero_mutex, LogType::LOG_CRITICAL);
    }

    //Set interupts
    //QMC5883l
    gpio_set_irq_enabled_with_callback(QMC5883L_DATA_READY_PIN, GPIO_IRQ_EDGE_RISE, true, &interrupt_core1);

    //MPU6050
    gpio_set_irq_enabled_with_callback(MPU6050_DATA_READY_PIN, GPIO_IRQ_EDGE_RISE, true, &interrupt_core1);

    //----------------------------------------------------------------------
    log(uart_zero, "CORE INIT END", &zero_mutex);

    //----------------------------------------------------------------------
    //MAIN LOOP
    while (true) {
        bool new_data = false;
        //----------------------------------------------------------------------
        //Read the data
        //Data from zero
        mutex_enter_blocking(&zero_mutex);
        uart_zero->readData();
        mutex_exit(&zero_mutex);

        //Data from qmc
        if(data_ready_qmc5883l){
            if (!qmc5883l.readData())
            {
                log(uart_zero, "QMC error: " + qmc5883l.lastError(), &zero_mutex, LogType::LOG_ERROR);
            }else{
                new_data = true;
            }
            data_ready_qmc5883l = false;
        }

        //Data from mpu6050
        if(data_ready_mpu6050){
            if(mpu6050.get_data_accel() || mpu6050.get_data_gyro()){
                log(uart_zero, "MPU6050 ERREUR DURANT LECTURE", &zero_mutex, LogType::LOG_ERROR);
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
            local_sensor_data.accel_x = mpu6050.accelX_processed;
            local_sensor_data.accel_y = mpu6050.accelY_processed;
            local_sensor_data.accel_z = mpu6050.accelZ_processed;
            local_sensor_data.gyro_x = mpu6050.gyroX_processed;
            local_sensor_data.gyro_y = mpu6050.gyroY_processed;
            local_sensor_data.gyro_z = mpu6050.gyroZ_processed;
            local_sensor_data.mag_x = qmc5883l.calibratedDataX();
            local_sensor_data.mag_y = qmc5883l.calibratedDataY();
            local_sensor_data.mag_z = qmc5883l.calibratedDataZ();
            local_sensor_data.pitch = atan2(local_sensor_data.accel_x, sqrt(local_sensor_data.accel_y * local_sensor_data.accel_y + local_sensor_data.accel_z * local_sensor_data.accel_z)) * RAD180;
            local_sensor_data.roll = atan2(local_sensor_data.accel_y, sqrt(local_sensor_data.accel_x * local_sensor_data.accel_x + local_sensor_data.accel_z * local_sensor_data.accel_z)) * RAD180;
            local_sensor_data.yaw = atan2(local_sensor_data.mag_y, local_sensor_data.mag_x) * RAD180;

            local_sensor_data.gps_latitude = gps.location.lat();
            local_sensor_data.gps_longitude = gps.location.lng();
            local_sensor_data.gps_altitude = gps.altitude.meters();
            local_sensor_data.gps_kmph = gps.speed.kmph();
            local_sensor_data.gps_course_deg = gps.course.deg();
        }

        local_sensor_data.uart_gps_connected = data_received_within_timeout(uart_gps.get_last_receive_time());
        local_sensor_data.i2c_connected = data_received_within_timeout(i2c.get_last_receive_time());
        local_sensor_data.uart_zero_connected = data_received_within_timeout(uart_zero->get_last_receive_time());

        //----------------------------------------------------------------------
        //Send the data to the other core
        mutex_enter_blocking(&data_mutex);
        memcpy((void*)&shared_sensor_data, &local_sensor_data, sizeof(SensorData));
        shared_sensor_data_ready = true;
        mutex_exit(&data_mutex);
    }
}

// Recois les données du zero, fait des calcul avec les données des sensors et l'envoie aux moteurs
void controlMotors(UART* uart_zero){
    //----------------------------------------------------------------------
    //Variable declaration
    SensorData local_sensor_data;

    log(uart_zero, "CORE INIT END", &zero_mutex);

    //----------------------------------------------------------------------
    //MAIN LOOP
    while (true) {
        //----------------------------------------------------------------------
        //Handle new data from pi zero
        mutex_enter_blocking(&zero_mutex);
        if(uart_zero->isNewDataReceived()){
            std::string data = uart_zero->getReceivedData();
            //uart_zero->write(data);
        }
        mutex_exit(&zero_mutex);

        //----------------------------------------------------------------------
        //Read sensor data from the other core
        mutex_enter_blocking(&data_mutex);
        bool data_ready = shared_sensor_data_ready;
        mutex_exit(&data_mutex);

        if (data_ready) {
            // Acquire lock, read shared data, and release lock
            mutex_enter_blocking(&data_mutex);
            memcpy(&local_sensor_data, (void*)&shared_sensor_data, sizeof(SensorData));
            shared_sensor_data_ready = false;
            mutex_exit(&data_mutex);

            mutex_enter_blocking(&zero_mutex);
            //uart_zero->writeBlock((const uint8_t *)&local_sensor_data, sizeof(SensorData));

            Message message;
            message.type = MessageType::SensorData;
            message.data.sensor_data = local_sensor_data;

            uart_zero->writeMessage(message);
            mutex_exit(&zero_mutex);
        }

        //----------------------------------------------------------------------
        //Process motor and sensor data together

        //----------------------------------------------------------------------
        //Send data to motor

        //----------------------------------------------------------------------
        //Send a copy of the sensor data to the pi zero

        sleep_ms(100);
    }
}

/*******************************************************************************
 * Main
 */
int main() {
    stdio_init_all();

    // Initialize the mutex
    mutex_init(&data_mutex);
    mutex_init(&zero_mutex);

    //Initialise communication with pizero
    UART uart_zero(uart0, 230400, 1, 0);

    //Commence le core 1
    multicore_launch_core1(readSensorsAndCalculateBasicData);

    //Passe les arguments au core 1
    multicore_fifo_push_blocking(reinterpret_cast<uint32_t>(&uart_zero));
    
    controlMotors(&uart_zero);
}

#include <stdio.h>
#include <sstream>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include "pico/multicore.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "i2c.h"
#include "mpu6050.h"
#include "qmc5883l.h"
#include "uart.h"
#include "TinyGPS++.h"

#define PI 3.14159265358979323846
#define RAD180 (180 * PI)

static const int RXPin_GPS = 9, TXPin_GPS = 8;
const char mpu6050_data_ready_pin = 17;
const char qmc5883l_data_ready_pin = 16;

volatile bool data_ready_qmc5883l = false;
volatile bool data_ready_mpu6050 = false;

// Shared data structure
struct SensorData {
    float pitch;
    float roll;
    float yaw;
    float gps_latitude;
    float gps_longitude;
    float gps_altitude;
};

volatile SensorData shared_data;
volatile bool shared_data_ready = false;
mutex_t data_mutex;
mutex_t zero_mutex;

/*******************************************************************************
 * Function Definitions
 */

void interrupt_core1(uint gpio, uint32_t events) {
    if(gpio == qmc5883l_data_ready_pin){
        data_ready_qmc5883l = true;
    }
    else{
        data_ready_mpu6050 = true;
    }
}

// Lis les données des sensors et fait les calcul de stabilisation
void readSensorsAndCalculateBasicData(){
    //----------------------------------------------------------------------
    //Variable declaration
    bool error_qmc5883l;
    bool new_data_gps;
    float rateCalibrationRoll, rateCalibrationPitch, rateCalibrationYaw;
    SensorData local_data;

    //----------------------------------------------------------------------
    //Get arguments
    UART* uart_zero = reinterpret_cast<UART*>(multicore_fifo_pop_blocking());

    //----------------------------------------------------------------------
    //Assign the sensor variable
    //SDA = 12, SCL = 13
    I2C i2c = I2C(i2c0, 12, 13, 400*1000);

    i2c.setup();

    UART uart_gps(uart1, 9600, RXPin_GPS, TXPin_GPS);

    MPU6050 mpu6050(&i2c);

    QMC5883L qmc5883l(&i2c);

    TinyGPSPlus gps;

    //----------------------------------------------------------------------
    //Initialise the sensors
    //MPU 6050 initialisation + calibration

    printf("INIT MPU6050\n");
    if(mpu6050.init()){
        printf("MPU6050 ERREUR DURANT INIT\n");
    }
    printf("FIN INIT MPU6050\n");

    printf("CALIBRATING MPU6050...\n");
    if(mpu6050.calibrate()){
        printf("MPU6050 ERREUR DURANT LA CALIBRATION\n");
    }
    printf("CALIBRATING FINISHED MPU6050\n");

    //QMC6883L initialisation + config
    printf("CONFIG DEBUT QMC5883L\n");
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
    printf("CONFIG FINI QMC5883L\n");

    if(error_qmc5883l){
        printf("CONFIG QMC5883L FAILED\n");
    }

    //Set interupts
    //QMC5883l
    gpio_init(qmc5883l_data_ready_pin);
    gpio_set_dir(qmc5883l_data_ready_pin, GPIO_IN);
    gpio_pull_down(qmc5883l_data_ready_pin);
    gpio_set_irq_enabled_with_callback(qmc5883l_data_ready_pin, GPIO_IRQ_EDGE_RISE, true, &interrupt_core1);

    //MPU6050
    gpio_init(mpu6050_data_ready_pin);
    gpio_set_dir(mpu6050_data_ready_pin, GPIO_IN);
    gpio_pull_down(mpu6050_data_ready_pin);
    gpio_set_irq_enabled_with_callback(mpu6050_data_ready_pin, GPIO_IRQ_EDGE_RISE, true, &interrupt_core1);

    //----------------------------------------------------------------------
    printf("\n\nCORE INIT END\n");

    //----------------------------------------------------------------------
    //MAIN LOOP
    while (true) {
        //----------------------------------------------------------------------
        //Read the data
        //Data from zero
        mutex_enter_blocking(&zero_mutex);
        uart_zero->readData();
        mutex_exit(&zero_mutex);

        //Data from qmc
        if(!error_qmc5883l && data_ready_qmc5883l){
            if (qmc5883l.readData())
            {
                //printRawDataOnly(qmc5883l.rawDataAxes());
                //plotData(qmc5883l.calibratedDataX(), qmc5883l.calibratedDataY(), qmc5883l.calibratedDataZ());
                //printf("\nAzimuth: %.2f\n", qmc5883l.azimuthZUp());
            }else{
                printf("Error: %d", qmc5883l.lastError());
            }
            data_ready_qmc5883l = false;
        }

        //Data from mpu6050
        if(data_ready_mpu6050){
            if(mpu6050.get_data_accel() || mpu6050.get_data_gyro()){
                printf("MPU6050 ERREUR DURANT LECTURE\n");
            }
            data_ready_mpu6050 = false;
        }

        //Data from gps
        while (uart_is_readable(uart1)) {
            gps.encode(uart_getc(uart1));
        }
        
        //----------------------------------------------------------------------
        //Process and store the data
        local_data.pitch = atan2(mpu6050.accelX_processed, sqrt(mpu6050.accelY_processed * mpu6050.accelY_processed + mpu6050.accelZ_processed * mpu6050.accelZ_processed)) * RAD180;
        local_data.roll = atan2(mpu6050.accelY_processed, sqrt(mpu6050.accelX_processed * mpu6050.accelX_processed + mpu6050.accelZ_processed * mpu6050.accelZ_processed)) * RAD180;
        local_data.yaw = atan2(qmc5883l.calibratedDataY(), qmc5883l.calibratedDataX()) * RAD180;

        if (gps.location.isValid()) {
            local_data.gps_latitude = gps.location.lat();
            local_data.gps_longitude = gps.location.lng();
        } else {
            local_data.gps_latitude = 0.0f;
            local_data.gps_longitude = 0.0f;
        }

        if (gps.altitude.isValid()) {
            local_data.gps_altitude = gps.altitude.meters();
        } else {
            local_data.gps_altitude = 0.0f;
        }

        //----------------------------------------------------------------------
        //Send the data to the other core
        mutex_enter_blocking(&data_mutex);
        memcpy((void*)&shared_data, &local_data, sizeof(SensorData));
        shared_data_ready = true;
        mutex_exit(&data_mutex);
    }
}

// Recois les données du zero, fait des calcul avec les données des sensors et l'envoie aux moteurs
void controlMotors(UART* uart_zero){
    //----------------------------------------------------------------------
    //Variable declaration
    SensorData local_data;

    printf("\n\nCORE INIT END\n");

    //----------------------------------------------------------------------
    //MAIN LOOP
    while (true) {
        //----------------------------------------------------------------------
        //Handle new data from pi zero
        mutex_enter_blocking(&zero_mutex);
        if(uart_zero->isNewDataReceived()){
            std::string data = uart_zero->getReceivedData();
            uart_zero->write(data);
        }
        mutex_exit(&zero_mutex);

        //----------------------------------------------------------------------
        //Read sensor data from the other core
        mutex_enter_blocking(&data_mutex);
        bool data_ready = shared_data_ready;
        mutex_exit(&data_mutex);

        if (data_ready) {
            // Acquire lock, read shared data, and release lock
            mutex_enter_blocking(&data_mutex);
            memcpy(&local_data, (void*)&shared_data, sizeof(SensorData));
            shared_data_ready = false;
            mutex_exit(&data_mutex);



            char buffer[255];
            snprintf(buffer, sizeof(buffer), "Pitch: %.2f, Roll: %.2f, Yaw: %.2f, GPS: (%.4f, %.4f, %.2f)\n",
                     local_data.pitch, local_data.roll, local_data.yaw,
                     local_data.gps_latitude, local_data.gps_longitude, local_data.gps_altitude);

            mutex_enter_blocking(&zero_mutex);
            uart_zero->write(buffer);
            mutex_exit(&zero_mutex);

            printf("Pitch: %.2f, Roll: %.2f, Yaw: %.2f, GPS: (%.4f, %.4f, %.2f)\n",
                   local_data.pitch, local_data.roll, local_data.yaw,
                   local_data.gps_latitude, local_data.gps_longitude, local_data.gps_altitude);
        }

        //----------------------------------------------------------------------
        //Process motor and sensor data together

        //----------------------------------------------------------------------
        //Send data to motor

        //----------------------------------------------------------------------
        //Send a copy of the sensor data to the pi zero
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
    UART uart_zero(uart0, 9600, 1, 0);

    //Commence le core 1
    multicore_launch_core1(readSensorsAndCalculateBasicData);

    //Passe les arguments au core 1
    multicore_fifo_push_blocking(reinterpret_cast<uint32_t>(&uart_zero));
    
    controlMotors(&uart_zero);
}

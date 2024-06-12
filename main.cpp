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

#define PI 3.14159265358979323846
#define RAD180 (180 * PI)

volatile bool data_ready_qmc5883l;
bool error_qmc5883l;
bool new_data_gps;

/*******************************************************************************
 * Function Definitions
 */

void printError(QMC5883L::Error error, char *error_location)
{
   printf("Error: ");
   switch(error)
   {
      case QMC5883L::i2c_buffer_overflow: printf("I2C buffer overflow");          break;
      case QMC5883L::i2c_address_nack:    printf("I2C address not acknowledged"); break;
      case QMC5883L::i2c_data_nack:       printf("I2C data not acknowledged");    break;
      case QMC5883L::i2c_other:           printf("I2C other");                    break;
      case QMC5883L::i2c_request_partial: printf("I2C request partial");          break;
      case QMC5883L::qmc_data_overflow:   printf("QMC5883L data overflow");       break;
   }
   printf("%d\n", error);
   printf(" in ");
   printf(error_location);
}

void printConfig(const QMC5883L::Config& config)
{
   printf(  "  Mode               = "); 
   switch (config.mode)
   {
      case QMC5883L::Mode::standby:    printf("Standby"); break;
      case QMC5883L::Mode::continuous: printf("Continuous"); break;
   }
   printf(  "  Output Data Rate   = ");
   switch (config.output_data_rate)
   {
      case QMC5883L::OutputDataRate::odr_10hz:  printf("10Hz");  break;
      case QMC5883L::OutputDataRate::odr_50hz:  printf("50Hz");  break;
      case QMC5883L::OutputDataRate::odr_100hz: printf("100Hz"); break;
      case QMC5883L::OutputDataRate::odr_200hz: printf("200Hz"); break;
   }
   printf(  "  Full Scale Range   = ");
   switch (config.full_scale_range)
   {
      case QMC5883L::FullScaleRange::rng_2g: printf("2G"); break;
      case QMC5883L::FullScaleRange::rng_8g: printf("5G"); break;
   }
   printf(  "  Over Sample Rate   = ");
   switch (config.over_sample_rate)
   {
      case QMC5883L::OverSampleRate::osr_64:  printf("64");  break;
      case QMC5883L::OverSampleRate::osr_128: printf("128"); break;
      case QMC5883L::OverSampleRate::osr_256: printf("256"); break;
      case QMC5883L::OverSampleRate::osr_512: printf("512"); break;
   }
   printf(  "  Interrupt Disabled = ");
   printf(config.interrupt_disabled ? "True" : "False");  
   printf("\r\n");
}

void printStatus(bool data_ready, bool data_overflow, bool data_skipped)
{
   printf("Status: Data Ready = ");
   printf(data_ready     ? "True " : "False");
   printf(", Data Overflow = ");
   printf(data_overflow  ? "True " : "False");
   printf(", Data Skipped = ");
   printf(data_skipped ? "True " : "False");
}

void plotData(float x, float y, float z)
{
   printf("QMC5883L Data: X = %.5f\n", x);
   printf("QMC5883L Data: Y = %.5f\n", y);
   printf("QMC5883L Data: Z = %.5f\n", z);
   printf("\n");
}

void printRawDataOnly(QMC5883L::RawDataAxes rawDataAxes)
{
   printf("%d\t", rawDataAxes.x);
   printf("%d\t", rawDataAxes.y);
   printf("%d\t", rawDataAxes.z);
   printf("\n");
}

void qmc_5883l_interupt(uint gpio, uint32_t events) {
    data_ready_qmc5883l = true;
}

void uart_gps_test(){
    UART uart_gps(uart1, 9600, 9, 8);

    new_data_gps = false;
    
    // Create a buffer to store received data
    char buffer[256];
    
    while (true) {
        
        //std::string receivedString = uart_gps.readLine();



        // Check if there is data available to read
        /*if (uart_is_readable(UART_ID_GPS)) {
            int index = 0;
            while (uart_is_readable(UART_ID_GPS) && index < sizeof(buffer) - 1) {
                // Read a character from the UART
                buffer[index++] = uart_getc(UART_ID_GPS);
            }
            buffer[index] = '\0'; // Null-terminate the string
            
            // Print the received data
            printf("Received: %s\n", buffer);
        }*/
        
        // Optional: Add a small delay to avoid flooding the output
        sleep_ms(100);
    }
    
    return;
}

void uartListenerTask() {
    UART* uart = reinterpret_cast<UART*>(multicore_fifo_pop_blocking());
    uart->listenForData();
}

/*******************************************************************************
 * Main
 */
int main() {
    //uart_gps_test();

    //Initialise communication with pizero

    UART uart_zero(uart0, 9600, 1, 0);

    //Commence le thread qui listen sur le uart

    multicore_launch_core1(uartListenerTask);

    multicore_fifo_push_blocking(reinterpret_cast<uint32_t>(&uart_zero));

    float rateCalibrationRoll, rateCalibrationPitch, rateCalibrationYaw;

    //SDA = 12, SCL = 13
    I2C i2c = I2C(i2c0, 12, 13, 400*1000);

    i2c.setup();

    //MPU 6050 initialisation
    MPU6050 mpu6050 = MPU6050(&i2c);

    printf("INIT MPU6050\n");
    if(mpu6050.init()){
        printf("MPU6050 ERREUR DURANT INIT\n");
        return 1;
    }
    printf("FIN INIT MPU6050\n");

    printf("CALIBRATING MPU6050...\n");
    if(mpu6050.calibrate()){
        printf("MPU6050 ERREUR DURANT LA CALIBRATION\n");
        return 1;
    }
    printf("CALIBRATING FINISHED MPU6050\n");

    //QMC6883L config
    QMC5883L qmc5883l = QMC5883L(&i2c);
    const char qmc5883l_data_ready_pin = 16;
    volatile bool data_ready;

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

    data_ready_qmc5883l = false;
    
    //Set the interupt
    gpio_init(qmc5883l_data_ready_pin);
    gpio_set_dir(qmc5883l_data_ready_pin, GPIO_IN);
    gpio_pull_down(qmc5883l_data_ready_pin);
    gpio_set_irq_enabled(qmc5883l_data_ready_pin, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled_with_callback(qmc5883l_data_ready_pin, GPIO_IRQ_EDGE_RISE, true, &qmc_5883l_interupt);

    printf("\n\nSTART\n");
    while (true) {
        //Handle pi zero
        if(uart_zero.isNewDataReceived()){
            std::string data = uart_zero.getReceivedData();
            uart_zero.write(data);
        }

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

        // Read X, Y, and Z values from registers (16 bits each)
        if(mpu6050.get_data_accel() || mpu6050.get_data_gyro()){
            printf("MPU6050 ERREUR DURANT LECTURE\n");
        }

        //acc_x = (int16_t)((data[1] << 8) | data[0]);
        //acc_y = (int16_t)((data[3] << 8) | data[2]);
        //acc_z = (int16_t)((data[5] << 8) | data[4]);

        // Convert measurements to [m/s^2]
        //acc_x_f = acc_x * SENSITIVITY_2G * EARTH_GRAVITY;
        //acc_y_f = acc_y * SENSITIVITY_2G * EARTH_GRAVITY;
        //acc_z_f = acc_z * SENSITIVITY_2G * EARTH_GRAVITY;

        // Print results
        //printf("Accel(g) X: %.2f | Y: %.2f | Z: %.2f\r\n", mpu6050.accelX_processed, mpu6050.accelY_processed, mpu6050.accelZ_processed);
        //printf("Gyro (deg) X: %.2f | Y: %.2f | Z: %.2f\r\n", mpu6050.gyroX_processed, mpu6050.gyroY_processed, mpu6050.gyroZ_processed);

        float pitch = atan2(mpu6050.accelX_processed, sqrt(mpu6050.accelY_processed * mpu6050.accelY_processed + mpu6050.accelZ_processed * mpu6050.accelZ_processed)) * RAD180;
        float roll = atan2(mpu6050.accelY_processed, sqrt(mpu6050.accelX_processed * mpu6050.accelX_processed + mpu6050.accelZ_processed * mpu6050.accelZ_processed)) * RAD180;
        float yaw = atan2(qmc5883l.calibratedDataY(), qmc5883l.calibratedDataX()) * RAD180;

        printf("DATA pitch: %.2f | roll: %.2f | yaw: %.2f\r\n", pitch, roll, yaw);

        sleep_ms(100);
        
        //printf("X: %.2f | Y: %.2f | Z: %.2f\r\n", acc_x, acc_y, acc_z);
        //printf("X: %.2f | Y: %.2f | Z: %.2f\r\n", acc_x_f, acc_y_f, acc_z_f);
    }
}

#include <stdio.h>
#include <sstream>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>

#include "pico/stdlib.h"
#include "UART.h"
#include "Drone.h"

Drone* globalDrone;

/*******************************************************************************
 * Function Definitions
 */

void interrupt(uint gpio, uint32_t events) {
    if(gpio == QMC5883L_DATA_READY_PIN){
        globalDrone->setDataReadyQMC5883L();
    }
    else if(gpio == MPU6050_DATA_READY_PIN){
        globalDrone->setDataReadyMPU6050();
    }
}

/*******************************************************************************
 * Main
 */
int main() {
    stdio_init_all();

    Drone drone = Drone();
    globalDrone = &drone;
    drone.init();

    //Set interupts
    //QMC5883l
    gpio_set_irq_enabled_with_callback(QMC5883L_DATA_READY_PIN, GPIO_IRQ_EDGE_RISE, true, &interrupt);

    //MPU6050
    gpio_set_irq_enabled_with_callback(MPU6050_DATA_READY_PIN, GPIO_IRQ_EDGE_RISE, true, &interrupt);

    //----------------------------------------------------------------------
    //MAIN LOOP
    
    int messageCount = 0;
    auto startTime = std::chrono::steady_clock::now();

    while (true) {
        //----------------------------------------------------------------------
        //Read the sensor data

        drone.sensorRead();
        
        //----------------------------------------------------------------------
        //Handle new data from the zero
        std::optional<Message> messageReceivedOpt = drone.receiveMessage();

        if(messageReceivedOpt.has_value()){
            Message messageReceived = messageReceivedOpt.value();
            if(messageReceived.type == MessageType::RequestData){
                switch(messageReceived.data.requestData.requestType) {
                    case RequestType::POSITION_REQUEST:
                        Message messagePosition;
                        messagePosition.type = MessageType::PositionData;
                        messagePosition.data.positionData = drone.getPositionData();

                        drone.sendMessage(messagePosition);
                        break;
                    case RequestType::SENSOR_REQUEST:
                        Message messageSensor;
                        messageSensor.type = MessageType::SensorData;
                        messageSensor.data.sensorData = drone.getSensorData();

                        drone.sendMessage(messageSensor);
                        break;
                    default:
                        break;
                }
            }
        }

        //----------------------------------------------------------------------
        //Handle new data from sensor

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
            drone.log(std::to_string(messageCount));
            messageCount = 0;
            startTime = std::chrono::steady_clock::now();
        }
    }
}

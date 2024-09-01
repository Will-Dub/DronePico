#include <stdio.h>
#include <sstream>
#include <cstdio>
#include <cstring>
#include <cctype>
#include <cmath>
#include <string>
#include <sstream>

#include "pico/stdlib.h"
#include "Drone.h"
#include "pico/binary_info.h"

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

uint8_t msgCount = 0;

/*******************************************************************************
 * Main
 */
int main() {
    stdio_init_all();

    Drone drone = Drone(1);
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

    long lastSendTime = 0;
    int interval = 5000;

    while (true) {
        //----------------------------------------------------------------------
        //Read the sensor data
        drone.sensorRead();
        
        //----------------------------------------------------------------------
        //Handle new data from lora and the zero

        // Lora
        std::optional<DataPacket> dataPacketLoraOpt = drone.receiveDataPacketLora();

        if (dataPacketLoraOpt.has_value()) {
            drone.log("LORA NEW PACKET!!\n");
            std::optional<DataPacket> returnDataPacket = drone.handleDataPacket(dataPacketLoraOpt.value());

            if(returnDataPacket.has_value()){
                drone.SendDataPacketUart(returnDataPacket.value());
            }
        }

        // Zero
        std::optional<DataPacket> dataPacketUartOpt = drone.receiveDataPacketUart();

        if (dataPacketUartOpt.has_value()) {
            std::optional<DataPacket> returnDataPacket = drone.handleDataPacket(dataPacketUartOpt.value());

            if(returnDataPacket.has_value()){
                drone.SendDataPacketUart(returnDataPacket.value());
            }
        }

        //----------------------------------------------------------------------
        //Handle new data from sensor

        //----------------------------------------------------------------------
        //Process motor and sensor data together

        //----------------------------------------------------------------------
        //Send data to motor

        //----------------------------------------------------------------------
        //Test
        messageCount++;

        auto currentTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
        if (elapsedTime >= 5) {
            //drone.log(std::to_string(messageCount));
            messageCount = 0;
            startTime = std::chrono::steady_clock::now();
        }
    }
}

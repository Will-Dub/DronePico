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

// Remove when in prod
#pragma GCC optimize ("O0")

Drone* globalDrone;

const uint LED_PIN = 25;
const uint INIT_BLINK_TIME_MS = 3000;
const uint PACKET_BLINK_TIME_MS = 300;

static uint32_t ledLastToggleTime = 0;
static uint ledLastToggleDuration = 0;

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
    else if(gpio == LORA_DATA_READY_PIN){
        globalDrone->readDataLora();
    }
}

void blinkLed(uint durationMs) {
    ledLastToggleTime = to_ms_since_boot(get_absolute_time());
    ledLastToggleDuration = durationMs;
    gpio_put(LED_PIN, true);
}

void refreshLed() {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // Check to turn off the led
    if((current_time - ledLastToggleTime) >= ledLastToggleDuration){
        gpio_put(LED_PIN, false);
    }
}

/*******************************************************************************
 * Main
 */
int main() {
    stdio_init_all();

    // LED init
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    blinkLed(INIT_BLINK_TIME_MS);

    // Drone init
    Drone drone = Drone(1);
    globalDrone = &drone;
    drone.init();

    //Set interupts
    //QMC5883l
    gpio_set_irq_enabled_with_callback(QMC5883L_DATA_READY_PIN, GPIO_IRQ_EDGE_RISE, true, &interrupt);

    //MPU6050
    gpio_set_irq_enabled_with_callback(MPU6050_DATA_READY_PIN, GPIO_IRQ_EDGE_RISE, true, &interrupt);

    // LORA
    gpio_set_irq_enabled_with_callback(LORA_DATA_READY_PIN, GPIO_IRQ_EDGE_RISE, true, &interrupt);

    //----------------------------------------------------------------------
    //MAIN LOOP
    
    int messageCount = 0;
    auto startTime = std::chrono::steady_clock::now();

    long lastSendTime = 0;
    int interval = 5000;

    bool lastLoraConnectionStatus = false;

    while (true) {
        //----------------------------------------------------------------------
        //Read the sensor data
        drone.sensorRead();
        
        //----------------------------------------------------------------------
        //Handle new data from lora and the zero

        // Lora
        std::optional<DataPacket> dataPacketLoraOpt = drone.receiveDataPacketLora();

        if (dataPacketLoraOpt.has_value()) {
            std::optional<DataPacket> returnDataPacket = drone.handleDataPacket(dataPacketLoraOpt.value());

            blinkLed(PACKET_BLINK_TIME_MS);

            if(returnDataPacket.has_value()){
                drone.SendDataPacketLora(returnDataPacket.value());
            }
        }

        // Zero
        std::optional<DataPacket> dataPacketUartOpt = drone.receiveDataPacketUart();

        if (dataPacketUartOpt.has_value()) {
            std::optional<DataPacket> returnDataPacket = drone.handleDataPacket(dataPacketUartOpt.value());

            blinkLed(PACKET_BLINK_TIME_MS);

            if(returnDataPacket.has_value()){
                drone.SendDataPacketUart(returnDataPacket.value());
            }
        }

        // Change of state handler, lora
        bool loraConnectionStatus = drone.getLoraConnectionStatus();
        if(lastLoraConnectionStatus != loraConnectionStatus){
            // Reset the drone connection
            drone.resetConnection();

            // Set the current state
            lastLoraConnectionStatus = loraConnectionStatus;
        }

        //----------------------------------------------------------------------
        //Handle new data from sensor

        //----------------------------------------------------------------------
        //Process motor and sensor data together

        //----------------------------------------------------------------------
        //Test

        messageCount++;

        auto currentTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
        if (elapsedTime >= 5) {
            drone.log(std::to_string(messageCount));
            startTime = std::chrono::steady_clock::now();
            messageCount = 0;
        }

        refreshLed();
    }
}

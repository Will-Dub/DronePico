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
uint8_t msgCount = 0;

const uint LED_PIN = 25;
const uint BLINK_INTERVAL_MS = 500;

/*******************************************************************************
 * Function Definitions
 */
void interrupt(uint gpio, uint32_t events) {
    gpio_acknowledge_irq(gpio, events);
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

void blink_led(uint pin, uint interval_ms) {
    static uint32_t last_toggle_time = 0;
    static bool led_state = false;

    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    if (current_time - last_toggle_time >= interval_ms) {
        led_state = !led_state;
        gpio_put(pin, led_state);

        last_toggle_time = current_time;
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

            if(returnDataPacket.has_value()){
                drone.SendDataPacketLora(returnDataPacket.value());
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

        blink_led(LED_PIN, BLINK_INTERVAL_MS);
    }
}

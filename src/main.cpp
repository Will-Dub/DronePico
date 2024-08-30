#include <stdio.h>
#include <sstream>
#include <cstdio>
#include <cstring>
#include <cctype>
#include <cmath>
#include <string>
#include <sstream>

#include "pico/stdlib.h"
#include "UART.h"
#include "Drone.h"

#include "pico/binary_info.h"
#include "LoRa-RP2040.h"

Drone* globalDrone;
string receivedData = "";
const size_t MAX_BUFFER_SIZE = 256;

/*******************************************************************************
 * Function Definitions
 */

//--------------------------------------------------------------------+
// Lora Helper
//--------------------------------------------------------------------+
void interrupt(uint gpio, uint32_t events) {
    if(gpio == QMC5883L_DATA_READY_PIN){
        globalDrone->setDataReadyQMC5883L();
    }
    else if(gpio == MPU6050_DATA_READY_PIN){
        globalDrone->setDataReadyMPU6050();
    }
}

uint8_t msgCount = 0;

void sendDataPacket(DataPacket dataPacket) {
    uint8_t buffer[256] = {0};
    size_t packet_size = dataPacket.serialize(buffer, sizeof(buffer));
    LoRa.beginPacket();
    LoRa.write(buffer, packet_size);
    LoRa.endPacket();
    return;
}

std::optional<DataPacket> getReceivedDataPacket() {
    LoRa.parsePacket();
    while (LoRa.available()) {
        receivedData += (char)LoRa.read();
    }
    while (receivedData.size() >= 10) {
        // Find the start marker
        auto start_it = std::find(receivedData.begin(), receivedData.end(), DataPacket::START_MARKER);
        if (start_it == receivedData.end()) {
            // No start marker found, clear all data if incomplete message
            receivedData.clear();
            return std::nullopt;
        }

        // Calculate the remaining data after the start marker
        size_t remaining_data = std::distance(start_it, receivedData.end());
        if (remaining_data < 12) {  // Minimum size check
            return std::nullopt;
        }

        // Extract the message length
        uint32_t message_length;
        memcpy(&message_length, &*(start_it + 7), sizeof(uint32_t));

        //Verify message length is in the range
        if (message_length > MAX_BUFFER_SIZE) {
            // Message size exceeds buffer limit, discard all data
            auto next_start_it = std::find(start_it + 1, receivedData.end(), DataPacket::START_MARKER);
            receivedData.erase(receivedData.begin(), next_start_it);
            return std::nullopt;
        }

        // Ensure we have the complete message
        size_t total_message_size = 12 + message_length;
        if (remaining_data < total_message_size) {
            return std::nullopt;
        }

        // Check the end marker
        auto end_it = start_it + total_message_size - 1;
        if (*end_it != DataPacket::END_MARKER) {
            // Invalid end marker, discard data up to next start marker
            auto next_start_it = std::find(start_it + 1, receivedData.end(), DataPacket::START_MARKER);
            if (next_start_it != receivedData.end()) {
                receivedData.erase(receivedData.begin(), next_start_it); // Discard up to next start marker
            } else {
                receivedData.clear(); // No more start marker found, clear all data
            }
            return std::nullopt;
        }

        // Extract and deserialize the message
        std::vector<uint8_t> buffer(start_it, end_it + 1);
        DataPacket dataPacket;
        if (dataPacket.deserialize(buffer.data(), buffer.size())) {
            receivedData.erase(receivedData.begin(), end_it + 1); // Remove the processed message including the end marker
            return dataPacket;
        } else {
            receivedData.erase(receivedData.begin(), start_it + 1); // Move past the invalid start marker
        }
    }

    return std::nullopt;
}



/*******************************************************************************
 * Main
 */
int main() {
    stdio_init_all();

    if (!LoRa.begin(433.425E6)) {
        printf("LoRa init failed. Check your connections.\n");
        while (true);
    }

    printf("LoRa init succeeded.\n");

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

    long lastSendTime = 0;
    int interval = 5000;

    while (true) {
        // parse for a packet, and call onReceive with the result:
        std::optional<DataPacket> dataPacketLoraOpt = getReceivedDataPacket();
        if(dataPacketLoraOpt.has_value()){
            DataPacket dataPacket = dataPacketLoraOpt.value();
            printf("NEW PACKET!!\n");
        }

        //----------------------------------------------------------------------
        //Read the sensor data
        drone.sensorRead();
        
        //----------------------------------------------------------------------
        //Handle new data from the zero
        std::optional<DataPacket> dataPacketUartOpt = drone.receiveDataPacketUart();

        if (dataPacketUartOpt.has_value()) {
            DataPacket dataPacketUart = dataPacketUartOpt.value();
            // TODO: drone id validation

            switch (dataPacketUart.type) {
                case DataType::GPS: {
                    // Get the position data
                    PositionData positionData = drone.getPositionData();

                    // Format the data
                    std::stringstream ss;
                    ss << positionData.gpsLatitude << ";"
                    << positionData.gpsLongitude << ";"
                    << positionData.gpsAltitude << ";"
                    << positionData.gpsKmph << ";"
                    << positionData.gpsCourseDeg;

                    std::string combinedString = ss.str();
                    std::vector<uint8_t> byteVector(combinedString.begin(), combinedString.end());

                    // Make the packet
                    DataPacket dataPacketReturn(1, 2, DataType::GPS, byteVector);

                    // Send the data
                    drone.SendDataPacketUart(dataPacketReturn);
                    break;
                }
                default:
                    break;
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
            drone.log(std::to_string(messageCount));
            messageCount = 0;
            startTime = std::chrono::steady_clock::now();
        }
    }
}

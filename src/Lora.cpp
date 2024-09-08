#include "Lora.h"

Lora::Lora(const long frequency)
    : FREQUENCY(frequency),
    isNewDataReceived(false){}

bool Lora::init(){
    isLoraInitialized = LoRa.begin(FREQUENCY);
    return isLoraInitialized;
}

void Lora::writeDataPacket(DataPacket dataPacket){
    if(!isLoraInitialized){
        return;
    }

    uint8_t buffer[MAX_SERIALISED_PACKET_SIZE] = {0};
    size_t packet_size = dataPacket.serialize(buffer, sizeof(buffer));
    LoRa.beginPacket();
    LoRa.write(buffer, packet_size);
    sleep_ms(5);
    LoRa.endPacket();

    return;
}

void Lora::readData(){
    if(!isLoraInitialized){
        return;
    }

    LoRa.parsePacket();

    if(!LoRa.available()){
        return;
    }

    isNewDataReceived = true;

    while (LoRa.available() && receivedData.size() < MAX_RECV_BUFFER_SIZE) {
        receivedData += (char)LoRa.read();
    }

    return;
}

std::optional<DataPacket> Lora::getReceivedDataPacket(){
    if(!isLoraInitialized){
        return std::nullopt;
    }
    
    isNewDataReceived = false;
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
        if (remaining_data < 12) {
            return std::nullopt;
        }

        // Extract the message length
        uint32_t message_length;
        memcpy(&message_length, &*(start_it + 7), sizeof(uint32_t));

        //Verify message length is in the range
        if (message_length > MAX_DATA_SIZE) {
            // Message size exceeds buffer limit, discard all
            auto next_start_it = std::find(start_it + 1, receivedData.end(), DataPacket::START_MARKER);
            receivedData.erase(receivedData.begin(), next_start_it);
            return std::nullopt;
        }

        // Ensure the message is in full
        size_t total_message_size = 12 + message_length;
        if (remaining_data < total_message_size) {
            return std::nullopt;
        }

        // Check end marker
        auto end_it = start_it + total_message_size - 1;
        if (*end_it != DataPacket::END_MARKER) {
            // Invalid end marker, discard data up to next start marker
            auto next_start_it = std::find(start_it + 1, receivedData.end(), DataPacket::START_MARKER);
            if (next_start_it != receivedData.end()) {
                receivedData.erase(receivedData.begin(), next_start_it);
            } else {
                receivedData.clear();
            }
            return std::nullopt;
        }

        // Extract and deserialize
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

bool Lora::getIsNewDataReceived(){
    return isNewDataReceived;
}

uint64_t Lora::getLastReceiveTime() {
    return to_us_since_boot(lastReceiveTime);
}
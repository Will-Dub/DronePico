#include "Lora.h"

Lora::Lora(const long frequency)
    : FREQUENCY(frequency),
    isNewDataReceived(false),
    isDataLeftToProcess(false){}

bool Lora::init(){
    isLoraInitialized = LoRa.begin(FREQUENCY);

    // Register the callback
    if(isLoraInitialized){
        LoRa.receive();
    }

    return isLoraInitialized;
}

void Lora::writeDataPacket(DataPacket dataPacket){
    if(!isLoraInitialized){
        return;
    }

    // Wait for last packet
    while (LoRa.beginPacket() == 0) {
      sleep_ms(10);
    }

    uint8_t buffer[MAX_SERIALISED_PACKET_SIZE] = {0};
    size_t packet_size = dataPacket.serialize(buffer, sizeof(buffer));
    LoRa.beginPacket();
    LoRa.write(buffer, packet_size);
    // True for non blocking
    LoRa.endPacket(true);
    LoRa.receive();
    return;
}

void Lora::readData(){
    if(!isLoraInitialized){
        return;
    }

    while (LoRa.available() && receivedData.size() < MAX_RECV_BUFFER_SIZE) {
        receivedData += (char)LoRa.read();
    }

    isNewDataReceived = true;

    return;
}

void Lora::recvInterrupt(){
    LoRa.handleDio0Rise();
    readData();
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
            // No start marker found, clear all data if incomplete dataPacket
            isDataLeftToProcess = false;
            receivedData.clear();
            return std::nullopt;
        }

        // Calculate the remaining data after the start marker
        size_t remaining_data = std::distance(start_it, receivedData.end());
        if (remaining_data < 12) {
            isDataLeftToProcess = false;
            return std::nullopt;
        }

        // Extract the data length
        uint32_t dataLength;
        memcpy(&dataLength, &*(start_it + 7), sizeof(uint32_t));

        //Verify data length is in the range
        if (dataLength > MAX_DATA_SIZE) {
            // data size exceeds buffer limit, discard all
            auto next_start_it = std::find(start_it + 1, receivedData.end(), DataPacket::START_MARKER);
            receivedData.erase(receivedData.begin(), next_start_it);
            isDataLeftToProcess = true;
            return std::nullopt;
        }

        // Ensure the packet is in full
        size_t totalDataPacketSize = 12 + dataLength;
        if (remaining_data < totalDataPacketSize) {
            isDataLeftToProcess = false;
            return std::nullopt;
        }

        // Check end marker
        auto end_it = start_it + totalDataPacketSize - 1;
        if (*end_it != DataPacket::END_MARKER) {
            // Invalid end marker, discard data up to next start marker
            auto next_start_it = std::find(start_it + 1, receivedData.end(), DataPacket::START_MARKER);
            if (next_start_it != receivedData.end()) {
                isDataLeftToProcess = true;
                receivedData.erase(receivedData.begin(), next_start_it);
            } else {
                isDataLeftToProcess = false;
                receivedData.clear();
            }
            return std::nullopt;
        }

        // Extract and deserialize
        std::vector<uint8_t> buffer(start_it, end_it + 1);
        DataPacket dataPacket;
        isDataLeftToProcess = true;
        if (dataPacket.deserialize(buffer.data(), buffer.size())) {
            receivedData.erase(receivedData.begin(), end_it + 1); // Remove the processed data packet including the end marker
            lastReceiveTime = get_absolute_time();
            return dataPacket;
        } else {
            receivedData.erase(receivedData.begin(), start_it + 1); // Move past the invalid start marker
        }
    }
    isDataLeftToProcess = false;

    return std::nullopt;
}

bool Lora::getIsNewDataToProcess(){
    return isNewDataReceived || isDataLeftToProcess;
}

uint64_t Lora::getLastReceiveTime() {
    return to_us_since_boot(lastReceiveTime);
}
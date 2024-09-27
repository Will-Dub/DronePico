#include "Lora.h"

Lora::Lora(const long frequency)
    : FREQUENCY(frequency),
    isNewDataReceived(false),
    isDataLeftToProcess(false){}

void Lora::onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header uint8_ts:
  uint8_t recipient = LoRa.read();          // recipient address
  uint8_t sender = LoRa.read();            // sender address
  uint8_t incomingMsgId = LoRa.read();     // incoming msg ID
  uint8_t incomingLength = LoRa.read();    // incoming msg length

  string incoming;                 // payload of packet

  while (LoRa.available()) {            // can't use readString() in callback, so
    incoming += (char)LoRa.read();      // add uint8_ts one by one
  }

  if (incomingLength != incoming.length()) {   // check length for error
    printf("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != 0x33 && recipient != 0xFF) {
    printf("This message is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  printf("Received from: 0x%x\n", sender);
  printf("Sent to: 0x%x\n", recipient);
  printf("Message ID: %d\n", incomingMsgId);
  printf("Message length: %d\n", incomingLength);
  printf("Message: %s\n", incoming.c_str());
  printf("RSSI: %s\n", LoRa.packetRssi());
  printf("Snr: %s\n", LoRa.packetSnr());
  printf("\n");
}

bool Lora::init(){
    isLoraInitialized = LoRa.begin(FREQUENCY);

    // Register the callback
    if(isLoraInitialized){
        LoRa.onReceive(onReceive);
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
      printf("Waiting for lora ... \n");
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
            printf("TOO LONG");
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
            printf("No end");
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
            printf("Error deserialize");
            receivedData.erase(receivedData.begin(), start_it + 1); // Move past the invalid start marker
        }
    }

    return std::nullopt;
}

bool Lora::getIsNewDataToProcess(){
    return isNewDataReceived || isDataLeftToProcess;
}

uint64_t Lora::getLastReceiveTime() {
    return to_us_since_boot(lastReceiveTime);
}
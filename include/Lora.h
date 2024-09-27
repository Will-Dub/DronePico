#ifndef LORA_PICO_H
#define LORA_PICO_H

#include <stdio.h>
#include "pico/stdlib.h"
#include <string>
#include "LoRa-RP2040.h"
#include "DataPacket.h"
#include <optional>
#include <algorithm>

class Lora
{
   public:
      static constexpr size_t MAX_RECV_BUFFER_SIZE = 1024;
      static constexpr size_t MAX_DATA_SIZE = 256;
      static constexpr size_t MAX_PACKET_SIZE = MAX_DATA_SIZE + DataPacket::HEADER_SIZE;
      static constexpr size_t MAX_SERIALISED_PACKET_SIZE = MAX_PACKET_SIZE + 2;

      Lora(const long frequency);

      bool init();

      void writeDataPacket(DataPacket dataPacket);

      void readData();

      std::optional<DataPacket> getReceivedDataPacket();

      bool getIsNewDataToProcess();

      uint64_t getLastReceiveTime();

      static void onReceive(int packetSize);

   private:
      const long FREQUENCY;
      const size_t MAX_BUFFER_SIZE = 256;
      std::string receivedData = "";
      bool isLoraInitialized = false;
      bool isNewDataReceived;
      bool isDataLeftToProcess;
      absolute_time_t lastReceiveTime;
};

#endif
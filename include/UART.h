#ifndef UART_H
#define UART_H

#include <stdio.h>
#include <string>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include <atomic>
#include <vector>
#include <optional>
#include <algorithm>
#include "DataPacket.h"

//Interface pour uart
class UART
{
    public:
        /**
         * Initialise le uart avec l'instance, le rate et les pins
         */
        UART(uart_inst_t *uart, uint baudrate, int rx_pin, int tx_pin);

        void flush();

        std::string getReceivedData();

        std::vector<std::string> getReceivedLines();

        std::optional<DataPacket> getReceivedDataPacket();

        uint64_t getLastReceiveTime();

        bool getIsNewDataReceived();

        void readData();

        void writeLine(const std::string& str);

        void write(const std::string& str);

        void writeBlock(const uint8_t* data, uint size);

        void writeDataPacket(const DataPacket &data_packet);

    private:
        absolute_time_t lastReceiveTime;
        uart_inst_t *instance;
        uint baudrate;
        const int RX_PIN;
        const int TX_PIN;
        bool isNewDataReceived;
        std::string receivedData;
        const size_t MAX_BUFFER_SIZE = 256;
};

#endif
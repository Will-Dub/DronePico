#include "UART.h"

UART::UART(uart_inst_t *uart_p, uint baudrate_p, uint rxPin_p, uint txPin_p):
    instance(uart_p),
    baudrate(baudrate_p),
    rxPin(rxPin_p),
    txPin(txPin_p),
    isNewDataReceived(false)
    {
        uart_init(instance, baudrate);
        gpio_set_function(txPin, GPIO_FUNC_UART);
        gpio_set_function(rxPin, GPIO_FUNC_UART);

        uart_set_format(instance, 8, 1, UART_PARITY_NONE);

        uart_set_hw_flow(instance, false, false);
        uart_set_fifo_enabled(instance, true);
    }

void UART::writeLine(const std::string& str) {
    for (char c : str) {
        uart_putc(instance, c);
    }
    uart_putc(instance, '\n');
}

void UART::write(const std::string& str) {
    for (char c : str) {
        uart_putc(instance, c);
    }
}

void UART::writeDataPacket(const DataPacket &data_packet){
    uint8_t buffer[256] = {0};
    size_t packet_size = data_packet.serialize(buffer, sizeof(buffer));
    
    uart_write_blocking(instance, buffer, packet_size);
}

void UART::writeBlock(const uint8_t* data, uint size) {
    uart_write_blocking(instance, data, size);
}

std::string UART::getReceivedData() {
    std::string receivedDataReturn = "";

    receivedDataReturn = receivedData;

    isNewDataReceived = false;
    receivedData.clear();

    return receivedDataReturn;
}

std::vector<std::string> UART::getReceivedLines() {
    std::vector<std::string> lines;
    
    size_t pos = 0;
    while ((pos = receivedData.find('\n')) != std::string::npos) {
        lines.push_back(receivedData.substr(0, pos));
        receivedData.erase(0, pos + 1);
    }
    
    isNewDataReceived = !receivedData.empty();
    return lines;
}

std::optional<DataPacket> UART::getReceivedDataPacket() {
    while (receivedData.size() >= 10) {
        // Find the start marker
        auto start_it = std::find(receivedData.begin(), receivedData.end(), DataPacket::START_MARKER);
        if (start_it == receivedData.end()) {
            // No start marker found, clear all data if incomplete data packet
            receivedData.clear();
            return std::nullopt;
        }

        // Calculate the remaining data after the start marker
        size_t remaining_data = std::distance(start_it, receivedData.end());
        if (remaining_data < 12) {  // Minimum size check
            return std::nullopt;
        }

        // Extract the data length
        uint32_t dataLength;
        memcpy(&dataLength, &*(start_it + 7), sizeof(uint32_t));

        //Verify data length is in the range
        if (dataLength > MAX_BUFFER_SIZE) {
            // data size exceeds buffer limit, discard all data
            auto next_start_it = std::find(start_it + 1, receivedData.end(), DataPacket::START_MARKER);
            receivedData.erase(receivedData.begin(), next_start_it);
            return std::nullopt;
        }

        // Ensure we have the complete data packet
        size_t totalDataPacketSize = 12 + dataLength;
        if (remaining_data < totalDataPacketSize) {
            return std::nullopt;
        }

        // Check the end marker
        auto end_it = start_it + totalDataPacketSize - 1;
        if (*end_it != DataPacket::END_MARKER) {
            // Invalid end
            auto next_start_it = std::find(start_it + 1, receivedData.end(), DataPacket::START_MARKER);
            if (next_start_it != receivedData.end()) {
                receivedData.erase(receivedData.begin(), next_start_it); // Remove all to next start
            } else {
                receivedData.clear(); // No start marker found, clear all data
            }
            return std::nullopt;
        }

        // Extract and deserialize the data packet
        std::vector<uint8_t> buffer(start_it, end_it + 1);
        DataPacket dataPacket;
        if (dataPacket.deserialize(buffer.data(), buffer.size())) {
            receivedData.erase(receivedData.begin(), end_it + 1); // Remove the data packet and the end marker
            return dataPacket;
        } else {
            receivedData.erase(receivedData.begin(), start_it + 1); // Remove the invalid start until next start
        }
    }

    return std::nullopt;
}

void UART::readData() {
    if(uart_is_readable(instance)){
        while(uart_is_readable(instance)) {
            uint8_t byte;
            uart_read_blocking(instance, &byte, 1);
            receivedData.push_back(byte);
        }
        lastReceiveTime = get_absolute_time();
        isNewDataReceived = true;
    }
}

void UART::flush() {
    while (uart_is_writable(instance) == 0);
}

bool UART::getIsNewDataReceived() {
    return isNewDataReceived;
}

uint64_t UART::getLastReceiveTime() {
    return to_us_since_boot(lastReceiveTime);
}
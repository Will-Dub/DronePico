#include "UART.h"

UART::UART(uart_inst_t *uart_p, uint baudrate_p, uint rx_pin_p, uint tx_pin_p):
    instance(uart_p),
    baudrate(baudrate_p),
    rx_pin(rx_pin_p),
    tx_pin(tx_pin_p),
    new_data_received(false)
    {
        uart_init(instance, baudrate);
        gpio_set_function(tx_pin, GPIO_FUNC_UART);
        gpio_set_function(rx_pin, GPIO_FUNC_UART);

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

void UART::writeMessage(const Message &message) {
    uint8_t buffer[256] = {0};
    size_t message_size = message.serialize(buffer, sizeof(buffer));
    
    uart_write_blocking(instance, buffer, message_size);
}

void UART::writeBlock(const uint8_t* data, uint size) {
    uart_write_blocking(instance, data, size);
}

std::optional<Message> UART::getReceiveMessage() {
    const size_t MAX_BUFFER_SIZE = 2000; // Maximum size of received_data buffer

    while (received_data.size() >= 6) {
        // Find the start marker
        auto start_it = std::find(received_data.begin(), received_data.end(), Message::START_MARKER);
        if (start_it == received_data.end()) {
            // No start marker found, clear all data if incomplete message
            received_data.clear();
            return std::nullopt;
        }

        // Calculate the remaining data after the start marker
        size_t remaining_data = std::distance(start_it, received_data.end());
        if (remaining_data < 6) {  // Minimum size check
            return std::nullopt;
        }

        // Extract the message length
        uint16_t message_length;
        memcpy(&message_length, &*(start_it + 1), sizeof(uint16_t));

        //Verify message length is in the range
        if (message_length > MAX_BUFFER_SIZE) {
            // Message size exceeds buffer limit, discard all data
            auto next_start_it = std::find(start_it + 1, received_data.end(), Message::START_MARKER);
            received_data.erase(received_data.begin(), next_start_it);
            return std::nullopt;
        }

        // Ensure we have the complete message
        size_t total_message_size = 6 + message_length;
        if (remaining_data < total_message_size) {
            return std::nullopt;
        }

        // Check the end marker
        auto end_it = start_it + total_message_size - 1; // Adjust for inclusive end marker check
        if (*end_it != Message::END_MARKER) {
            // Invalid end marker, discard data up to next start marker
            auto next_start_it = std::find(start_it + 1, received_data.end(), Message::START_MARKER);
            if (next_start_it != received_data.end()) {
                received_data.erase(received_data.begin(), next_start_it); // Discard up to next start marker
            } else {
                received_data.clear(); // No more start marker found, clear all data
            }
            return std::nullopt;
        }

        // Extract and deserialize the message
        std::vector<uint8_t> buffer(start_it, end_it + 1);
        Message message;
        if (message.deserialize(buffer.data(), buffer.size())) {
            received_data.erase(received_data.begin(), end_it + 1); // Remove the processed message including the end marker
            return message;
        } else {
            received_data.erase(received_data.begin(), start_it + 1); // Move past the invalid start marker
        }
    }

    return std::nullopt;
}

void UART::readData() {
    if(uart_is_readable(instance)){
        while(uart_is_readable(instance)) {
            uint8_t byte;
            uart_read_blocking(instance, &byte, 1);
            received_data.push_back(byte);
        }
        last_receive_time = get_absolute_time();
        new_data_received = true;
    }
}

void UART::flush() {
    while (uart_is_writable(instance) == 0);
}

bool UART::isNewDataReceived() {
    return new_data_received;
}

uint64_t UART::get_last_receive_time() {
    return to_us_since_boot(last_receive_time);
}
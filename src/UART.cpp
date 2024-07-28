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

std::string UART::getReceivedData() {
    std::string received_data_return = "";

    received_data_return = received_data;

    new_data_received = false;
    received_data.clear();

    return received_data_return;
}

std::vector<std::string> UART::getReceivedLines() {
    std::vector<std::string> lines;
    
    size_t pos = 0;
    while ((pos = received_data.find('\n')) != std::string::npos) {
        lines.push_back(received_data.substr(0, pos));
        received_data.erase(0, pos + 1);
    }
    
    new_data_received = !received_data.empty();
    return lines;
}

std::optional<Message> UART::getReceiveMessage() {
    if (received_data.size() >= 2) {
        uint16_t message_length;
        memcpy(&message_length, received_data.data(), sizeof(uint16_t));

        if (received_data.size() >= sizeof(uint16_t) + message_length) {
            std::vector<uint8_t> buffer(received_data.begin(), received_data.begin() + sizeof(uint16_t) + message_length);
            Message message;
            message.deserialize(buffer.data(), buffer.size());

            // Erase processed bytes
            received_data.erase(received_data.begin(), received_data.begin() + sizeof(uint16_t) + message_length);

            return message;
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

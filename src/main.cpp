#include <stdio.h>
#include <sstream>
#include <cstdio>
#include <cstring>
#include <cctype>
#include <cmath>
#include <string>

#include "pico/stdlib.h"
#include "UART.h"
#include "Drone.h"

#include "pico/binary_info.h"
#include "LoRa-RP2040.h"

Drone* globalDrone;

/*******************************************************************************
 * Function Definitions
 */

void interrupt(uint gpio, uint32_t events) {
    if(gpio == QMC5883L_DATA_READY_PIN){
        globalDrone->setDataReadyQMC5883L();
    }
    else if(gpio == MPU6050_DATA_READY_PIN){
        globalDrone->setDataReadyMPU6050();
    }
}










uint8_t msgCount = 0;

void sendMessage(string outgoing) {
  int n = outgoing.length();
  char send[n+1];
  strcpy(send,outgoing.c_str());
  printf("Sending: %s\n",send);
  LoRa.beginPacket();                   // start packet
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(5);        // add payload length
  LoRa.write((uint8_t*)send, sizeof(send));
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
  printf("-----------------SENT-----------------\n");
  printf("Message ID: %d\n", msgCount);
  printf("Message length: %d\n", sizeof(send)+1);
  printf("Message: %s\n", send);
  printf("----------------------------------\n");
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return
  // read packet header uint8_ts:
  uint8_t incomingMsgId = LoRa.read();     // incoming msg ID
  uint8_t incomingLength = LoRa.read();    // incoming msg length

  string incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }
  printf("-----------------RECEIVED-----------------\n");
  if (incomingLength != incoming.length()) {   // check length for error
    printf("ERROR: message length does not match length\n");
  }

  // if message is for this device, or broadcast, print details:
  printf("Message ID: %d\n", incomingMsgId);
  printf("Supposed Message length: %d\n", incomingLength);
  printf("Message length: %d\n", incoming.length());
  printf("Message: %s\n", incoming.c_str());
  globalDrone->log("Received: " + incoming);
  printf("\n");
  //printf("RSSI: %d\n", LoRa.packetRssi());
  //printf("Snr: %d\n", LoRa.packetSnr());
  printf("----------------------------------\n");
}











/*******************************************************************************
 * Main
 */
int main() {
    stdio_init_all();

    printf("\nLoRa Duplex\n");

    // override the default CS, reset, and IRQ pins (optional)
    // LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

    /*if (!LoRa.begin(433.425E6)) {             // initialize ratio at 915 MHz
        printf("LoRa init failed. Check your connections.\n");
        while (true);                       // if failed, do nothing
    }*/

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
    int interval = 2000;

    Esc esc = Esc(4);

    while (true) {
        esc.init();
        esc.setSpeedUs(1200);
        sleep_ms(100000);
        /*if (to_ms_since_boot(get_absolute_time()) - lastSendTime > interval) {
            char message[] = "HeLoRa World!";   // send a message
            sendMessage(message);
            lastSendTime = to_ms_since_boot(get_absolute_time());            // timestamp the message
            interval = (rand()%2000) + 1000;    // 2-3 seconds
        }
        // parse for a packet, and call onReceive with the result:
        onReceive(LoRa.parsePacket());

        //----------------------------------------------------------------------
        //Read the sensor data

        drone.sensorRead();
        
        //----------------------------------------------------------------------
        //Handle new data from the zero
        std::optional<Message> messageReceivedOpt = drone.receiveMessage();

        if(messageReceivedOpt.has_value()){
            Message messageReceived = messageReceivedOpt.value();
            if(messageReceived.type == MessageType::RequestData){
                switch(messageReceived.data.requestData.requestType) {
                    case RequestType::POSITION_REQUEST:
                        Message messagePosition;
                        messagePosition.type = MessageType::PositionData;
                        messagePosition.data.positionData = drone.getPositionData();

                        drone.sendMessage(messagePosition);
                        break;
                    case RequestType::SENSOR_REQUEST:
                        Message messageSensor;
                        messageSensor.type = MessageType::SensorData;
                        messageSensor.data.sensorData = drone.getSensorData();

                        drone.sendMessage(messageSensor);
                        break;
                    default:
                        break;
                }
            }
        }

        //----------------------------------------------------------------------
        //Handle new data from sensor

        messageCount++;*/

        //----------------------------------------------------------------------
        //Process motor and sensor data together

        //----------------------------------------------------------------------
        //Send data to motor

        //----------------------------------------------------------------------
        //Send a copy of the sensor data to the pi zero

        auto currentTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
        if (elapsedTime >= 5) {
            std::string input = "hfhfhfhfhf";
            std::vector<uint8_t> dataVector(input.begin(), input.end());
            uint8_t droneId = 1;
            uint32_t packetId = 123;
            DataType type = DataType::TEST;

            DataPacket dataPacket(droneId, packetId, type, dataVector);
            drone.sendDataPacket(dataPacket);
            drone.log(std::to_string(messageCount));
            messageCount = 0;
            startTime = std::chrono::steady_clock::now();
        }
    }
}

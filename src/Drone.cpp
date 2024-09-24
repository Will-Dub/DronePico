#include "Drone.h"

Drone::Drone(uint droneId)
    : i2c(i2c1, SDA_PIN_I2C, SCL_PIN_I2C, 100*1000),
    mpu6050(&i2c),
    qmc5883l(&i2c),
    uartGps(uart1, 9600, RX_PIN_GPS, TX_PIN_GPS),
    gps(),
    uartZero(uart0, 230400, RX_PIN_ZERO, TX_PIN_ZERO),
    lora(LORA_MHZ),
    motorController(MOTOR_1_PIN, MOTOR_2_PIN, MOTOR_3_PIN, MOTOR_4_PIN),
    DRONE_ID(droneId),
    nextPacketId(1){}

void Drone::init(){
    PositionData position_data = {};
    SensorData sensor_data = {};

    i2c.setup();

    log("INIT MPU6050");
    if(mpu6050.init()){
        mpu6050Error = true;
        log("MPU6050 ERREUR DURANT INIT", LogType::LOG_CRITICAL);
    }
    log("FIN INIT MPU6050");

    log("CALIBRATING MPU6050...");
    if(mpu6050.calibrate()){
        mpu6050Error = true;
        log("MPU6050 ERREUR DURANT LA CALIBRATION", LogType::LOG_CRITICAL);
    }
    log("CALIBRATING FINISHED MPU6050");

    //QMC6883L initialisation + config
    log("CONFIG DEBUT QMC5883L");
    if (!qmc5883l.begin())
    {
        qmc5883lError = true;
    }
    else
    {
        qmc5883l.setConfig({ QMC5883L::Mode::continuous,
                            QMC5883L::OutputDataRate::odr_10hz,
                            QMC5883L::FullScaleRange::rng_8g,
                            QMC5883L::OverSampleRate::osr_512,
                            false                              });

        if (!qmc5883l.writeConfig())
        {
            qmc5883lError = true;
        }
        else
        {
            qmc5883l.setCalibration({ {-281.852962, 
                                        -27.985588,
                                        557.383102 },
                                   {{   0.805536,   -0.000364,  -0.011148},
                                    {  -0.000364,   0.819217,   -0.090000},
                                    {  -0.011148,    -0.090000,   0.785799} }});

            qmc5883lError = false;
        }
    }
    log("CONFIG FINI QMC5883L");

    log("CONFIG DEBUT LORA");

    lora.init();

    log("CONFIG FINI LORA");
}

void Drone::sensorRead(){
    if(statusData.useQmc5883l && dataReadyQmc5883l){
        if (!qmc5883l.readData())
        {
            log("QMC error: " + qmc5883l.lastError(), LogType::LOG_ERROR);
        }else{
            sensorData.magX = qmc5883l.calibratedDataX();
            sensorData.magY = qmc5883l.calibratedDataY();
            sensorData.magZ = qmc5883l.calibratedDataZ();
            newSensorData = true;
        }
        dataReadyQmc5883l = false;
    }

    //Data from mpu6050
    if(statusData.useMpu6050 && dataReadyMpu6050){
        if(mpu6050.getDataAccel() || mpu6050.getDataGyro()){
            log("MPU6050 ERREUR DURANT LECTURE", LogType::LOG_ERROR);
        }else{
            sensorData.accelX = mpu6050.accelXProcessed;
            sensorData.accelY = mpu6050.accelYProcessed;
            sensorData.accelZ = mpu6050.accelZProcessed;
            sensorData.gyroX = mpu6050.gyroXProcessed;
            sensorData.gyroY = mpu6050.gyroYProcessed;
            sensorData.gyroZ = mpu6050.gyroZProcessed;
            newSensorData = true;
        }
        dataReadyMpu6050 = false;
    }

    //Data from gps
    if(statusData.useGps){
        uartZero.readData();
    
        if(uartZero.getIsNewDataToProcess()){
            std::string gpsData = uartZero.getReceivedData();
            for (char c : gpsData) {
                gps.encode(c);
            }
            newPositionData = true;
        }
    }
}

void Drone::SendDataPacketLora(DataPacket dataPacket){
    lora.writeDataPacket(dataPacket);
}

void Drone::SendDataPacketUart(DataPacket dataPacket){
    uartZero.writeDataPacket(dataPacket);
}

std::optional<DataPacket> Drone::receiveDataPacketLora(){
    lora.readData();
    
    if(lora.getIsNewDataToProcess()){
        return lora.getReceivedDataPacket();
    }

    return std::nullopt;
}

std::optional<DataPacket> Drone::receiveDataPacketUart(){
    uartZero.readData();
    
    if(uartZero.getIsNewDataToProcess()){
        return uartZero.getReceivedDataPacket();
    }

    return std::nullopt;
}

void Drone::setUseMpu6050(bool enable){
    statusData.useMpu6050 = enable;
}

void Drone::setUseQmc5883l(bool enable){
    statusData.useQmc5883l = enable;
}

void Drone::setUseGps(bool enable){
    statusData.useGps = enable;
}

void Drone::setUseLog(bool enable){
    statusData.useLog = enable;
}

void Drone::setDataReadyQMC5883L(){
    dataReadyMpu6050 = true;
}

void Drone::setDataReadyMPU6050(){
    dataReadyQmc5883l = true;
}

bool Drone::isNewPositionData(){
    return newPositionData;
}

bool Drone::isNewSensorData(){
    return newSensorData;
}

StatusData Drone::getStatusData(){
    statusData.uartGpsConnected = isDataReceivedWithinTimeout(uartGps.getLastReceiveTime());
    statusData.i2cConnected = isDataReceivedWithinTimeout(i2c.getLastReceiveTime());
    statusData.uartZeroConnected = isDataReceivedWithinTimeout(uartZero.getLastReceiveTime());
    statusData.loraConnected = isDataReceivedWithinTimeout(lora.getLastReceiveTime());
    statusData.useMotor = motorController.getIsInit();
    return statusData;
}

PositionData Drone::getPositionData(){
    if(newPositionData){
        positionData.gpsLatitude = gps.location.lat();
        positionData.gpsLongitude = gps.location.lng();
        positionData.gpsAltitude = gps.altitude.meters();
        positionData.gpsKmph = gps.speed.kmph();
        positionData.gpsCourseDeg = gps.course.deg();
        newPositionData = false;
    }
    
    return positionData;
}

SensorData Drone::getSensorData(){
    sensorData.pitch = atan2(sensorData.accelX, sqrt(sensorData.accelY * sensorData.accelY + sensorData.accelZ * sensorData.accelZ)) * RAD180;
    sensorData.roll = atan2(sensorData.accelY, sqrt(sensorData.accelX * sensorData.accelX + sensorData.accelZ * sensorData.accelZ)) * RAD180;
    sensorData.yaw = atan2(sensorData.magY, sensorData.magX) * RAD180;
    newSensorData = false;
    return sensorData;
}

uint Drone::getDroneId(){
    return DRONE_ID;
}

void Drone::log(const std::string& data, LogType logType){
    if(statusData.useLog == false){
        return;
    }

    // Create the data vector with the log type
    std::string formattedStr = std::to_string(static_cast<int>(logType)) + ";";
    std::vector<uint8_t> dataVec(data.begin(), data.end());
    dataVec.insert(dataVec.begin(), formattedStr.begin(), formattedStr.end());

    // Initialise the packet
    DataPacket logPacket = DataPacket(DRONE_ID, 1, DataType::LOG, dataVec);

    // Send the packet
    uartZero.writeDataPacket(logPacket);
    lora.writeDataPacket(logPacket);

    return;
}

bool Drone::isDataReceivedWithinTimeout(uint64_t lastReceivedTime){
    uint64_t currentMs = to_us_since_boot(get_absolute_time()) / 1000;
    uint64_t lastMs = lastReceivedTime / 1000;

    return (currentMs - lastMs <= TIMEOUT);
}

void Drone::motorInit(){
    motorController.init();
    statusData.useMotor = true;
    return;
}

void Drone::motorUninit(){
    motorController.uninit();
    statusData.useMotor = false;
    return;
}

//TODO find a place for this
bool isValidInteger(const std::string& str) {
    if (str.empty()) return false;
    size_t start = 0;
    
    // Handle optional negative sign
    if (str[0] == '-') {
        if (str.size() == 1) return false; // "-" is not a valid integer
        start = 1;
    }
    
    // Check if all remaining characters are digits
    for (size_t i = start; i < str.size(); ++i) {
        if (!std::isdigit(str[i])) return false;
    }
    
    return true;
}

std::vector<int> splitAndConvertToInts(const std::string& str, char delimiter) {
    std::vector<int> values;
    std::stringstream ss(str);
    std::string item;

    while (std::getline(ss, item, delimiter)) {
        if (isValidInteger(item)) {
            // Convert string to integer
            std::istringstream(item) >> std::ws; // Skip leading whitespaces
            int value;
            std::istringstream(item) >> value;
            values.push_back(value);
        } else {
            std::cerr << "Error: Invalid integer format: " << item << std::endl;
        }
    }

    return values;
}

std::optional<DataPacket> Drone::handleDataPacket(DataPacket receivedDataPacket){
    //Verify for drone id
    if(receivedDataPacket.droneId != DRONE_ID){
        return std::nullopt;
    }

    // Check if it's a start packet
    if(receivedDataPacket.type == DataType::START){
        nextPacketId = 0;
        return receivedDataPacket;
    }

    // Data already processed. Packet with id of 0 are always handled
    if(receivedDataPacket.packetId != 0){
        if(receivedDataPacket.packetId < nextPacketId){
            // Packet already processed
            return std::nullopt;
        }
        nextPacketId = receivedDataPacket.packetId += 1;
    }

    // Handle the packet
    switch (receivedDataPacket.type) {
        case DataType::CONTROL: {
            std::string dataStr(receivedDataPacket.data.begin(), receivedDataPacket.data.end());

            std::vector<int> controlValues = splitAndConvertToInts(dataStr, ';');

            if (controlValues.size() == 4) {
                motorController.control(controlValues[0], controlValues[1], controlValues[2], controlValues[3]);
            }
        }
        case DataType::GPS: {
            // Get the position data
            PositionData positionData = getPositionData();

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
            DataPacket dataPacketReturn(DRONE_ID, receivedDataPacket.packetId, DataType::GPS, byteVector);
            return dataPacketReturn;
        }
        case DataType::SENSOR: {
            // Get the sensor data
            SensorData sensorData = getSensorData();

            // Format the data
            std::stringstream ss;
            ss << sensorData.accelX << ";"
            << sensorData.accelY << ";"
            << sensorData.accelZ << ";"
            << sensorData.gyroX << ";"
            << sensorData.gyroY << ";"
            << sensorData.gyroZ << ";"
            << sensorData.magX << ";"
            << sensorData.magY << ";"
            << sensorData.magZ << ";"
            << sensorData.pitch << ";"
            << sensorData.roll << ";"
            << sensorData.yaw;

            std::string combinedString = ss.str();
            std::vector<uint8_t> byteVector(combinedString.begin(), combinedString.end());

            // Make the packet
            DataPacket dataPacketReturn(DRONE_ID, receivedDataPacket.packetId, DataType::SENSOR, byteVector);
            return dataPacketReturn;
        }
        case DataType::STATUS: {
            // Get the status data
            StatusData statusData = getStatusData();

            // Format the data
            std::stringstream ss;
            ss << statusData.uartZeroConnected << ";"
            << statusData.uartGpsConnected << ";"
            << statusData.i2cConnected << ";"
            << statusData.loraConnected << ";"
            << statusData.useMotor << ";"
            << statusData.useMpu6050 << ";"
            << statusData.useQmc5883l << ";"
            << statusData.useGps << ";"
            << statusData.useLog << ";";

            std::string combinedString = ss.str();
            std::vector<uint8_t> byteVector(combinedString.begin(), combinedString.end());

            // Make the packet
            DataPacket dataPacketReturn(DRONE_ID, receivedDataPacket.packetId, DataType::STATUS, byteVector);
            return dataPacketReturn;
        }
        case DataType::START_SPECIFIC: {
            // Start a specific part of the drone
            std::string dataStr(receivedDataPacket.data.begin(), receivedDataPacket.data.end());
            
            if(dataStr == "MOTOR"){
                motorInit();
            }else if(dataStr == "MPU6050"){
                setUseMpu6050(true);
            }else if(dataStr == "QMC5883L"){
                setUseQmc5883l(true);
            }else if(dataStr == "GPS"){
                setUseGps(true);
            }else if(dataStr == "LOG"){
                setUseLog(true);
            }

            //TODO remove that
            // Get the status data
            StatusData statusData = getStatusData();

            // Format the data
            std::stringstream ss;
            ss << statusData.uartZeroConnected << ";"
            << statusData.uartGpsConnected << ";"
            << statusData.i2cConnected << ";"
            << statusData.loraConnected << ";"
            << statusData.useMotor << ";"
            << statusData.useMpu6050 << ";"
            << statusData.useQmc5883l << ";"
            << statusData.useGps << ";"
            << statusData.useLog << ";";

            std::string combinedString = ss.str();
            std::vector<uint8_t> byteVector(combinedString.begin(), combinedString.end());

            // Make the packet
            DataPacket dataPacketReturn(DRONE_ID, receivedDataPacket.packetId, DataType::STATUS, byteVector);
            return dataPacketReturn;
        }
        case DataType::STOP_SPECIFIC: {
            // Stop a specific part of the drone
            std::string dataStr(receivedDataPacket.data.begin(), receivedDataPacket.data.end());
            
            if(dataStr == "MOTOR"){
                motorUninit();
            }else if(dataStr == "MPU6050"){
                setUseMpu6050(false);
            }else if(dataStr == "QMC5883L"){
                setUseQmc5883l(false);
            }else if(dataStr == "GPS"){
                setUseGps(false);
            }else if(dataStr == "LOG"){
                setUseLog(false);
            }
        }
        default:
            break;
    }

    return std::nullopt;
}

#include "Drone.h"

Drone::Drone()
    : i2c(i2c1, SDA_PIN_I2C, SCL_PIN_I2C, 100*1000),
    mpu6050(&i2c),
    qmc5883l(&i2c),
    uartGps(uart1, 9600, RX_PIN_GPS, TX_PIN_GPS),
    gps(),
    uartZero(uart0, 230400, RX_PIN_ZERO, TX_PIN_ZERO),
    motor1(MOTOR1_PIN),
    motor2(MOTOR2_PIN),
    motor3(MOTOR3_PIN),
    motor4(MOTOR4_PIN){}

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
    while (statusData.useGps && uart_is_readable(uart1)) {
        char c = uart_getc(uart1);
        gps.encode(c);
        if (gps.location.isUpdated()) {
            positionData.gpsLatitude = gps.location.lat();
            positionData.gpsLongitude = gps.location.lng();
            positionData.gpsAltitude = gps.altitude.meters();
            positionData.gpsKmph = gps.speed.kmph();
            positionData.gpsCourseDeg = gps.course.deg();
            newPositionData = true;
        }
    }
}

void Drone::SendDataPacketUart(DataPacket dataPacket){
    uartZero.writeDataPacket(dataPacket);
}

std::optional<DataPacket> Drone::receiveDataPacketUart(){
    uartZero.readData();
    
    if(uartZero.isNewDataReceived()){
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
    return statusData;
}

PositionData Drone::getPositionData(){
    newPositionData = false;
    return positionData;
}

SensorData Drone::getSensorData(){
    sensorData.pitch = atan2(sensorData.accelX, sqrt(sensorData.accelY * sensorData.accelY + sensorData.accelZ * sensorData.accelZ)) * RAD180;
    sensorData.roll = atan2(sensorData.accelY, sqrt(sensorData.accelX * sensorData.accelX + sensorData.accelZ * sensorData.accelZ)) * RAD180;
    sensorData.yaw = atan2(sensorData.magY, sensorData.magX) * RAD180;
    newSensorData = false;
    return sensorData;
}

void Drone::log(const std::string& data, LogType dataType){
    Message message;
    message.type = MessageType::LogData;
    message.data.logData.type = dataType;

    std::strncpy(message.data.logData.message, data.c_str(), sizeof(message.data.logData.message) - 1);
    message.data.logData.message[sizeof(message.data.logData.message) - 1] = '\0';

    uartZero.writeMessage(message);
    return;
}

bool Drone::isDataReceivedWithinTimeout(uint64_t lastReceivedTime){
    uint64_t currentMs = to_us_since_boot(get_absolute_time()) / 1000;
    uint64_t lastMs = lastReceivedTime / 1000;

    return (currentMs - lastMs <= timeout);
}

void Drone::motorInit(){
    motor1.init();
    motor2.init();
    motor3.init();
    motor4.init();
    return;
}
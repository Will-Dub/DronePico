#include "Drone.h"

Drone::Drone()
    : i2c(i2c1, SdaPin_I2C, SclPin_I2C, 100*1000),
    mpu6050(&i2c),
    qmc5883l(&i2c),
    uartGps(uart1, 9600, RXPin_GPS, TXPin_GPS),
    gps(),
    uartZero(uart0, 230400, RXPin_ZERO, TXPin_ZERO){}

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
    if(statusData.use_qmc5883l && dataReadyQmc5883l){
        if (!qmc5883l.readData())
        {
            log("QMC error: " + qmc5883l.lastError(), LogType::LOG_ERROR);
        }else{
            sensorData.mag_x = qmc5883l.calibratedDataX();
            sensorData.mag_y = qmc5883l.calibratedDataY();
            sensorData.mag_z = qmc5883l.calibratedDataZ();
            newSensorData = true;
        }
        dataReadyQmc5883l = false;
    }

    //Data from mpu6050
    if(statusData.use_mpu6050 && dataReadyMpu6050){
        if(mpu6050.get_data_accel() || mpu6050.get_data_gyro()){
            log("MPU6050 ERREUR DURANT LECTURE", LogType::LOG_ERROR);
        }else{
            sensorData.accel_x = mpu6050.accelX_processed;
            sensorData.accel_y = mpu6050.accelY_processed;
            sensorData.accel_z = mpu6050.accelZ_processed;
            sensorData.gyro_x = mpu6050.gyroX_processed;
            sensorData.gyro_y = mpu6050.gyroY_processed;
            sensorData.gyro_z = mpu6050.gyroZ_processed;
            newSensorData = true;
        }
        dataReadyMpu6050 = false;
    }

    //Data from gps
    while (statusData.use_gps && uart_is_readable(uart1)) {
        char c = uart_getc(uart1);
        gps.encode(c);
        if (gps.location.isUpdated()) {
            positionData.gps_latitude = gps.location.lat();
            positionData.gps_longitude = gps.location.lng();
            positionData.gps_altitude = gps.altitude.meters();
            positionData.gps_kmph = gps.speed.kmph();
            positionData.gps_course_deg = gps.course.deg();
            newPositionData = true;
        }
    }
}

void Drone::sendMessage(Message message){
    uartZero.writeMessage(message);
}

std::optional<Message> Drone::receiveMessage(){
    uartZero.readData();
    return uartZero.getReceiveMessage();
}

void Drone::setUseMpu6050(bool enable){
    statusData.use_mpu6050 = enable;
}

void Drone::setUseQmc5883l(bool enable){
    statusData.use_qmc5883l = enable;
}

void Drone::setUseGps(bool enable){
    statusData.use_gps = enable;
}

void Drone::setUseLog(bool enable){
    statusData.use_log = enable;
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
    statusData.uart_gps_connected = isDataReceivedWithinTimeout(uartGps.get_last_receive_time());
    statusData.i2c_connected = isDataReceivedWithinTimeout(i2c.get_last_receive_time());
    statusData.uart_zero_connected = isDataReceivedWithinTimeout(uartZero.get_last_receive_time());
    return statusData;
}

PositionData Drone::getPositionData(){
    newPositionData = false;
    return positionData;
}

SensorData Drone::getSensorData(){
    sensorData.pitch = atan2(sensorData.accel_x, sqrt(sensorData.accel_y * sensorData.accel_y + sensorData.accel_z * sensorData.accel_z)) * RAD180;
    sensorData.roll = atan2(sensorData.accel_y, sqrt(sensorData.accel_x * sensorData.accel_x + sensorData.accel_z * sensorData.accel_z)) * RAD180;
    sensorData.yaw = atan2(sensorData.mag_y, sensorData.mag_x) * RAD180;
    newSensorData = false;
    return sensorData;
}

void Drone::log(const std::string& data, LogType dataType){
    Message message;
    message.type = MessageType::LogData;
    message.data.log_data.type = dataType;

    std::strncpy(message.data.log_data.message, data.c_str(), sizeof(message.data.log_data.message) - 1);
    message.data.log_data.message[sizeof(message.data.log_data.message) - 1] = '\0';

    uartZero.writeMessage(message);
    return;
}

bool Drone::isDataReceivedWithinTimeout(uint64_t lastReceivedTime){
    uint64_t current_ms = to_us_since_boot(get_absolute_time()) / 1000;
    uint64_t last_ms = lastReceivedTime / 1000;

    return (current_ms - last_ms <= timeout);
}
#include <device_MPU6050.h>
#include <util/delay.h>

// bool MPU6050::readAllSensors(MPU6050_Data &data) {
//     uint8_t buffer[14];  // We'll read all sensor data at once (14 bytes total)
    
//     // Start reading from ACCEL_XOUT_H register (0x3B)
//     if (!start(DEVICE_ADDRESS, I2C::Write)) return false;
//     if (!write(ACCEL_XOUT_H)) return false;
//     if (!restart(DEVICE_ADDRESS, I2C::Read)) return false;
    
//     // Read all 14 bytes in sequence
//     for (uint8_t i = 0; i < 14; i++) {
//         buffer[i] = read(i < 13);  // Send NACK on last byte
//     }
//     stop();
    
//     // Combine high and low bytes into 16-bit values
//     // Accelerometer data (bytes 0-5)
//     data.accelerometer[0] = (buffer[0] << 8) | buffer[1];  // X-axis
//     data.accelerometer[1] = (buffer[2] << 8) | buffer[3];  // Y-axis
//     data.accelerometer[2] = (buffer[4] << 8) | buffer[5];  // Z-axis
    
//     // Temperature data (bytes 6-7)
//     data.temperature = (buffer[6] << 8) | buffer[7];
    
//     // Gyroscope data (bytes 8-13)
//     data.gyroscope[0] = (buffer[8] << 8) | buffer[9];    // X-axis
//     data.gyroscope[1] = (buffer[10] << 8) | buffer[11];  // Y-axis
//     data.gyroscope[2] = (buffer[12] << 8) | buffer[13];  // Z-axis
    
//     return true;
// }

MPU6050::MPU6050(volatile uint8_t *sda_pin_reg, volatile uint8_t *sda_ddr, volatile uint8_t *sda_port, uint8_t sda_pin,
                 volatile uint8_t *scl_pin_reg, volatile uint8_t *scl_ddr, volatile uint8_t *scl_port, uint8_t scl_pin) : 
                 I2C(sda_pin_reg, sda_ddr, sda_port, sda_pin,
                 scl_pin_reg, scl_ddr, scl_port, scl_pin) {
                
}


bool MPU6050::wakeUp() {
    // Write 0 to PWR_MGMT_1 to wake up the device
    uint8_t data[2] = {PWR_MGMT_1, 0x00};
    return writeMessage(DEVICE_ADDRESS, data, 2);
}

bool MPU6050::setRegisterPointer(uint8_t reg) {
    startCondition();
    
    bool success = writeByte((DEVICE_ADDRESS << 1) | 0); // Write mode
    success = success && writeByte(reg);

    return success;
}

bool MPU6050::initialize() {
    // Wake up the MPU6050
    bool success = wakeUp();

    // read accelerometer sensitivity
    success = success && setRegisterPointer(ACCEL_CONFIG);
    uint8_t accel_config;
    success = success && readMessage(DEVICE_ADDRESS, &accel_config, 1);
    accel_range = (accel_config >> 3) & 0x03; // extract



    // convert to sensitivity
    switch (accel_range) {
        case 0: accel_sensitivity = 16384.0; break; // ±2g
        case 1: accel_sensitivity = 8192.0;  break; // ±4g
        case 2: accel_sensitivity = 4096.0;  break; // ±8g
        case 3: accel_sensitivity = 2048.0;  break; // ±16g
        default: accel_sensitivity = 16384.0; break; // default to ±2g
    }

    // read gyroscope range
    success = success && setRegisterPointer(GYRO_CONFIG);
    uint8_t gyro_config;
    success = success && readMessage(DEVICE_ADDRESS, &gyro_config, 1);
    gyro_range = (gyro_config >> 3) & 0x03; // extract

    // convert to sensitivity
    switch (gyro_range) {
        case 0: gyro_sensitivity = 131.0;   break; // ±250°/s
        case 1: gyro_sensitivity = 65.5;    break; // ±500°/s
        case 2: gyro_sensitivity = 32.8;    break; // ±1000°/s
        case 3: gyro_sensitivity = 16.4;    break; // ±2000°/s
        default: gyro_sensitivity = 131.0;   break; // default to ±250°/s
    }

    return success;
}

bool MPU6050::readAllSensors(MPU6050::MPU6050_Data &data) {
    setRegisterPointer(ACCEL_XOUT_H);

    const uint8_t length = 14;
    uint8_t buffer[length];

    if (!readMessage(DEVICE_ADDRESS, buffer, length)) {
        return false;
    }

    int16_t accelerometer[3]; // X, Y, Z
    int16_t gyroscope[3];     // X, Y, Z
    int16_t temperature;

    // Combine high and low bytes into 16-bit values
    // Accelerometer data (bytes 0-5)
    accelerometer[0] = (buffer[0] << 8) | buffer[1];  // X-axis
    accelerometer[1] = (buffer[2] << 8) | buffer[3];  // Y-axis
    accelerometer[2] = (buffer[4] << 8) | buffer[5];  // Z-axis
    // Temperature data (bytes 6-7)
    temperature = (buffer[6] << 8) | buffer[7];
    // Gyroscope data (bytes 8-13)
    gyroscope[0] = (buffer[8] << 8) | buffer[9];    // X-axis
    gyroscope[1] = (buffer[10] << 8) | buffer[11];  // Y-axis
    gyroscope[2] = (buffer[12] << 8) | buffer[13];  // Z-axis

    // Convert to physical units
    data.accel_x = static_cast<float>(accelerometer[0]) / accel_sensitivity;
    data.accel_y = static_cast<float>(accelerometer[1]) / accel_sensitivity;
    data.accel_z = static_cast<float>(accelerometer[2]) / accel_sensitivity;

    data.gyro_x = static_cast<float>(gyroscope[0]) / gyro_sensitivity;
    data.gyro_y = static_cast<float>(gyroscope[1]) / gyro_sensitivity;
    data.gyro_z = static_cast<float>(gyroscope[2]) / gyro_sensitivity;

    data.temperature = (static_cast<float>(temperature) / 340.0) + 36.53;

    return true;
}

bool MPU6050::setAccelRange(int range) { // 0=±2g,1=±4g,2=±8g,3=±16g
    if (range < 0 || range > 3) {
        return false; // Invalid range
    }

    // Read the current ACCEL_CONFIG value
    setRegisterPointer(ACCEL_CONFIG);
    uint8_t accel_config;
    if (!readMessage(DEVICE_ADDRESS, &accel_config, 1)) {
        return false;
    }

    // Set the new range
    accel_config = (accel_config & 0xE7) | (range << 3);
    uint8_t data[2] = {ACCEL_CONFIG, accel_config};
    if (!writeMessage(DEVICE_ADDRESS, data, 2)) {
        return false;
    }

    // Update local variables
    accel_range = range;
    switch (accel_range) {
        case 0: accel_sensitivity = 16384.0; break; // ±2g
        case 1: accel_sensitivity = 8192.0;  break; // ±4g
        case 2: accel_sensitivity = 4096.0;  break; // ±8g
        case 3: accel_sensitivity = 2048.0;  break; // ±16g
    }

    return true;
}

bool MPU6050::setGyroRange(int range) { // 0=±250°/s,1=±500°/s,2=±1000°/s,3=±2000°/s
    if (range < 0 || range > 3) {
        return false; // Invalid range
    }

    // Read the current GYRO_CONFIG value
    setRegisterPointer(GYRO_CONFIG);
    uint8_t gyro_config;
    if (!readMessage(DEVICE_ADDRESS, &gyro_config, 1)) {
        return false;
    }

    // Set the new range
    gyro_config = (gyro_config & 0xE7) | (range << 3);
    uint8_t data[2] = {GYRO_CONFIG, gyro_config};
    if (!writeMessage(DEVICE_ADDRESS, data, 2)) {
        return false;
    }

    // Update local variables
    gyro_range = range;
    switch (gyro_range) {
        case 0: gyro_sensitivity = 131.0; break;  // ±250°/s
        case 1: gyro_sensitivity = 65.5; break;   // ±500°/s
        case 2: gyro_sensitivity = 32.8; break;   // ±1000°/s
        case 3: gyro_sensitivity = 16.4; break;   // ±2000°/s
    }

    return true;
}

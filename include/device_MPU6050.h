#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <protocol_I2C.h>

// namespace MPU6050_ns {
//     typedef struct {
//         int16_t accelerometer[3]; // X, Y, Z
//         int16_t gyroscope[3];     // X, Y, Z
//         int16_t temperature;
//     } MPU6050_Data;
// }


class MPU6050 : protected I2C { 
    
public:
    typedef struct {
        float accel_x;
        float accel_y;
        float accel_z;

        float gyro_x;
        float gyro_y;
        float gyro_z;

        float temperature;
    } MPU6050_Data;

    MPU6050(volatile uint8_t *sda_pin_reg, volatile uint8_t *sda_ddr, volatile uint8_t *sda_port, uint8_t sda_pin,
            volatile uint8_t *scl_pin_reg, volatile uint8_t *scl_ddr, volatile uint8_t *scl_port, uint8_t scl_pin);

    MPU6050() = delete;

    bool readAccelerometer(int16_t &ax, int16_t &ay, int16_t &az);
    bool readGyroscope(int16_t &gx, int16_t &gy, int16_t &gz);
    bool readTemperature(int16_t &temp);
    bool readAllSensors(MPU6050::MPU6050_Data &data);
    bool initialize();

    int getAccelRange() const { return accel_range; }
    int getGyroRange() const { return gyro_range; }

    bool setAccelRange(int range);
    bool setGyroRange(int range);

private:
    // config variables
    float accel_sensitivity = 0; 
    float gyro_sensitivity = 0;  

    int accel_range = 0;
    int gyro_range = 0;
    
    
    // Reading 14 bytes starting from ACCEL_XOUT_H (0x3B) will get all sensor data:
    // [0-5]:   Accelerometer (X,Y,Z)
    // [6-7]:   Temperature
    // [8-13]:  Gyroscope (X,Y,Z


    // Accelerometer X-axis high byte register address
    static const auto ACCEL_XOUT_H = 0x3B;
    // Gyroscope X-axis high byte register address
    static const auto GYRO_XOUT_H = 0x43;
    // Accelerometer Y-axis high byte register address
    static const auto ACCEL_YOUT_H = 0x3D;
    // Gyroscope Y-axis high byte register address
    static const auto GYRO_YOUT_H = 0x45;
    // Accelerometer Z-axis high byte register address
    static const auto ACCEL_ZOUT_H = 0x3F;
    // Gyroscope Z-axis high byte register address
    static const auto GYRO_ZOUT_H = 0x47;
    // Accelerometer X-axis low byte register address
    static const auto ACCEL_XOUT_L = 0x3C;
    // Gyroscope X-axis low byte register address
    static const auto GYRO_XOUT_L = 0x44;
    // Accelerometer Y-axis low byte register address
    static const auto ACCEL_YOUT_L = 0x3E;
    // Gyroscope Y-axis low byte register address
    static const auto GYRO_YOUT_L = 0x46;
    // Accelerometer Z-axis low byte register address
    static const auto ACCEL_ZOUTL = 0x40;
    // Gyroscope Z-axis low byte register address
    static const auto GYRO_ZOUT_L = 0x48;
    // Temperature sensor high byte register address
    static const auto TEMP_OUT_H = 0x41;  
    // Temperature sensor low byte register address
    static const auto TEMP_OUT_L = 0x42;
    // Device identification register address (should return 0x68)
    static const auto WHO_AM_I = 0x75;
    // MPU6050 I2C device address (AD0 pin low)
    static const auto DEVICE_ADDRESS = 0x68;
    // Power management register 1 - controls device power mode and clock source
    static const auto PWR_MGMT_1 = 0x6B;
    // Power management register 2 - controls individual power state of accelerometer and gyroscope axes
    static const auto PWR_MGMT_2 = 0x6C;
    // Accelerometer configuration register - sets full scale range
    static const auto ACCEL_CONFIG = 0x1C;
    // Gyroscope configuration register - sets full scale range
    static const auto GYRO_CONFIG = 0x1B; 
    // Sample rate divider register - sets sample rate
    static const auto SMPLRT_DIV = 0x19;
    // Configuration register - sets external synchronization and digital low pass filter
    static const auto CONFIG = 0x1A;

    bool setRegisterPointer(uint8_t reg);
    bool wakeUp();
};

#endif // MPU6050_H
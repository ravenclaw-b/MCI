#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <avr/io.h>

class I2C {
public:
    // Preferred constructor: pass PINx, DDRx, PORTx pointers explicitly
    I2C(volatile uint8_t *sda_pin_reg, volatile uint8_t *sda_ddr, volatile uint8_t *sda_port, uint8_t sda_pin,
        volatile uint8_t *scl_pin_reg, volatile uint8_t *scl_ddr, volatile uint8_t *scl_port, uint8_t scl_pin);

    // Delay config (microseconds)
    void setDelay(int microseconds);

    // High-level I2C operations
    bool writeMessage(uint8_t address, const uint8_t *data, unsigned int length);
    bool readMessage(uint8_t address, uint8_t *data, unsigned int length);

private:
    // Registers for SDA
    volatile uint8_t *SDA_PIN_REG;
    volatile uint8_t *SDA_DDR;
    volatile uint8_t *SDA_PORT;
    uint8_t SDA_PIN;

    // Registers for SCL
    volatile uint8_t *SCL_PIN_REG;
    volatile uint8_t *SCL_DDR;
    volatile uint8_t *SCL_PORT;
    uint8_t SCL_PIN;

    // Configurable delay
    int I2C_DELAY_US = 5;

    // Low-level helpers
    void pull_scl_low();
    void release_scl();
    void pull_sda_low();
    void release_sda();

    bool read_scl();
    bool read_sda();

    inline void delay();

protected:
    bool writeByte(uint8_t data);
    bool readByte(uint8_t &data, bool ack);

    void startCondition();
    void stopCondition();
};

#endif // I2C_H

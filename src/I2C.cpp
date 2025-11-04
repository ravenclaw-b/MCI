#include "I2C.h"

#include <util/delay.h>

// ---------- Constructors ----------
// Preferred explicit constructor (PIN register provided)
I2C::I2C(volatile uint8_t *sda_pin_reg, volatile uint8_t *sda_ddr, volatile uint8_t *sda_port, uint8_t sda_pin,
         volatile uint8_t *scl_pin_reg, volatile uint8_t *scl_ddr, volatile uint8_t *scl_port, uint8_t scl_pin)
    : SDA_PIN_REG(sda_pin_reg), SDA_DDR(sda_ddr), SDA_PORT(sda_port), SDA_PIN(sda_pin),
      SCL_PIN_REG(scl_pin_reg), SCL_DDR(scl_ddr), SCL_PORT(scl_port), SCL_PIN(scl_pin) 
{
    // Ensure pins are released (inputs) and enable pull-ups
    (*SDA_DDR) &= ~(1 << SDA_PIN); // input
    (*SDA_PORT) |= (1 << SDA_PIN); // enable pull-up
    (*SCL_DDR) &= ~(1 << SCL_PIN); // input
    (*SCL_PORT) |= (1 << SCL_PIN); // enable pull-up
}


// ---------- Public API ----------
void I2C::setDelay(int microseconds) {
    I2C_DELAY_US = microseconds;
}

bool I2C::writeMessage(uint8_t address, const uint8_t *data, unsigned int length) {
    startCondition();

    if (!writeByte(address << 1)) { // Write mode
        stopCondition();
        return false; // No ACK from slave
    }
    for (unsigned int i = 0; i < length; i++) {
        if (!writeByte(data[i])) {
            stopCondition();
            return false; // No ACK from slave
        }
    }
    stopCondition();
    return true;
}

bool I2C::readMessage(uint8_t address, uint8_t *data, unsigned int length) {
    startCondition();

    if (!writeByte((address << 1) | 0x01)) { // Read mode
        stopCondition();
        return false; // No ACK from slave
    }
    for (unsigned int i = 0; i < length; i++) {
        bool ack = (i < length - 1); // ACK all but last byte
        if (!readByte(data[i], ack)) {
            stopCondition();
            return false; // Read error
        }
    }
    stopCondition();
    return true;
}

// ---------- Low-level pin control ----------
void I2C::pull_scl_low() {
    (*SCL_DDR) |= (1 << SCL_PIN); // Set SCL as output
    (*SCL_PORT) &= ~(1 << SCL_PIN); // Drive SCL low
}

void I2C::release_scl() {
    (*SCL_DDR) &= ~(1 << SCL_PIN); // Set SCL as input
    (*SCL_PORT) |= (1 << SCL_PIN); // Enable pull-up resistor on SCL
}

void I2C::pull_sda_low() {
    (*SDA_DDR) |= (1 << SDA_PIN); // Set SDA as output
    (*SDA_PORT) &= ~(1 << SDA_PIN); // Drive SDA low
}

void I2C::release_sda() {
    (*SDA_DDR) &= ~(1 << SDA_PIN); // Set SDA as input
    (*SDA_PORT) |= (1 << SDA_PIN); // Enable pull-up resistor on SDA
}

// Corrected read functions — read from PINx register and mask bit
bool I2C::read_scl() {
    return ( (*SCL_PIN_REG) & (1 << SCL_PIN) ) != 0;
}

bool I2C::read_sda() {
    return ( (*SDA_PIN_REG) & (1 << SDA_PIN) ) != 0;
}

inline void I2C::delay() {
    for (int i = 0; i < I2C_DELAY_US; ++i) {
        _delay_us(1);
    }
}

// ---------- Byte-level protocols ----------
bool I2C::writeByte(uint8_t data) {
    for (int i = 0; i < 8; i++)
    {
        pull_scl_low(); // Clock low
        delay();

        if (data & 0x80) {
            release_sda(); // Send 1
        } else {
            pull_sda_low(); // Send 0
        }
        delay();

        release_scl(); // Clock high
        uint32_t timeout = 10000;
        while (!read_scl() && --timeout) { }
        delay();

        data <<= 1;
    }

    // ACK/NACK bit
    pull_scl_low();
    release_sda(); // Release SDA for ACK/NACK
    delay();

    release_scl(); // Clock high
    uint32_t timeout = 10000;
    while (!read_scl() && --timeout) { }
    delay();

    bool ack = !read_sda(); // ACK is low (0), NACK is high (1)
    pull_scl_low();

    return ack;
}

bool I2C::readByte(uint8_t &data, bool ack) {
    data = 0;
    for (int i = 0; i < 8; i++) {
        data <<= 1;

        pull_scl_low();
        release_sda(); // Release SDA for reading
        delay();

        release_scl(); // Clock high
        uint32_t timeout = 10000;
        while (!read_scl() && --timeout) { }
        delay();

        if (read_sda()) {
            data |= 0x01; // Read bit
        }
    }

    // Send ACK/NACK bit
    pull_scl_low();
    if (ack) {
        pull_sda_low(); // Send ACK (0)
    } else {
        release_sda(); // Send NACK (1)
    }
    delay();

    release_scl(); // Clock high
    uint32_t timeout = 10000;
    while (!read_scl() && --timeout) { }
    delay();

    pull_scl_low();
    release_sda(); // Release SDA

    return true;
}

// ---------- Conditions ----------
void I2C::startCondition() {
    release_sda();
    release_scl();
    delay();

    pull_sda_low();
    delay();
    pull_scl_low();
    delay();
}

void I2C::stopCondition() {
    pull_sda_low();
    delay();
    release_scl();
    uint32_t timeout = 10000;
    while (!read_scl() && --timeout) { }
    delay();

    release_sda();
    delay();
}

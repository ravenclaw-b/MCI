#ifndef PROTOCOL_SPI_H
#define PROTOCOL_SPI_H

#include <stdint.h>
#include <stddef.h>
#include <avr/io.h>

class SPI {
public:
    enum class BitOrder : uint8_t {
        MSBFirst = 0,
        LSBFirst = 1
    };

    enum class ClockPolarity : uint8_t {
        IdleLow = 0,
        IdleHigh = 1
    };

    enum class ClockPhase : uint8_t {
        SampleLeadingEdge = 0,
        SampleTrailingEdge = 1
    };

    SPI(volatile uint8_t *mosi_pin_reg, volatile uint8_t *mosi_ddr, volatile uint8_t *mosi_port, uint8_t mosi_pin,
        volatile uint8_t *miso_pin_reg, volatile uint8_t *miso_ddr, volatile uint8_t *miso_port, uint8_t miso_pin,
        volatile uint8_t *sck_pin_reg, volatile uint8_t *sck_ddr, volatile uint8_t *sck_port, uint8_t sck_pin,
        volatile uint8_t *ss_pin_reg, volatile uint8_t *ss_ddr, volatile uint8_t *ss_port, uint8_t ss_pin);

    SPI() = delete;

    void begin(bool autoChipSelect = true);

    void setBitOrder(BitOrder order);
    void setDataMode(uint8_t mode);
    void setDataMode(ClockPolarity polarity, ClockPhase phase);
    void setDelaysMicroseconds(double lowPhase, double highPhase);
    void setClockHz(uint32_t frequencyHz);
    void setChipSelectPolarity(bool activeLow);
    void setAutoChipSelect(bool enable);

    void select();
    void deselect();

    uint8_t transferByte(uint8_t data);
    void transferBytes(const uint8_t *tx, uint8_t *rx, size_t length);
    void writeBytes(const uint8_t *data, size_t length);

private:
    volatile uint8_t *MOSI_PIN_REG;
    volatile uint8_t *MOSI_DDR;
    volatile uint8_t *MOSI_PORT;
    uint8_t MOSI_PIN;
    uint8_t MOSI_MASK;

    volatile uint8_t *MISO_PIN_REG;
    volatile uint8_t *MISO_DDR;
    volatile uint8_t *MISO_PORT;
    uint8_t MISO_PIN;
    uint8_t MISO_MASK;

    volatile uint8_t *SCK_PIN_REG;
    volatile uint8_t *SCK_DDR;
    volatile uint8_t *SCK_PORT;
    uint8_t SCK_PIN;
    uint8_t SCK_MASK;

    volatile uint8_t *SS_PIN_REG;
    volatile uint8_t *SS_DDR;
    volatile uint8_t *SS_PORT;
    uint8_t SS_PIN;
    uint8_t SS_MASK;

    BitOrder bitOrder = BitOrder::MSBFirst;
    ClockPolarity clockPolarity = ClockPolarity::IdleLow;
    ClockPhase clockPhase = ClockPhase::SampleLeadingEdge;

    double delayLowUs = 1.0;
    double delayHighUs = 1.0;

    bool chipSelectActiveLow = true;
    bool autoChipSelect = true;

    void driveMosi(bool high);
    bool sampleMiso() const;
    void driveClockIdle();
    void driveClockActive();
    void driveChipSelect(bool active);
    uint8_t transferByteCore(uint8_t data);
};

#endif // PROTOCOL_SPI_H
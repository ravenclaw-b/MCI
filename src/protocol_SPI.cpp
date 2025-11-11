#include "protocol_SPI.h"

#include <util/delay.h>

SPI::SPI(volatile uint8_t *mosi_pin_reg, volatile uint8_t *mosi_ddr, volatile uint8_t *mosi_port, uint8_t mosi_pin,
		 volatile uint8_t *miso_pin_reg, volatile uint8_t *miso_ddr, volatile uint8_t *miso_port, uint8_t miso_pin,
		 volatile uint8_t *sck_pin_reg, volatile uint8_t *sck_ddr, volatile uint8_t *sck_port, uint8_t sck_pin,
		 volatile uint8_t *ss_pin_reg, volatile uint8_t *ss_ddr, volatile uint8_t *ss_port, uint8_t ss_pin)
	: MOSI_PIN_REG(mosi_pin_reg), MOSI_DDR(mosi_ddr), MOSI_PORT(mosi_port), MOSI_PIN(mosi_pin),
	  MOSI_MASK(static_cast<uint8_t>(1U << mosi_pin)),
	  MISO_PIN_REG(miso_pin_reg), MISO_DDR(miso_ddr), MISO_PORT(miso_port), MISO_PIN(miso_pin),
	  MISO_MASK(static_cast<uint8_t>(1U << miso_pin)),
	  SCK_PIN_REG(sck_pin_reg), SCK_DDR(sck_ddr), SCK_PORT(sck_port), SCK_PIN(sck_pin),
	  SCK_MASK(static_cast<uint8_t>(1U << sck_pin)),
	  SS_PIN_REG(ss_pin_reg), SS_DDR(ss_ddr), SS_PORT(ss_port), SS_PIN(ss_pin),
	  SS_MASK(static_cast<uint8_t>(1U << ss_pin))
{
}

void SPI::begin(bool autoChipSelectParam) {
	autoChipSelect = autoChipSelectParam;

	// Configure directions
	(*MOSI_DDR) |= MOSI_MASK;
	(*SCK_DDR) |= SCK_MASK;
	(*SS_DDR) |= SS_MASK;
	(*MISO_DDR) &= static_cast<uint8_t>(~MISO_MASK);

	// Default idle levels
	driveMosi(false);
	driveClockIdle();
	deselect();
}

void SPI::setBitOrder(BitOrder order) {
	bitOrder = order;
}

void SPI::setDataMode(uint8_t mode) {
	mode &= 0x03;
	ClockPolarity pol = (mode & 0x02) ? ClockPolarity::IdleHigh : ClockPolarity::IdleLow;
	ClockPhase pha = (mode & 0x01) ? ClockPhase::SampleTrailingEdge : ClockPhase::SampleLeadingEdge;
	setDataMode(pol, pha);
}

void SPI::setDataMode(ClockPolarity polarity, ClockPhase phase) {
	clockPolarity = polarity;
	clockPhase = phase;
	driveClockIdle();
}

void SPI::setDelaysMicroseconds(double lowPhase, double highPhase) {
	if (lowPhase < 0.0) {
		lowPhase = 0.0;
	}
	if (highPhase < 0.0) {
		highPhase = 0.0;
	}
	delayLowUs = lowPhase;
	delayHighUs = highPhase;
}

void SPI::setClockHz(uint32_t frequencyHz) {
	if (frequencyHz == 0) {
		return;
	}

	double periodUs = 1000000.0 / static_cast<double>(frequencyHz);
	double halfPeriod = periodUs * 0.5;

	if (halfPeriod < 0.25) {
		halfPeriod = 0.25;
	}

	delayLowUs = halfPeriod;
	delayHighUs = halfPeriod;
}

void SPI::setChipSelectPolarity(bool activeLow) {
	chipSelectActiveLow = activeLow;
	deselect();
}

void SPI::setAutoChipSelect(bool enable) {
	autoChipSelect = enable;
}

void SPI::select() {
	driveChipSelect(true);
}

void SPI::deselect() {
	driveChipSelect(false);
}

uint8_t SPI::transferByte(uint8_t data) {
	bool manageCs = autoChipSelect;
	if (manageCs) {
		select();
	}

	uint8_t value = transferByteCore(data);

	if (manageCs) {
		deselect();
	}

	return value;
}

void SPI::transferBytes(const uint8_t *tx, uint8_t *rx, size_t length) {
	if (length == 0) {
		return;
	}

	bool manageCs = autoChipSelect;
	if (manageCs) {
		select();
	}

	for (size_t i = 0; i < length; ++i) {
		uint8_t outbound = tx ? tx[i] : 0xFF;
		uint8_t inbound = transferByteCore(outbound);
		if (rx) {
			rx[i] = inbound;
		}
	}

	if (manageCs) {
		deselect();
	}
}

void SPI::writeBytes(const uint8_t *data, size_t length) {
	transferBytes(data, nullptr, length);
}

void SPI::driveMosi(bool high) {
	if (high) {
		(*MOSI_PORT) |= MOSI_MASK;
	} else {
		(*MOSI_PORT) &= static_cast<uint8_t>(~MOSI_MASK);
	}
}

bool SPI::sampleMiso() const {
	return ((*MISO_PIN_REG) & MISO_MASK) != 0;
}

void SPI::driveClockIdle() {
	if (clockPolarity == ClockPolarity::IdleLow) {
		(*SCK_PORT) &= static_cast<uint8_t>(~SCK_MASK);
	} else {
		(*SCK_PORT) |= SCK_MASK;
	}
}

void SPI::driveClockActive() {
	if (clockPolarity == ClockPolarity::IdleLow) {
		(*SCK_PORT) |= SCK_MASK;
	} else {
		(*SCK_PORT) &= static_cast<uint8_t>(~SCK_MASK);
	}
}

void SPI::driveChipSelect(bool active) {
	if (active) {
		if (chipSelectActiveLow) {
			(*SS_PORT) &= static_cast<uint8_t>(~SS_MASK);
		} else {
			(*SS_PORT) |= SS_MASK;
		}
	} else {
		if (chipSelectActiveLow) {
			(*SS_PORT) |= SS_MASK;
		} else {
			(*SS_PORT) &= static_cast<uint8_t>(~SS_MASK);
		}
	}
}

uint8_t SPI::transferByteCore(uint8_t data) {
	uint8_t received = 0;

	if (clockPhase == ClockPhase::SampleLeadingEdge) {
		driveClockIdle();

		for (uint8_t i = 0; i < 8; ++i) {
			uint8_t shift = (bitOrder == BitOrder::MSBFirst) ? static_cast<uint8_t>(7 - i) : i;
			bool outBit = ((data >> shift) & 0x01) != 0;

			driveMosi(outBit);
			if (delayLowUs > 0.0) {
				_delay_us(delayLowUs);
			}

			driveClockActive();
			if (delayHighUs > 0.0) {
				_delay_us(delayHighUs);
			}

			if (sampleMiso()) {
				received |= static_cast<uint8_t>(1U << shift);
			}

			driveClockIdle();
			if (delayLowUs > 0.0) {
				_delay_us(delayLowUs);
			}
		}
	} else {
		driveClockIdle();

		for (uint8_t i = 0; i < 8; ++i) {
			uint8_t shift = (bitOrder == BitOrder::MSBFirst) ? static_cast<uint8_t>(7 - i) : i;
			bool outBit = ((data >> shift) & 0x01) != 0;

			driveClockActive();
			if (delayHighUs > 0.0) {
				_delay_us(delayHighUs);
			}

			driveMosi(outBit);
			if (delayLowUs > 0.0) {
				_delay_us(delayLowUs);
			}

			driveClockIdle();

			if (sampleMiso()) {
				received |= static_cast<uint8_t>(1U << shift);
			}

			if (delayHighUs > 0.0) {
				_delay_us(delayHighUs);
			}
		}
	}

	return received;
}
// Software UART with pin-change-interrupt RX and bit-banged TX
#include "protocol_UART.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// Static registries for multi-instance support
UART *UART::instancesPCINT0[MAX_UARTS_PER_GROUP] = {nullptr};
UART *UART::instancesPCINT1[MAX_UARTS_PER_GROUP] = {nullptr};
UART *UART::instancesPCINT2[MAX_UARTS_PER_GROUP] = {nullptr};
uint8_t UART::countPCINT0 = 0;
uint8_t UART::countPCINT1 = 0;
uint8_t UART::countPCINT2 = 0;
volatile uint8_t UART::lastPINB = 0xFF;
volatile uint8_t UART::lastPINC = 0xFF;
volatile uint8_t UART::lastPIND = 0xFF;

// Precise cycle-based delay using _delay_loop_2 (4 cycles per count)
static inline void _bit_delay(uint16_t loops) {
    // loops=0 is a no-op; ensure minimal overhead
    if (loops) {
        _delay_loop_2(loops);
    }
}

// Constructor
UART::UART(volatile uint8_t *tx_pin_reg, volatile uint8_t *tx_ddr, volatile uint8_t *tx_port, uint8_t tx_pin,
           volatile uint8_t *rx_pin_reg, volatile uint8_t *rx_ddr, volatile uint8_t *rx_port, uint8_t rx_pin, unsigned long baud)
    : TX_PIN_REG(tx_pin_reg), TX_DDR(tx_ddr), TX_PORT(tx_port), TX_PIN(tx_pin),
      RX_PIN_REG(rx_pin_reg), RX_DDR(rx_ddr), RX_PORT(rx_port), RX_PIN(rx_pin),
    rxPCMSK(nullptr), rxPCIEBit(0), rxMask(0), rxGroupIdx(0xFF),
    baudrate(baud), bitDelayUs(0), halfBitUs(0), bitLoopCount(0), halfBitLoopCount(0) {

    // Configure TX pin as output, idle high
    *TX_DDR |= (1 << TX_PIN);
    *TX_PORT |= (1 << TX_PIN);

    // Configure RX pin as input with pull-up (idle high)
    *RX_DDR &= ~(1 << RX_PIN);
    *RX_PORT |= (1 << RX_PIN);

    // Precompute timing (approximate)
    if (baudrate == 0) baudrate = 9600; // fallback
    unsigned long bitus = 1000000UL / baudrate; // approximate whole bit duration
    if (bitus == 0) bitus = 1;
    bitDelayUs = (unsigned int)bitus;
    halfBitUs = (unsigned int)(bitus / 2U);
    if (halfBitUs == 0) halfBitUs = 1;

    // Compute loop counts: cycles per bit = F_CPU / baudrate
    // Subtract overhead for function call and loop iteration (~20-30 cycles)
    unsigned long cyclesPerBit = (F_CPU / baudrate);
    if (cyclesPerBit < 40) cyclesPerBit = 40; // sanity check
    
    // Account for overhead: function call + loop management ~24 cycles
    if (cyclesPerBit > 24) {
        cyclesPerBit -= 24;
    }
    
    // Round to multiple of 4 (since _delay_loop_2 consumes 4 cycles per count)
    cyclesPerBit -= (cyclesPerBit % 4UL);
    bitLoopCount = (uint16_t)(cyclesPerBit / 4UL);
    halfBitLoopCount = (uint16_t)(bitLoopCount / 2U);
    if (halfBitLoopCount == 0) halfBitLoopCount = 1;

    // Map RX_PIN_REG to PCINT group and mask
    if (RX_PIN_REG == &PINB) {
        rxPCMSK = &PCMSK0; rxPCIEBit = (1 << PCIE0); rxGroupIdx = 0;
    } else if (RX_PIN_REG == &PINC) {
        rxPCMSK = &PCMSK1; rxPCIEBit = (1 << PCIE1); rxGroupIdx = 1;
    } else if (RX_PIN_REG == &PIND) {
        rxPCMSK = &PCMSK2; rxPCIEBit = (1 << PCIE2); rxGroupIdx = 2;
    } else {
        rxPCMSK = nullptr; rxPCIEBit = 0; rxGroupIdx = 0xFF;
    }

    rxMask = (uint8_t)(1U << RX_PIN);

    // Register instance in proper group (if capacity allows)
    if (rxGroupIdx == 0) {
        if (countPCINT0 < MAX_UARTS_PER_GROUP) instancesPCINT0[countPCINT0++] = this;
    } else if (rxGroupIdx == 1) {
        if (countPCINT1 < MAX_UARTS_PER_GROUP) instancesPCINT1[countPCINT1++] = this;
    } else if (rxGroupIdx == 2) {
        if (countPCINT2 < MAX_UARTS_PER_GROUP) instancesPCINT2[countPCINT2++] = this;
    }

    // Enable pin-change interrupt for RX pin
    if (rxPCMSK && rxPCIEBit) {
        uint8_t sreg = SREG; cli();
        PCICR |= rxPCIEBit;       // enable PCINT group
        *rxPCMSK |= rxMask;       // enable this pin
        // Initialize last PIN snapshot
        if (rxGroupIdx == 0) lastPINB = PINB;
        else if (rxGroupIdx == 1) lastPINC = PINC;
        else if (rxGroupIdx == 2) lastPIND = PIND;
        SREG = sreg;              // restore
    }
}

// Transmit one byte, bit-banged
void UART::sendByte(uint8_t data) {
    uint8_t sreg = SREG; cli(); // Disable interrupts for accurate timing

    // Start bit (low)
    *TX_PORT &= ~(1 << TX_PIN);
    _bit_delay(bitLoopCount);

    // Data bits (LSB first)
    for (uint8_t i = 0; i < 8; i++) {
        if (data & (1 << i)) {
            *TX_PORT |= (1 << TX_PIN);
        } else {
            *TX_PORT &= ~(1 << TX_PIN);
        }
    _bit_delay(bitLoopCount);
    }

    // Stop bit (high)
    *TX_PORT |= (1 << TX_PIN);
    _bit_delay(bitLoopCount);

    SREG = sreg; // Restore interrupt state
    
    // CRITICAL: Add inter-byte delay for receiver to process
    // DFPlayer and other devices need ~1 bit time between bytes
    _bit_delay(bitLoopCount);
}

void UART::sendBytes(const uint8_t *data, unsigned int length) {
    for (unsigned int i = 0; i < length; i++) {
        sendByte(data[i]);
    }
}

void UART::sendString(const char *str) {
    while (*str) {
        sendByte(static_cast<uint8_t>(*str++));
    }
}

// -------- RX API --------
int UART::available() const {
    return (uint8_t)(RX_BUF_SIZE + rxHead - rxTail) % RX_BUF_SIZE;
}

int UART::peek() const {
    if (rxHead == rxTail) return -1;
    return rxBuf[rxTail];
}

int UART::read() {
    if (rxHead == rxTail) return -1;
    uint8_t b = rxBuf[rxTail];
    rxTail = (uint8_t)((rxTail + 1) % RX_BUF_SIZE);
    return b;
}

void UART::flush() {
    uint8_t sreg = SREG; cli();
    rxHead = rxTail = 0;
    SREG = sreg;
}

void UART::handleReceiveInterrupt(void (*callback)(uint8_t)) {
    rxCallback = callback;
}

inline void UART::storeRx(uint8_t b) {
    uint8_t next = (uint8_t)((rxHead + 1) % RX_BUF_SIZE);
    if (next != rxTail) {
        rxBuf[rxHead] = b;
        rxHead = next;
        if (rxCallback) rxCallback(b);
    } else {
        rxOverflowCount++;
    }
}

// Called from ISR context when RX pin changes state
void UART::onRxPinChange() {
    if (!rxPCMSK) return;
    // Falling edge start bit detection: ensure line is low now
    if ((*RX_PIN_REG & rxMask) != 0) return;
    // Disable this pin's PCINT during sampling to avoid reentry
    *rxPCMSK &= (uint8_t)~rxMask;
    sampleRx();
    *rxPCMSK |= rxMask;
}

// Blocking sampling routine (executed within ISR)
void UART::sampleRx() {
    // Wait half a bit to center of start bit, then verify it's still low
    _bit_delay(halfBitLoopCount);
    if ((*RX_PIN_REG & rxMask) != 0) {
        // Glitch; not a real start bit
        return;
    }

    // Now sample 8 data bits at bit centers
    uint8_t value = 0;
    for (uint8_t i = 0; i < 8; i++) {
    _bit_delay(bitLoopCount);
        if (*RX_PIN_REG & rxMask) {
            value |= (1 << i);
        }
    }

    // Stop bit check (should be high)
    _bit_delay(bitLoopCount);
    if ((*RX_PIN_REG & rxMask) == 0) {
        rxFrameErrorCount++;
        return; // framing error, drop the byte
    }

    storeRx(value);
}

// -------- Pin Change Interrupt Vectors --------
ISR(PCINT0_vect) {
    uint8_t current = PINB;
    uint8_t changed = current ^ UART::lastPINB;
    UART::lastPINB = current;
    if (changed) {
        for (uint8_t i = 0; i < UART::countPCINT0; i++) {
            UART *u = UART::instancesPCINT0[i];
            if (!u) continue;
            if (changed & u->rxMask) { // pin changed
                // Check falling edge: previously high, now low
                if (!(current & u->rxMask)) {
                    u->onRxPinChange();
                }
            }
        }
    }
}

ISR(PCINT1_vect) {
    uint8_t current = PINC;
    uint8_t changed = current ^ UART::lastPINC;
    UART::lastPINC = current;
    if (changed) {
        for (uint8_t i = 0; i < UART::countPCINT1; i++) {
            UART *u = UART::instancesPCINT1[i];
            if (!u) continue;
            if (changed & u->rxMask) {
                if (!(current & u->rxMask)) {
                    u->onRxPinChange();
                }
            }
        }
    }
}

ISR(PCINT2_vect) {
    uint8_t current = PIND;
    uint8_t changed = current ^ UART::lastPIND;
    UART::lastPIND = current;
    if (changed) {
        for (uint8_t i = 0; i < UART::countPCINT2; i++) {
            UART *u = UART::instancesPCINT2[i];
            if (!u) continue;
            if (changed & u->rxMask) {
                if (!(current & u->rxMask)) {
                    u->onRxPinChange();
                }
            }
        }
    }
}


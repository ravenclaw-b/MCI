#include <stdint.h>
#include <avr/io.h>

class UART {

public:
    // Bit-banged UART over arbitrary GPIO pins (AVR). RX uses Pin Change Interrupts.
    UART(volatile uint8_t *tx_pin_reg, volatile uint8_t *tx_ddr, volatile uint8_t *tx_port, uint8_t tx_pin,
         volatile uint8_t *rx_pin_reg, volatile uint8_t *rx_ddr, volatile uint8_t *rx_port, uint8_t rx_pin, unsigned long baud);

    UART() = delete;

    // Transmit
    void sendByte(uint8_t data);
    void sendBytes(const uint8_t *data, unsigned int length);
    void sendString(const char *str);

    // RX API (non-blocking)
    int available() const;   // number of bytes in RX buffer
    int read();              // returns -1 if none
    int peek() const;        // returns -1 if none
    void flush();            // clear RX buffer

    // Optional callback, called from ISR context when a byte is received
    void handleReceiveInterrupt(void (*callback)(uint8_t));

    // Error counters
    uint16_t overflowCount() const { return rxOverflowCount; }
    uint16_t frameErrorCount() const { return rxFrameErrorCount; }

private:
    // Registers for TX
    volatile uint8_t *TX_PIN_REG;
    volatile uint8_t *TX_DDR;
    volatile uint8_t *TX_PORT;
    uint8_t TX_PIN;

    // Registers for RX
    volatile uint8_t *RX_PIN_REG;
    volatile uint8_t *RX_DDR;
    volatile uint8_t *RX_PORT;
    uint8_t RX_PIN;

    // RX Pin Change Interrupt wiring
    volatile uint8_t *rxPCMSK;   // pointer to PCMSK0/1/2
    uint8_t rxPCIEBit;           // PCIE0/1/2 bit mask for PCICR
    uint8_t rxGroupIdx;          // 0: PCINT0(PINB), 1: PCINT1(PINC), 2: PCINT2(PIND), 0xFF invalid

    // Timing
    unsigned long baudrate;
    unsigned int bitDelayUs;     // whole bit time in microseconds
    unsigned int halfBitUs;      // half bit time
    uint16_t bitLoopCount;       // _delay_loop_2 count for full bit (4 cycles per count)
    uint16_t halfBitLoopCount;   // _delay_loop_2 count for half bit

    // RX buffering (ISR-driven)
    static const uint8_t RX_BUF_SIZE = 64;
    volatile uint8_t rxBuf[RX_BUF_SIZE];
    volatile uint8_t rxHead = 0;
    volatile uint8_t rxTail = 0;

    // Error counters
    volatile uint16_t rxOverflowCount = 0;
    volatile uint16_t rxFrameErrorCount = 0;

    // Optional user callback (ISR context!)
    void (*rxCallback)(uint8_t) = nullptr;

public:
    // Mask of the RX pin (public so ISRs can check it)
    uint8_t rxMask;              // (1 << RX_PIN) for edge detection

    // Instance registries per PCINT group (support multiple UARTs)
    static const uint8_t MAX_UARTS_PER_GROUP = 6;
    static UART *instancesPCINT0[MAX_UARTS_PER_GROUP];
    static UART *instancesPCINT1[MAX_UARTS_PER_GROUP];
    static UART *instancesPCINT2[MAX_UARTS_PER_GROUP];
    static uint8_t countPCINT0;
    static uint8_t countPCINT1;
    static uint8_t countPCINT2;

    // Last sampled PIN state per group for edge detection
    static volatile uint8_t lastPINB;
    static volatile uint8_t lastPINC;
    static volatile uint8_t lastPIND;

    // Called by ISR dispatchers
    void onRxPinChange();

    // Internal helpers
    void sampleRx();          // blocking sampling routine (inside ISR)
    inline void storeRx(uint8_t b);
};
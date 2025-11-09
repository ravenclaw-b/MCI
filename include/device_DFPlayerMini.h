#ifndef DFPLAYERMINI_H
#define DFPLAYERMINI_H

#include <stdint.h>
#include <protocol_UART.h>
#include <util/delay.h>

// DFPlayer Mini MP3 module driver over software UART
// Follows the project pattern: device_* inherits protocol_*
//
// Usage:
//   DFPlayerMini player(&PIND, &DDRD, &PORTD, PD4,    // TX pin (to DFPlayer RX)
//                       &PIND, &DDRD, &PORTD, PD5,    // RX pin (to DFPlayer TX)
//                       9600);                         // Baud rate
//   
//   sei();  // Enable interrupts for RX
//   delay(1000);  // Wait for module power-up
//   
//   if (!player.begin()) {
//     // Error: no card detected
//   }
//   
//   player.setVolume(25);  // 0-30
//   player.playTrack(1);   // Play 0001.mp3
//
class DFPlayerMini : protected UART {
public:
    DFPlayerMini(volatile uint8_t *tx_pin_reg, volatile uint8_t *tx_ddr, volatile uint8_t *tx_port, uint8_t tx_pin,
                 volatile uint8_t *rx_pin_reg, volatile uint8_t *rx_ddr, volatile uint8_t *rx_port, uint8_t rx_pin,
                 unsigned long baud = 9600, bool feedback = false)
        : UART(tx_pin_reg, tx_ddr, tx_port, tx_pin, rx_pin_reg, rx_ddr, rx_port, rx_pin, baud),
          wantFeedback(feedback) {}

    DFPlayerMini() = delete;

    // Initialize DFPlayer - gives module time to mount SD card
    // Call this in setup() to ensure module is ready before sending commands
    // Parameters:
    //   doReset: Send reset command (usually not needed, default false)
    //   timeoutMs: Delay to allow card mounting (default 2000ms = 2 seconds)
    // Returns: true (always, for API compatibility with response-checking versions)
    bool begin(bool doReset = false, uint16_t timeoutMs = 2000);

    // Basic control API
    void selectTF() { sendCommand(CMD_SEL_DEV, 0x0002); }
    void reset()    { sendCommand(CMD_RESET,  0x0000); }
    void setVolume(uint8_t vol) { if (vol > 30) vol = 30; sendCommand(CMD_SET_VOL, vol); }

    // Play controls
    void playTrack(uint16_t index) { if (index == 0) index = 1; sendCommand(CMD_PLAY_IDX, index); }
    void playFolderTrack(uint8_t folder, uint8_t index) { uint16_t p = ((uint16_t)folder << 8) | (uint16_t)index; sendCommand(CMD_PLAY_FOLDER, p); }
    void pause()  { sendCommand(CMD_PAUSE, 0x0000); }
    void resume() { sendCommand(CMD_RESUME,0x0000); }
    void next()   { sendCommand(CMD_NEXT,  0x0000); }
    void prev()   { sendCommand(CMD_PREV,  0x0000); }

    // Expose minimal RX to let users poll responses if feedback=true
    int available() const { return UART::available(); }
    int read()            { return UART::read(); }
    int peek() const      { return UART::peek(); }
    void flush()          { UART::flush(); }

    // Wait for DFPlayer to initialize (waits for "Card Online" response like official library)
    // Returns true if card detected, false on timeout
    bool waitForCardOnline(uint16_t timeoutMs = 3000);

private:
    bool wantFeedback;

    // DFPlayer command codes
    static const uint8_t CMD_NEXT        = 0x01;
    static const uint8_t CMD_PREV        = 0x02;
    static const uint8_t CMD_PLAY_IDX    = 0x03;
    static const uint8_t CMD_INC_VOL     = 0x04;
    static const uint8_t CMD_DEC_VOL     = 0x05;
    static const uint8_t CMD_SET_VOL     = 0x06;
    static const uint8_t CMD_SEL_DEV     = 0x09;
    static const uint8_t CMD_RESET       = 0x0C;
    static const uint8_t CMD_RESUME      = 0x0D;
    static const uint8_t CMD_PAUSE       = 0x0E;
    static const uint8_t CMD_PLAY_FOLDER = 0x0F;

    // DFPlayer response codes (from official library)
    static const uint8_t RSP_CARD_INSERTED = 0x3A;
    static const uint8_t RSP_CARD_ONLINE   = 0x3F;  // Card ready - bit 1 for TF card
    static const uint8_t RSP_USB_ONLINE    = 0x3F;  // USB ready - bit 0 for USB

    // Parse received 10-byte response frame
    // Returns: response command byte (0x3F for Card Online), or 0 if invalid/incomplete
    uint8_t parseResponse(uint16_t &parameter) {
        if (UART::available() < 10) return 0;  // Need full 10-byte frame
        
        uint8_t frame[10];
        for (int i = 0; i < 10; i++) {
            int b = UART::read();
            if (b < 0) return 0;
            frame[i] = (uint8_t)b;
        }
        
        // Validate frame structure
        if (frame[0] != 0x7E || frame[1] != 0xFF || frame[2] != 0x06 || frame[9] != 0xEF) {
            return 0;  // Invalid frame
        }
        
        // Validate checksum
        uint16_t sum = frame[1] + frame[2] + frame[3] + frame[4] + frame[5] + frame[6];
        uint16_t receivedCs = ((uint16_t)frame[7] << 8) | frame[8];
        uint16_t expectedCs = 0xFFFF - sum + 1;
        if (receivedCs != expectedCs) {
            return 0;  // Checksum mismatch
        }
        
        // Extract command and parameter
        parameter = ((uint16_t)frame[5] << 8) | frame[6];
        return frame[3];  // Return command byte
    }

    inline void sendCommand(uint8_t cmd, uint16_t param) {
        uint8_t f[10];
        f[0] = 0x7E;    // start
        f[1] = 0xFF;    // version
        f[2] = 0x06;    // length
        f[3] = cmd;     // command
        f[4] = wantFeedback ? 0x01 : 0x00; // feedback
        f[5] = (uint8_t)((param >> 8) & 0xFF);
        f[6] = (uint8_t)(param & 0xFF);
        uint16_t sum = (uint16_t)(f[1] + f[2] + f[3] + f[4] + f[5] + f[6]);
        uint16_t cs = (uint16_t)(0xFFFF - sum + 1);
        f[7] = (uint8_t)((cs >> 8) & 0xFF);
        f[8] = (uint8_t)(cs & 0xFF);
        f[9] = 0xEF;    // end
        UART::sendBytes(f, sizeof(f));
        
        // CRITICAL: DFPlayer needs time to process each command
        // Official library uses 10ms delay after each command
        _delay_ms(10);
    }
};

#endif // DFPLAYERMINI_H
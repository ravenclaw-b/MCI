#include <device_DFPlayerMini.h>
#include <util/delay.h>
#include <Arduino.h>  // For millis()

// Initialize DFPlayer module - waits for "Card Online" or "USB Online" response
// Official DFRobot library behavior:
// - Optionally sends reset command
// - Waits up to timeoutMs for module to report card/USB ready
// - Returns true if storage device detected
// 
// Note: Most DFPlayer clones don't send unsolicited "Card Online" messages.
// They only respond when feedback is explicitly requested in commands.
// So we use a simple delay-based approach instead.
bool DFPlayerMini::begin(bool doReset, uint16_t timeoutMs) {
    // Flush any stale data in RX buffer
    flush();
    
    if (doReset) {
        reset();
        _delay_ms(200);  // Give reset time to execute
    }
    
    // Most DFPlayer modules don't send automatic "Card Online" messages
    // unless feedback is enabled in every command. Instead, we just
    // give the module time to mount the SD card after power-up.
    // Official modules take 1-3 seconds to be ready.
    if (timeoutMs > 0) {
        delay(timeoutMs);
    }
    
    // Assume success - the module will simply not play if card is missing
    // User can check this by whether playback actually starts
    return true;
}

// Wait for DFPlayer module to send "Card Online" response
// Official library waits up to 2 seconds after reset
bool DFPlayerMini::waitForCardOnline(uint16_t timeoutMs) {
    unsigned long startTime = millis();
    
    while (millis() - startTime < timeoutMs) {
        if (available() >= 10) {
            uint16_t param;
            uint8_t cmd = parseResponse(param);
            
            // 0x3F is "Card Online" - check parameter for card type
            // bit 0 = USB, bit 1 = TF/SD card
            if (cmd == RSP_CARD_ONLINE) {
                if (param & 0x02) {  // TF card online
                    return true;
                }
                if (param & 0x01) {  // USB online
                    return true;
                }
            }
        }
        _delay_ms(10);  // Poll every 10ms
    }
    
    return false;  // Timeout
}

// DEPRECATED: Old convenience function - use begin() instead
bool DFPlayerMini_initializeTF(DFPlayerMini &df, uint8_t volume = 25) {
    // Give module time to boot, then select TF and set volume.
    _delay_ms(1500);
    df.selectTF();
    _delay_ms(120);
    df.setVolume(volume);
    _delay_ms(120);
    return true;
}

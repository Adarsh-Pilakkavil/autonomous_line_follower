#ifndef HAMMER_ARRAY_H
#define HAMMER_ARRAY_H

#include "Config.h"

/**
 * HammerArray: 16-Sensor 74HC165 Interface (INTEGER ONLY)
 * Resolution: -70 (Far Left) to 70 (Far Right)
 */

class HammerArray {
private:
    uint16_t sensorRaw;     
    int lastPosition = 0;
    
    // Integer weights: -70 to 70
    const int weights[16] = {
        -70, -60, -50, -40, -30, -20, -10, -5,
          5,  10,  20,  30,  40,  50,  60,  70
    };

public:
    void begin() {
        pinMode(PIN_HAMMER_LATCH, OUTPUT);
        pinMode(PIN_HAMMER_CLOCK, OUTPUT);
        pinMode(PIN_HAMMER_DATA, INPUT);
        digitalWrite(PIN_HAMMER_LATCH, HIGH);
        digitalWrite(PIN_HAMMER_CLOCK, LOW);
    }

    void read() {
        digitalWrite(PIN_HAMMER_LATCH, LOW);
        delayMicroseconds(2); 
        digitalWrite(PIN_HAMMER_LATCH, HIGH);

        uint16_t raw = 0;
        for(int i = 0; i < 16; i++) {
            if(digitalRead(PIN_HAMMER_DATA) == LOW) { 
                raw |= (1 << (15 - i)); 
            }
            digitalWrite(PIN_HAMMER_CLOCK, HIGH);
            delayMicroseconds(1);
            digitalWrite(PIN_HAMMER_CLOCK, LOW);
        }
        sensorRaw = raw;
    }

    /**
     * Returns: Integer position -70 to 70.
     * Returns: -999 if no line detected.
     */
    int HammerArray::calculatePosition() {
    uint16_t raw = getRaw(); // Grabs your 16 bits
    
    // Check if the line is completely lost
    if (raw == 0) {
        return -999; // Your line-lost failsafe flag
    }

    long weightedSum = 0;
    int activeSensors = 0;

    int weights[16] = {-150, -100, -70, -45, -25, -15, -5, -1, 1, 5, 15, 25, 45, 70, 100, 150};

    for (int i = 0; i < 16; i++) {
        // Extract the bit for the current sensor (assuming Left-to-Right layout)
        bool seesLine = (raw >> (15 - i)) & 0x01; 
        
        if (seesLine) {
            weightedSum += weights[i];
            activeSensors++;
        }
    }

    // Calculate the final smoothed position
    int pos = weightedSum / activeSensors;
    
    return pos;
}
    uint16_t getRaw() { return sensorRaw; }
    bool isAllBlack() { return sensorRaw == 0xFFFF; }
    bool isFarLeft()  { return (sensorRaw & 0xE000) != 0; } 
    bool isFarRight() { return (sensorRaw & 0x0007) != 0; }
};

#endif

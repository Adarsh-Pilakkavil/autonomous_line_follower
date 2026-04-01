#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ===========================================
// PIN DEFINITIONS (INTEGER OPTIMIZED)
// ===========================================

// --- 16-Sensor 74HC165 Interface ---
const uint8_t PIN_HAMMER_LATCH = A0;   
const uint8_t PIN_HAMMER_CLOCK = A2;   
const uint8_t PIN_HAMMER_DATA  = A3;  

// --- L298N Motor Driver Pins ---
const uint8_t ENA = 11; // PWM Left Motor
const uint8_t IN1 = 2;  // Left Motor Direction 1
const uint8_t IN2 = 3;  // Left Motor Direction 2
const uint8_t IN3 = 4;  // Right Motor Direction 1
const uint8_t IN4 = 5;  // Right Motor Direction 2
const uint8_t ENB = 6;  // PWM Right Motor

// --- Ultrasonic HC-SR04 ---
const uint8_t PIN_TRIG = 7; // Moved from 2 to avoid conflict with IN1
const uint8_t PIN_ECHO = 9;

// --- PID CONSTANTS (Scaled Integers) ---
// Error is now in range -70 to 70
const uint8_t BASE_SPEED = 180; 
const uint8_t MAX_SPEED  = 255;

const int Kp = 40; 
const int Ki = 1;  
const int Kd = 150; 

// --- I2C Addresses ---
const uint8_t ADDR_MPU6050 = 0x68; 
const uint8_t ADDR_TCS34725 = 0x29; 
const uint8_t ADDR_COLOR = 0x29; 
const uint8_t ADDR_REAR_ARRAY = 0x20; 

// --- Enums ---
enum DetectedColor {
    COLOR_BLACK,
    COLOR_WHITE,
    COLOR_RED,
    COLOR_BLUE,
    COLOR_UNKNOWN
};

#endif

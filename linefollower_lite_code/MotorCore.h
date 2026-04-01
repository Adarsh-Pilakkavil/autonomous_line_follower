#ifndef MOTOR_CORE_H
#define MOTOR_CORE_H

#include "Config.h"

class MotorCore {
public:
    void begin() {
        pinMode(ENA, OUTPUT);
        pinMode(IN1, OUTPUT);
        pinMode(IN2, OUTPUT);
        pinMode(IN3, OUTPUT);
        pinMode(IN4, OUTPUT);
        pinMode(ENB, OUTPUT);
        
        stop();
    }

    // speed: -255 to 255
    void setSpeeds(int leftSpeed, int rightSpeed, int balance = 100) {
        // Clamp balance and normalize speeds
        balance = constrain(balance, 50, 150);
        
        int bLeft = 200 - balance;
        int bRight = balance;

        int finalLeft = (leftSpeed * bLeft) / 100;
        int finalRight = (rightSpeed * bRight) / 100;

        // Clamp final speeds
        finalLeft = constrain(finalLeft, -255, 255);
        finalRight = constrain(finalRight, -255, 255);

        // Left Motor
        if(finalLeft >= 0) {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            analogWrite(ENA, finalLeft);
        } else {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            analogWrite(ENA, -finalLeft);
        }

        // Right Motor
        if(finalRight >= 0) {
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            analogWrite(ENB, finalRight);
        } else {
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            analogWrite(ENB, -finalRight);
        }
    }

    void stop() {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        analogWrite(ENA, 0);
        analogWrite(ENB, 0);
    }

    // Forced Diagnostic Test
    void diagnosticTest() {
        Serial.println(F("--- Motor Diagnostic Start ---"));
        Serial.print(F("Left (IN1=2, IN2=3, ENA=11): "));
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, 150);
        Serial.println(F("Fwd 150"));
        delay(1000);
        stop();
        delay(500);
        
        Serial.print(F("Right (IN3=4, IN4=5, ENB=6): "));
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, 150);
        Serial.println(F("Fwd 150"));
        delay(1000);
        stop();
        Serial.println(F("--- Diagnostic End ---"));
    }
};

#endif

/*
 * ======================================================================================
 * PROJECT: TRACE AND TRANSFORM - MK1001 LITE (SIMPLE PID)
 * ======================================================================================
 * Simple PID Line Follower with Debugging and Motor Test Modes.
 * 
 * Hardware:
 * - HammerArray (16-sensor 74HC165 interface)
 * - L298N Motor Driver
 * ======================================================================================
 */

#include <EEPROM.h>
#include "Config.h"
#include "HammerArray.h"
#include "MotorCore.h"

// EEPROM Memory Addresses
const int ADDR_KP       = 0;
const int ADDR_KI       = 4;
const int ADDR_KD       = 8;
const int ADDR_BASE     = 12;
const int ADDR_BAL      = 16;
const int ADDR_MAX_SPD  = 20;
const int ADDR_INIT     = 100; 

// Objects
HammerArray hammer;
MotorCore motors;

// Live PID & Failsafe Backup Variables
int currentKp, currentKi, currentKd, currentBase, currentBal, currentMaxSpeed;
int backupKp, backupKi, backupKd, backupBase, backupBal, backupMaxSpeed;
int testSpeed = 180;

// PID Runtime Variables
int error = 0, lastError = 0, lastValidPos = 0;
long integral = 0;

// Track Logic Memory
bool telemetryActive = false, rawArrayActive = false;
bool motorTestActive = false; 

// State Tracking
enum RobotState { FOLLOWING, DEBUGGING_MODE, MOTOR_TEST_MODE };
RobotState currentState = FOLLOWING;
RobotState lastState = DEBUGGING_MODE; 

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(10); 

    motors.begin();
    hammer.begin();

    loadSettings();
    Serial.println(F("\n--- MK1001 LITE (SIMPLE PID) READY ---"));
    Serial.print(F("Current PID: P=")); Serial.print(currentKp);
    Serial.print(F(" I=")); Serial.print(currentKi);
    Serial.print(F(" D=")); Serial.print(currentKd);
    Serial.print(F(" BaseSpd=")); Serial.print(currentBase);
    Serial.print(F(" MaxSpd=")); Serial.println(currentMaxSpeed);
    printHelp();
}

void loop() {
    if (Serial.available() > 0) handleSerialCLI();
    
    if (currentState != lastState) {
        reportStateChange();
        lastState = currentState;
    }

    hammer.read();
    int pos = hammer.calculatePosition(); 

    if (telemetryActive) streamTelemetry(pos);
    if (rawArrayActive) streamRawArray();
    if (currentState == DEBUGGING_MODE) runLiteDebugger(pos);

    switch (currentState) {
        case FOLLOWING:
            if (pos == -999) {
                // Line Lost: Spin Turn to find line faster
                if (lastValidPos < 0) {
                    motors.setSpeeds(120, -120, currentBal); // Fix: Spin Left
                } else if (lastValidPos > 0) {
                    motors.setSpeeds(-120, 120, currentBal); // Fix: Spin Right
                } else {
                    motors.stop(); // Nowhere to go, stop
                }
            } else {
                // Line Found: Run PID
                runPID(pos);
            }
            break;

        case DEBUGGING_MODE: 
            if (!motorTestActive) motors.stop(); 
            break;

        case MOTOR_TEST_MODE:
            // Motors are controlled via CLI directly, loop does not override
            break;
    }
}

void reportStateChange() {
    Serial.print(F("\n[STATE CHANGE] >>> "));
    switch (currentState) {
        case FOLLOWING:         Serial.println(F("FOLLOWING")); break;
        case DEBUGGING_MODE:    Serial.println(F("DEBUGGING_MODE")); break;
        case MOTOR_TEST_MODE:   Serial.println(F("MOTOR_TEST_MODE")); break;
    }
}

void runLiteDebugger(int pos) {
    static unsigned long lastD = 0;
    if (millis() - lastD > 100) {
        lastD = millis();
        uint16_t raw = hammer.getRaw();
        Serial.print(F("Dbg: [Array:"));
        for (int i = 0; i < 16; i++) Serial.print((raw >> (15 - i)) & 0x01);
        Serial.print(F("] Pos:")); Serial.print(pos);
        if (motorTestActive) Serial.print(F(" | MOTORS RUNNING"));
        Serial.println();
    }
}

void loadSettings() {
    if (EEPROM.read(ADDR_INIT) != 'P') {
        currentKp = Kp; currentKi = Ki; currentKd = Kd; 
        currentBase = BASE_SPEED; currentBal = 100; 
        currentMaxSpeed = MAX_SPEED;
        saveSettings();
        EEPROM.write(ADDR_INIT, 'P');
    } else {
        EEPROM.get(ADDR_KP, currentKp);
        EEPROM.get(ADDR_KI, currentKi);
        EEPROM.get(ADDR_KD, currentKd);
        EEPROM.get(ADDR_BASE, currentBase);
        EEPROM.get(ADDR_BAL, currentBal);
        EEPROM.get(ADDR_MAX_SPD, currentMaxSpeed);
        if (currentBal < 50 || currentBal > 150) currentBal = 100;
        if (currentMaxSpeed < 50 || currentMaxSpeed > 255) currentMaxSpeed = 255;
    }
    syncBackup();
}

void saveSettings() {
    EEPROM.put(ADDR_KP, currentKp);
    EEPROM.put(ADDR_KI, currentKi);
    EEPROM.put(ADDR_KD, currentKd);
    EEPROM.put(ADDR_BASE, currentBase);
    EEPROM.put(ADDR_BAL, currentBal);
    EEPROM.put(ADDR_MAX_SPD, currentMaxSpeed);
}

void syncBackup() {
    backupKp = currentKp; backupKi = currentKi; 
    backupKd = currentKd; backupBase = currentBase;
    backupBal = currentBal; backupMaxSpeed = currentMaxSpeed;
}

void printHelp() {
    Serial.println(F("\n--- SIMPLE PID CLI COMMANDS ---"));
    Serial.println(F("g : Toggle DEBUGGING MODE (Sensor Print)"));
    Serial.println(F("m : Toggle MOTOR TEST MODE"));
    Serial.println(F("f : Resume FOLLOWING"));
    Serial.println(F("x : STOP"));
    Serial.println(F("c : Show Current PID & BaseSpeed"));
    Serial.println(F("v <val> : Set Global Test Speed (for 8/2/4/6)"));
    Serial.print(F("y <val> : Motor Balance (Current: ")); Serial.print(currentBal); Serial.println(F(")"));
    
    Serial.println(F("\n--- GLOBAL PID TUNING ---"));
    Serial.println(F("p/i/d/b/s <val> : Tune PID, BaseSpeed, MaxSpeed (e.g. 's 200')"));
    Serial.println(F("u/r             : Undo/Reset PID"));
    
    if (currentState == MOTOR_TEST_MODE || currentState == DEBUGGING_MODE) {
        Serial.println(F("\n--- MOTOR TEST CONTROLS ---"));
        Serial.println(F("8/2/4/6 : Fwd/Back/Left/Right"));
        Serial.println(F("5       : Stop Motors"));
        Serial.println(F("L <val> : Test LEFT Motor"));
        Serial.println(F("R <val> : Test RIGHT Motor"));
        Serial.println(F("D       : Run Hardware Diagnostic Test"));
    }
}

void handleSerialCLI() {
    while (Serial.available() > 0) {
        char cmd = Serial.read();
        if (isspace(cmd)) continue;

        int val = 0;
        // If the command is one that expects a value, parse the integer. 
        // parseInt() automatically skips leading spaces.
        if (cmd == 'p' || cmd == 'i' || cmd == 'd' || cmd == 'b' || cmd == 's' || cmd == 'y' || cmd == 'v' || cmd == 'L' || cmd == 'R') {
            val = Serial.parseInt();
        }
        
        // Context-aware commands
        if (currentState == DEBUGGING_MODE || currentState == MOTOR_TEST_MODE) {
            switch (cmd) {
                case '8': motorTestActive = true; motors.setSpeeds(testSpeed, testSpeed, currentBal); continue;
                case '2': motorTestActive = true; motors.setSpeeds(-testSpeed, -testSpeed, currentBal); continue;
                case '4': motorTestActive = true; motors.setSpeeds(-testSpeed, testSpeed, currentBal); continue;
                case '6': motorTestActive = true; motors.setSpeeds(testSpeed, -testSpeed, currentBal); continue;
                case '5': motorTestActive = false; motors.stop(); continue;
                case 'L': motorTestActive = true; motors.setSpeeds(val, 0, currentBal); Serial.print(F("M:L->")); Serial.println(val); continue;
                case 'R': motorTestActive = true; motors.setSpeeds(0, val, currentBal); Serial.print(F("M:R->")); Serial.println(val); continue;
                case 'D': motors.diagnosticTest(); continue;
            }
        }

        // Global EEPROM Tuning Commands
        if (cmd == 'p' || cmd == 'i' || cmd == 'd' || cmd == 'b' || cmd == 's' || cmd == 'y') syncBackup(); 
        switch (cmd) {
            case 'v': testSpeed = val; Serial.print(F("Test speed->")); Serial.println(testSpeed); continue;
            case 'c': 
                Serial.print(F("Current PID: P=")); Serial.print(currentKp);
                Serial.print(F(" I=")); Serial.print(currentKi);
                Serial.print(F(" D=")); Serial.print(currentKd);
                Serial.print(F(" BaseSpd=")); Serial.print(currentBase);
                Serial.print(F(" MaxSpd=")); Serial.println(currentMaxSpeed);
                continue;
            case 'p': currentKp = val; saveSettings(); Serial.print(F("Kp->")); Serial.println(val); continue;
            case 'i': currentKi = val; saveSettings(); Serial.print(F("Ki->")); Serial.println(val); continue;
            case 'd': currentKd = val; saveSettings(); Serial.print(F("Kd->")); Serial.println(val); continue;
            case 'b': currentBase = val; saveSettings(); Serial.print(F("Base->")); Serial.println(val); continue;
            case 's': currentMaxSpeed = val; saveSettings(); Serial.print(F("MaxSpd->")); Serial.println(val); continue;
            case 'y': currentBal = val; saveSettings(); Serial.print(F("Bal->")); Serial.println(val); continue;
            case 'u': 
                currentKp = backupKp; currentKi = backupKi; currentKd = backupKd; currentBase = backupBase; currentBal = backupBal; currentMaxSpeed = backupMaxSpeed;
                saveSettings(); Serial.println(F("Undo Changes.")); continue;
            case 'r': 
                currentKp = Kp; currentKi = Ki; currentKd = Kd; currentBase = BASE_SPEED; currentBal = 100; currentMaxSpeed = MAX_SPEED;
                saveSettings(); Serial.println(F("Reset All.")); continue;
        }

        switch (cmd) {
            case '?':
            case 'h': printHelp(); break;
            case 'g':
                if (currentState == DEBUGGING_MODE) {
                    currentState = FOLLOWING;
                    Serial.println(F("[MODE] Exited Debugging. Resuming FOLLOWING."));
                } else {
                    currentState = DEBUGGING_MODE;
                    Serial.println(F("[MODE] DEBUGGING_MODE Active. Sensor Array output started."));
                }
                motorTestActive = false;
                motors.stop();
                break;
            case 'm':
                if (currentState == MOTOR_TEST_MODE) currentState = FOLLOWING;
                else {
                    currentState = MOTOR_TEST_MODE;
                    Serial.println(F("[MODE] MOTOR_TEST_MODE Active. Use 8/2/4/6 to move."));
                }
                motorTestActive = false;
                motors.stop();
                break;
            case 'f': currentState = FOLLOWING; integral = 0; break;
            case 'x': currentState = DEBUGGING_MODE; motors.stop(); break; // Simplified STOP
            case 'a': rawArrayActive = !rawArrayActive; break;
            case 't': telemetryActive = !telemetryActive; break;
        }
    }
}

void runPID(int pos) {
    // Multiply pos by -1 to fix opposite motor configuration
    error = -pos;
    integral = constrain(integral + error, -500, 500);
    int correction = (error * currentKp) + (int)(integral * currentKi) + ((error - lastError) * currentKd);
    lastError = error;
    
    if(pos != 0) lastValidPos = pos; // Save last known side

    int leftSpeed = currentBase + correction;
    int rightSpeed = currentBase - correction;

    // Allow motors to spin backwards for sharp/spin turns
    leftSpeed = constrain(leftSpeed, -currentMaxSpeed/4, currentMaxSpeed);
    rightSpeed = constrain(rightSpeed, -currentMaxSpeed/4, currentMaxSpeed);

    motors.setSpeeds(leftSpeed, rightSpeed, currentBal);
}

void streamTelemetry(int pos) {
    static unsigned long lastT = 0;
    if (millis() - lastT > 50) {
        lastT = millis();
        Serial.print(pos); Serial.print(F(","));
        Serial.print(currentKp); Serial.print(F(","));
        Serial.print(currentKi); Serial.print(F(","));
        Serial.println(currentKd);
    }
}

void streamRawArray() {
    static unsigned long lastA = 0;
    if (millis() - lastA > 100) {
        lastA = millis();
        uint16_t raw = hammer.getRaw();
        for (int i = 0; i < 16; i++) {
            Serial.print((raw >> (15 - i)) & 0x01);
        }
        Serial.println();
    }
}

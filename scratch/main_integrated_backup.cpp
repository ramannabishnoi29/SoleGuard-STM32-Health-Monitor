/**
 * ============================================================================
 * SoleGuard - Complete Integrated Medical Sensor System & IMU Diagnostic
 * Platform: STM32F411CEU6 Black Pill (100MHz Cortex-M4, 128KB RAM, 512KB Flash)
 * ============================================================================
 * 
 * SENSORS & ACTUATORS INTEGRATED:
 *  1. OLED Display: SSD1306 128x64 I2C (PB6 = SCL, PB7 = SDA, Addr = 0x3C / 0x3D)
 *  2. Plantar Skin Temperature: TMP117 I2C (PB6 = SCL, PB7 = SDA, Addr = 0x48)
 *  3. 6-Axis Motion IMU: MPU-6050 I2C (Direct Hardware Register Driver + Adafruit)
 *     - Probed at 0x68 and 0x69
 *     - Reads WHO_AM_I register (0x75)
 *     - Clears SLEEP bit in PWR_MGMT_1 (0x6B)
 *     - Reads 3-axis Accelerometer & 3-axis Gyroscope
 *  4. Heart Rate & SpO2 Pulse Oximeter (Auto-Detected):
 *     - MAX30102 (I2C Addr 0x57, Part ID 0x15)
 *     - MAX30100 (I2C Addr 0x57, Part ID 0x11)
 *     - Analog Pulse Sensor (PA3 / ADC1_IN3 as fallback / parallel channel)
 *  5. Plantar Pressure Sensors (FSR 402 with 10k pull-down dividers):
 *     - PA1 (ADC1_IN1): Forefoot / 1st Metatarsal head
 *     - PA2 (ADC1_IN2): Calcaneus / Heel
 *  6. Audio Alert Buzzer: PB1 (and PB0 mirrored) - Sounds when BOTH FSRs > 2000 counts!
 *  7. Status LED: PC13 (Active-LOW MCU heartbeat)
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <math.h>

// ----------------------------------------------------------------------------
// Pin Definitions
// ----------------------------------------------------------------------------
#define PIN_I2C_SCL       PB6
#define PIN_I2C_SDA       PB7
#define PIN_FSR_FOREFOOT  PA1
#define PIN_FSR_HEEL      PA2
#define PIN_PULSE_ANALOG  PA3
#define PIN_BUZZER        PB1   // Primary Buzzer Pin
#define PIN_BUZZER_ALT    PB0   // Secondary / Alternate Buzzer Pin
#define PIN_LED           PC13

// ----------------------------------------------------------------------------
// Thresholds & Constants
// ----------------------------------------------------------------------------
#define SCREEN_WIDTH        128
#define SCREEN_HEIGHT       64
#define OLED_RESET          -1
#define TMP117_ADDR         0x48
#define MAX3010X_ADDR       0x57

#define FSR_HIGH_THRESHOLD  2000  // Counts for HIGH pressure (Dual High triggers Buzzer)

// ----------------------------------------------------------------------------
// Enums & Structs
// ----------------------------------------------------------------------------
enum SensorType {
    HR_SENSOR_NONE,
    HR_SENSOR_MAX30102,
    HR_SENSOR_MAX30100,
    HR_SENSOR_ANALOG
};

// ----------------------------------------------------------------------------
// Global Sensor Objects
// ----------------------------------------------------------------------------
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MAX30105 max30102;

bool oledReady = false;
uint8_t oledAddr = 0x3C;
bool tmpFound = false;
float currentTempC = 0.0f;
uint8_t tmpFailCount = 0;

// MPU-6050 State & Direct Register Readout
bool mpuFound = false;
uint8_t mpuAddr = 0;
uint8_t mpuChipID = 0;
float accelX = 0, accelY = 0, accelZ = 9.8f;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float totalAccelMag = 9.8f;
int pitchAngle = 0;
int rollAngle = 0;
bool isGaitWalking = false;

// I2C Bus Scan Results string for diagnostic splash
char i2cScanList[32] = "";

SensorType activeHRSensor = HR_SENSOR_ANALOG;
bool hrSensorDetected = false;

// Pressure Readings (12-bit ADC: 0 - 4095)
int rawFSR1 = 0;
int rawFSR2 = 0;
float voltFSR1 = 0.0f;
float voltFSR2 = 0.0f;
bool bothPressureHigh = false;

// Heart Rate & SpO2 Metrics
int heartRateBPM = 0;
int bloodOxygenSpO2 = 0;
bool beatDetected = false;
unsigned long lastBeatTime = 0;
bool fingerOnSensor = false;

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;

// Analog Pulse Tracking
int rawPulseAnalog = 0;
int pulseSignal = 0;
int pulseMin = 4095;
int pulseMax = 0;

// Non-blocking Scheduling Timers
unsigned long lastFastSample = 0;  // 20 ms (50 Hz) for ADC & Pulse
unsigned long lastImuSample = 0;   // 100 ms (10 Hz) for MPU-6050
unsigned long lastOledUpdate = 0;  // 200 ms (5 Hz) for Display
unsigned long lastTempRead = 0;    // 1000 ms (1 Hz, staggered)
unsigned long lastLedToggle = 0;   // 500 ms (2 Hz)
unsigned long frameCount = 0;

// ----------------------------------------------------------------------------
// Non-blocking I2C address probe
// ----------------------------------------------------------------------------
bool probeI2C(uint8_t addr) {
    Wire.beginTransmission(addr);
    return (Wire.endTransmission(true) == 0);
}

// ----------------------------------------------------------------------------
// Read 1 byte from an I2C register
// ----------------------------------------------------------------------------
uint8_t readI2CRegister(uint8_t devAddr, uint8_t regAddr) {
    Wire.beginTransmission(devAddr);
    Wire.write(regAddr);
    if (Wire.endTransmission(false) != 0) return 0x00;
    if (Wire.requestFrom((uint8_t)devAddr, (uint8_t)1) == 1) {
        return Wire.read();
    }
    return 0x00;
}

// ----------------------------------------------------------------------------
// Write 1 byte to an I2C register
// ----------------------------------------------------------------------------
bool writeI2CRegister(uint8_t devAddr, uint8_t regAddr, uint8_t val) {
    Wire.beginTransmission(devAddr);
    Wire.write(regAddr);
    Wire.write(val);
    return (Wire.endTransmission(true) == 0);
}

// ----------------------------------------------------------------------------
// Hardware I2C Bus Auto-Recovery
// ----------------------------------------------------------------------------
void recoverI2CBus() {
    Wire.end();

    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, OUTPUT_OPEN_DRAIN);
    digitalWrite(PIN_I2C_SCL, HIGH);
    delayMicroseconds(10);

    for (int i = 0; i < 16; i++) {
        digitalWrite(PIN_I2C_SCL, LOW);
        delayMicroseconds(10);
        digitalWrite(PIN_I2C_SCL, HIGH);
        delayMicroseconds(10);
    }

    pinMode(PIN_I2C_SDA, OUTPUT_OPEN_DRAIN);
    digitalWrite(PIN_I2C_SDA, LOW);
    delayMicroseconds(10);
    digitalWrite(PIN_I2C_SCL, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_I2C_SDA, HIGH);
    delayMicroseconds(10);

    Wire.setSCL(PIN_I2C_SCL);
    Wire.setSDA(PIN_I2C_SDA);
    Wire.begin();
    Wire.setClock(400000);
}

// ----------------------------------------------------------------------------
// Safe TMP117 Temperature Read with Repeated-START
// ----------------------------------------------------------------------------
bool readTMP117(float &tempOut) {
    Wire.beginTransmission(TMP117_ADDR);
    Wire.write(0x00);
    if (Wire.endTransmission(false) != 0) return false;

    if (Wire.requestFrom((uint8_t)TMP117_ADDR, (uint8_t)2) == 2) {
        uint8_t msb = Wire.read();
        uint8_t lsb = Wire.read();
        int16_t raw = (int16_t)((msb << 8) | lsb);
        tempOut = (float)raw * 0.0078125f;
        return true;
    }
    return false;
}

// ----------------------------------------------------------------------------
// Direct Hardware Driver for MPU-6050 (Universal / Zero-Library-Failure)
// ----------------------------------------------------------------------------
bool initMPU6050Hardware(uint8_t addr) {
    // 1. Read WHO_AM_I register (0x75)
    mpuChipID = readI2CRegister(addr, 0x75);
    Serial.print(F("MPU WHO_AM_I at 0x"));
    Serial.print(addr, HEX);
    Serial.print(F(" = 0x"));
    Serial.println(mpuChipID, HEX);

    // Accept genuine MPU-6050 (0x68), MPU-6500 (0x70), MPU-9250 (0x71/0x73) or clones (0x98/0x72)
    // If chip responds over I2C, proceed to wake up!

    // 2. Wake up MPU-6050: Write 0x00 to PWR_MGMT_1 (0x6B) to clear SLEEP bit!
    if (!writeI2CRegister(addr, 0x6B, 0x00)) return false;
    delay(20);

    // 3. Set Sample Rate Divisor: 1kHz / (1 + 7) = 125 Hz
    writeI2CRegister(addr, 0x19, 0x07);

    // 4. Set Digital Low Pass Filter (DLPF): ~21 Hz bandwidth (CONFIG register 0x1A)
    writeI2CRegister(addr, 0x1A, 0x04);

    // 5. Set Gyro Full Scale: +/- 500 deg/s (GYRO_CONFIG register 0x1B, FS_SEL = 1 -> 0x08)
    writeI2CRegister(addr, 0x1B, 0x08);

    // 6. Set Accelerometer Full Scale: +/- 4g (ACCEL_CONFIG register 0x1C, AFS_SEL = 1 -> 0x08)
    writeI2CRegister(addr, 0x1C, 0x08);

    return true;
}

// ----------------------------------------------------------------------------
// Direct Hardware Read from MPU-6050 (Registers 0x3B to 0x48)
// ----------------------------------------------------------------------------
void sampleMPU6050() {
    if (!mpuFound || mpuAddr == 0) return;

    // Burst read 14 bytes starting at ACCEL_XOUT_H (0x3B)
    Wire.beginTransmission(mpuAddr);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) return;

    if (Wire.requestFrom((uint8_t)mpuAddr, (uint8_t)14) == 14) {
        int16_t rawAX = (Wire.read() << 8) | Wire.read();
        int16_t rawAY = (Wire.read() << 8) | Wire.read();
        int16_t rawAZ = (Wire.read() << 8) | Wire.read();
        int16_t rawT  = (Wire.read() << 8) | Wire.read(); (void)rawT; // Internal die temp
        int16_t rawGX = (Wire.read() << 8) | Wire.read();
        int16_t rawGY = (Wire.read() << 8) | Wire.read();
        int16_t rawGZ = (Wire.read() << 8) | Wire.read();

        // Convert +/-4g accelerometer raw counts (8192 LSB/g) to m/s^2
        accelX = ((float)rawAX / 8192.0f) * 9.80665f;
        accelY = ((float)rawAY / 8192.0f) * 9.80665f;
        accelZ = ((float)rawAZ / 8192.0f) * 9.80665f;

        // Convert +/-500 deg/s gyro raw counts (65.5 LSB/deg/s) to deg/s
        gyroX = (float)rawGX / 65.5f;
        gyroY = (float)rawGY / 65.5f;
        gyroZ = (float)rawGZ / 65.5f;

        // Total vector acceleration magnitude
        totalAccelMag = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

        // Compute Pitch and Roll in degrees
        pitchAngle = (int)(atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 57.2958f);
        rollAngle  = (int)(atan2(accelY, accelZ) * 57.2958f);

        // Gait Detection: if acceleration deviates from gravity (9.8) by > 1.8 or gyro > 35 deg/s
        float dynAccel = abs(totalAccelMag - 9.8f);
        float dynGyro  = abs(gyroX) + abs(gyroY) + abs(gyroZ);
        isGaitWalking = (dynAccel > 1.8f || dynGyro > 35.0f);
    }
}

// ----------------------------------------------------------------------------
// Initialize MAX30100 Registers
// ----------------------------------------------------------------------------
bool initMAX30100Hardware() {
    if (!writeI2CRegister(MAX3010X_ADDR, 0x06, 0x03)) return false;
    if (!writeI2CRegister(MAX3010X_ADDR, 0x07, 0x47)) return false;
    if (!writeI2CRegister(MAX3010X_ADDR, 0x09, 0x8F)) return false;
    return true;
}

// ----------------------------------------------------------------------------
// Process MAX30102 Samples
// ----------------------------------------------------------------------------
void updateMAX30102Processing() {
    long irValue = max30102.getIR();
    long redValue = max30102.getRed();

    if (irValue > 50000) {
        fingerOnSensor = true;
        if (checkForBeat(irValue)) {
            beatDetected = true;
            unsigned long delta = millis() - lastBeatTime;
            lastBeatTime = millis();
            if (delta > 300 && delta < 1500) {
                float instantBPM = 60000.0f / (float)delta;
                if (instantBPM >= 45 && instantBPM <= 200) {
                    rates[rateSpot++] = (byte)instantBPM;
                    rateSpot %= RATE_SIZE;
                    int sum = 0;
                    for (byte i = 0; i < RATE_SIZE; i++) sum += rates[i];
                    heartRateBPM = sum / RATE_SIZE;
                }
            }
            if (redValue > 0 && irValue > 0) {
                float ratio = ((float)redValue / (float)irValue);
                int calcSpO2 = 110 - (int)(25.0f * ratio);
                if (calcSpO2 >= 88 && calcSpO2 <= 100) bloodOxygenSpO2 = calcSpO2;
                else if (calcSpO2 > 100) bloodOxygenSpO2 = 99;
            }
        }
    } else {
        fingerOnSensor = false;
        heartRateBPM = 0;
        bloodOxygenSpO2 = 0;
    }

    if (millis() - lastBeatTime > 2500) {
        beatDetected = false;
        heartRateBPM = 0;
    }
}

// ----------------------------------------------------------------------------
// Process MAX30100 FIFO Samples
// ----------------------------------------------------------------------------
void updateMAX30100Processing() {
    Wire.beginTransmission(MAX3010X_ADDR);
    Wire.write(0x05);
    if (Wire.endTransmission(false) == 0) {
        if (Wire.requestFrom((uint8_t)MAX3010X_ADDR, (uint8_t)4) == 4) {
            uint16_t irRaw = (Wire.read() << 8) | Wire.read();
            uint16_t redRaw = (Wire.read() << 8) | Wire.read();

            if (irRaw > 10000) {
                fingerOnSensor = true;
                if (checkForBeat(irRaw)) {
                    beatDetected = true;
                    unsigned long delta = millis() - lastBeatTime;
                    lastBeatTime = millis();
                    if (delta > 300 && delta < 1500) {
                        int instantBPM = 60000 / delta;
                        if (instantBPM >= 45 && instantBPM <= 200) {
                            heartRateBPM = (heartRateBPM == 0) ? instantBPM : (heartRateBPM * 3 + instantBPM) / 4;
                        }
                    }
                    if (redRaw > 0 && irRaw > 0) {
                        float r = (float)redRaw / (float)irRaw;
                        int s = 110 - (int)(25.0f * r);
                        if (s >= 88 && s <= 100) bloodOxygenSpO2 = s;
                        else if (s > 100) bloodOxygenSpO2 = 99;
                    }
                }
            } else {
                fingerOnSensor = false;
                heartRateBPM = 0;
                bloodOxygenSpO2 = 0;
            }
        }
    }

    if (millis() - lastBeatTime > 2500) {
        beatDetected = false;
        heartRateBPM = 0;
    }
}

// ----------------------------------------------------------------------------
// Process Analog Pulse Sensor (PA3)
// ----------------------------------------------------------------------------
void updateAnalogPulseProcessing() {
    rawPulseAnalog = analogRead(PIN_PULSE_ANALOG);
    pulseSignal = (pulseSignal * 7 + rawPulseAnalog) / 8;

    if (pulseSignal < pulseMin) pulseMin = pulseSignal;
    if (pulseSignal > pulseMax) pulseMax = pulseSignal;
    pulseMin += 1;
    pulseMax -= 1;
    if (pulseMax < pulseMin + 100) pulseMax = pulseMin + 100;
    int threshold = (pulseMin + pulseMax) / 2;

    unsigned long now = millis();
    if (pulseSignal > threshold + 30 && !beatDetected && (now - lastBeatTime > 300)) {
        beatDetected = true;
        unsigned long interval = now - lastBeatTime;
        lastBeatTime = now;
        if (interval > 300 && interval < 1500) {
            int instant = 60000 / interval;
            heartRateBPM = (heartRateBPM == 0) ? instant : (heartRateBPM * 3 + instant) / 4;
        }
    }

    if (pulseSignal < threshold - 20) beatDetected = false;
    if (now - lastBeatTime > 2500) heartRateBPM = 0;
}

// ----------------------------------------------------------------------------
// SETUP
// ----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    // 1. Diagnostic LED & Buzzer Outputs
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_BUZZER_ALT, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(PIN_BUZZER_ALT, LOW);

    // 2. Analog Inputs (12-bit ADC: 0 - 4095)
    pinMode(PIN_FSR_FOREFOOT, INPUT_ANALOG);
    pinMode(PIN_FSR_HEEL, INPUT_ANALOG);
    pinMode(PIN_PULSE_ANALOG, INPUT_ANALOG);
    analogReadResolution(12);

    // 3. I2C Bus Initialization (400 kHz Fast Mode)
    Wire.setSCL(PIN_I2C_SCL);
    Wire.setSDA(PIN_I2C_SDA);
    Wire.begin();
    Wire.setClock(400000);

    delay(150);

    // 4. Exhaustive I2C Bus Scanner across 0x01 to 0x77
    strcpy(i2cScanList, "");
    for (uint8_t a = 1; a < 127; a++) {
        if (probeI2C(a)) {
            char buf[8];
            snprintf(buf, sizeof(buf), "0x%02X ", a);
            strcat(i2cScanList, buf);
        }
    }
    Serial.print(F("I2C Scanner Found: "));
    Serial.println(i2cScanList);

    // 5. OLED Display Probe
    if (probeI2C(0x3C)) oledAddr = 0x3C;
    else if (probeI2C(0x3D)) oledAddr = 0x3D;

    if (display.begin(SSD1306_SWITCHCAPVCC, oledAddr)) {
        oledReady = true;
    }

    // 6. TMP117 Temperature Probe
    tmpFound = probeI2C(TMP117_ADDR);
    if (tmpFound) {
        readTMP117(currentTempC);
    }

    // 7. MPU-6050 Check & Initialization (Probing 0x68 and 0x69)
    if (probeI2C(0x68)) {
        mpuAddr = 0x68;
        mpuFound = initMPU6050Hardware(0x68);
    } else if (probeI2C(0x69)) {
        mpuAddr = 0x69;
        mpuFound = initMPU6050Hardware(0x69);
    } else {
        mpuAddr = 0;
        mpuFound = false;
    }

    // 8. Pulse Oximeter Detection (0x57)
    if (probeI2C(MAX3010X_ADDR)) {
        uint8_t partID = readI2CRegister(MAX3010X_ADDR, 0xFF);
        if (partID == 0x15) {
            if (max30102.begin(Wire, I2C_SPEED_FAST)) {
                max30102.setup();
                max30102.setPulseAmplitudeRed(0x1F);
                max30102.setPulseAmplitudeIR(0x1F);
                max30102.setPulseAmplitudeGreen(0);
                activeHRSensor = HR_SENSOR_MAX30102;
                hrSensorDetected = true;
            }
        } else if (partID == 0x11) {
            if (initMAX30100Hardware()) {
                activeHRSensor = HR_SENSOR_MAX30100;
                hrSensorDetected = true;
            }
        }
    }

    if (!hrSensorDetected) {
        activeHRSensor = HR_SENSOR_ANALOG;
    }

    // 9. Startup Diagnostic Display: Shows I2C Scan & MPU details for 1.8 seconds!
    if (oledReady) {
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);

        display.setCursor(4, 2);
        display.print(F("HARDWARE DIAGNOSTIC"));
        display.drawFastHLine(0, 12, 128, SSD1306_WHITE);

        // Line 1: I2C Scan
        display.setCursor(2, 16);
        display.print(F("I2C: "));
        display.print(i2cScanList);

        // Line 2: MPU-6050 Status
        display.setCursor(2, 28);
        display.print(F("MPU: "));
        if (mpuFound) {
            display.print(F("0x"));
            display.print(mpuAddr, HEX);
            display.print(F(" ID:0x"));
            display.print(mpuChipID, HEX);
            display.print(F(" OK"));
        } else {
            display.print(F("NOT FOUND (0x68/69)"));
        }

        // Line 3: TMP117 & HR Sensor
        display.setCursor(2, 40);
        display.print(F("TMP:"));
        display.print(tmpFound ? F("OK(0x48)") : F("NONE"));
        display.print(F(" HR:"));
        if (activeHRSensor == HR_SENSOR_MAX30102) display.print(F("30102"));
        else if (activeHRSensor == HR_SENSOR_MAX30100) display.print(F("30100"));
        else display.print(F("PA3"));

        // Line 4: Buzzer
        display.setCursor(2, 52);
        display.print(F("BUZZER: PB1 ACTIVE"));

        display.display();

        // 80ms startup chirp
        digitalWrite(PIN_BUZZER, HIGH);
        digitalWrite(PIN_BUZZER_ALT, HIGH);
        delay(80);
        digitalWrite(PIN_BUZZER, LOW);
        digitalWrite(PIN_BUZZER_ALT, LOW);
        delay(1600); // Allow user to clearly read the hardware diagnostic
    }
}

// ----------------------------------------------------------------------------
// MAIN LOOP
// ----------------------------------------------------------------------------
void loop() {
    unsigned long now = millis();

    // ========================================================================
    // Task 1: High-Speed Sensor Sampling (Every 20 ms / 50 Hz)
    // ========================================================================
    if (now - lastFastSample >= 20) {
        lastFastSample = now;

        // Sample Plantar Pressure (Forefoot & Heel)
        rawFSR1 = analogRead(PIN_FSR_FOREFOOT);
        rawFSR2 = analogRead(PIN_FSR_HEEL);
        voltFSR1 = (rawFSR1 * 3.3f) / 4095.0f;
        voltFSR2 = (rawFSR2 * 3.3f) / 4095.0f;

        // Check if BOTH pressure sensors are experiencing HIGH load (> 2000 counts)
        bothPressureHigh = (rawFSR1 >= FSR_HIGH_THRESHOLD && rawFSR2 >= FSR_HIGH_THRESHOLD);

        // Control Buzzer Alert: SOUND ALARM WHEN BOTH PRESSURE SENSORS ARE HIGH!
        if (bothPressureHigh) {
            digitalWrite(PIN_BUZZER, HIGH);
            digitalWrite(PIN_BUZZER_ALT, HIGH);
        } else {
            digitalWrite(PIN_BUZZER, LOW);
            digitalWrite(PIN_BUZZER_ALT, LOW);
        }

        // Process Heart Rate Sensor
        if (activeHRSensor == HR_SENSOR_MAX30102) {
            updateMAX30102Processing();
        } else if (activeHRSensor == HR_SENSOR_MAX30100) {
            updateMAX30100Processing();
        } else {
            updateAnalogPulseProcessing();
        }
    }

    // ========================================================================
    // Task 2: MPU-6050 Motion & Gait Sampling (Every 100 ms / 10 Hz)
    // ========================================================================
    if (now - lastImuSample >= 100) {
        lastImuSample = now;
        sampleMPU6050();
    }

    // ========================================================================
    // Task 3: Temperature Reading (Every 1000 ms, staggered to prevent collisions)
    // ========================================================================
    if (now - lastTempRead >= 1000) {
        lastTempRead = now;

        float t = 0.0f;
        if (readTMP117(t)) {
            tmpFound = true;
            currentTempC = t;
            tmpFailCount = 0;
        } else {
            tmpFailCount++;
            if (tmpFailCount >= 3) {
                recoverI2CBus();
                tmpFound = probeI2C(TMP117_ADDR);
                tmpFailCount = 0;
            }
        }
    }

    // ========================================================================
    // Task 4: OLED Screen Refresh (Every 200 ms / 5 Hz)
    // ========================================================================
    if (now - lastOledUpdate >= 200) {
        lastOledUpdate = now;
        frameCount++;

        if (oledReady) {
            display.clearDisplay();

            // --- Header Bar ---
            display.fillRect(0, 0, 128, 9, SSD1306_WHITE);
            display.setTextColor(SSD1306_BLACK);
            display.setTextSize(1);
            display.setCursor(2, 1);
            display.print(F("SOLEGARD MONITOR"));

            display.setCursor(105, 1);
            if (beatDetected) display.print(F("<3"));
            else display.print(F("  "));
            display.setCursor(120, 1);
            display.print((frameCount / 5) % 10);

            display.setTextColor(SSD1306_WHITE);

            // --- Row 1 (y: 11): TMP117 Temperature & Status ---
            display.setCursor(0, 11);
            display.print(F("TMP:"));
            if (tmpFound) {
                display.print(currentTempC, 1);
                display.drawCircle(58, 11, 1, SSD1306_WHITE);
                display.setCursor(62, 11);
                display.print(F("C"));
                display.setCursor(82, 11);
                if (currentTempC > 35.5f) display.print(F("[WARM]"));
                else display.print(F("[NORM]"));
            } else {
                display.print(F("TMP117 N/C"));
            }

            // --- Row 2 (y: 20): Heart Rate & SpO2 ---
            display.setCursor(0, 20);
            if (activeHRSensor == HR_SENSOR_MAX30102 || activeHRSensor == HR_SENSOR_MAX30100) {
                if (fingerOnSensor) {
                    display.print(F("HR:"));
                    if (heartRateBPM > 0) display.print(heartRateBPM);
                    else display.print(F("--"));
                    display.print(F("bpm"));

                    display.setCursor(76, 20);
                    display.print(F("O2:"));
                    if (bloodOxygenSpO2 > 0) {
                        display.print(bloodOxygenSpO2);
                        display.print(F("%"));
                    } else display.print(F("--%"));
                } else {
                    display.print(F("HR:[Place Finger]"));
                }
            } else {
                display.print(F("HR:"));
                if (heartRateBPM > 40 && heartRateBPM < 200) {
                    display.print(heartRateBPM);
                    display.print(F(" BPM"));
                } else {
                    display.print(F("RAW "));
                    display.print(rawPulseAnalog);
                }

                int pulseBar = ((rawPulseAnalog - 1500) * 36) / 2500;
                if (pulseBar < 0) pulseBar = 0;
                if (pulseBar > 36) pulseBar = 36;
                display.drawRect(88, 20, 38, 7, SSD1306_WHITE);
                if (pulseBar > 0) display.fillRect(89, 21, pulseBar, 5, SSD1306_WHITE);
            }

            // --- Row 3 (y: 29): MPU-6050 Motion / Tilt / Gait ---
            display.setCursor(0, 29);
            display.print(F("IMU:"));
            if (mpuFound) {
                if (isGaitWalking) {
                    display.print(F("WALK ("));
                    display.print(totalAccelMag, 1);
                    display.print(F("m/s)"));
                } else {
                    // Show live Pitch and Roll angles in degrees
                    display.print(F("P:"));
                    display.print(pitchAngle);
                    display.print(F(" R:"));
                    display.print(rollAngle);
                    display.drawCircle(64, 29, 1, SSD1306_WHITE);
                    display.setCursor(94, 29);
                    display.print(F("[STAT]"));
                }
            } else {
                display.print(F("N/C:"));
                display.print(i2cScanList); // Shows exactly what was found on the bus!
            }

            display.drawFastHLine(0, 38, 128, SSD1306_WHITE);

            // --- Row 4 (y: 40): Forefoot Pressure Sensor (PA1) ---
            display.setCursor(0, 40);
            display.print(F("FF:"));
            display.print(rawFSR1);
            display.setCursor(44, 40);
            display.print(voltFSR1, 1);
            display.print(F("V"));

            display.drawRect(78, 40, 49, 6, SSD1306_WHITE);
            int bar1 = (rawFSR1 * 45) / 4095;
            if (bar1 > 45) bar1 = 45;
            if (bar1 > 0) display.fillRect(80, 41, bar1, 4, SSD1306_WHITE);

            // --- Row 5 (y: 48): Heel Pressure Sensor (PA2) ---
            display.setCursor(0, 48);
            display.print(F("HL:"));
            display.print(rawFSR2);
            display.setCursor(44, 48);
            display.print(voltFSR2, 1);
            display.print(F("V"));

            display.drawRect(78, 48, 49, 6, SSD1306_WHITE);
            int bar2 = (rawFSR2 * 45) / 4095;
            if (bar2 > 45) bar2 = 45;
            if (bar2 > 0) display.fillRect(80, 49, bar2, 4, SSD1306_WHITE);

            // --- Row 6 (y: 56): Alarm Banner / Footer ---
            if (bothPressureHigh) {
                // Inverted flashing alarm banner
                display.fillRect(0, 56, 128, 8, SSD1306_WHITE);
                display.setTextColor(SSD1306_BLACK);
                display.setCursor(2, 56);
                display.print(F("! DUAL HIGH: BUZZ ON !"));
            } else if (rawFSR1 >= FSR_HIGH_THRESHOLD || rawFSR2 >= FSR_HIGH_THRESHOLD) {
                display.setCursor(0, 56);
                display.print(F("ALERT: SINGLE OVERLOAD"));
            } else {
                display.setCursor(0, 56);
                display.print(F("STATUS: ALL NORMAL"));
            }

            display.display();
        }
    }

    // ========================================================================
    // Task 5: MCU Heartbeat LED (Every 500 ms) & Serial Telemetry
    // ========================================================================
    if (now - lastLedToggle >= 500) {
        lastLedToggle = now;
        digitalWrite(PIN_LED, !digitalRead(PIN_LED));

        Serial.print(F("T:"));
        Serial.print(currentTempC, 1);
        Serial.print(F("C | FF:"));
        Serial.print(rawFSR1);
        Serial.print(F(" | HL:"));
        Serial.print(rawFSR2);
        Serial.print(F(" | BUZZ:"));
        Serial.print(bothPressureHigh ? F("ON") : F("OFF"));
        Serial.print(F(" | IMU: P:"));
        Serial.print(pitchAngle);
        Serial.print(F(" R:"));
        Serial.print(rollAngle);
        Serial.print(F(" ("));
        Serial.print(isGaitWalking ? F("WALK") : F("STAT"));
        Serial.print(F(") | HR:"));
        Serial.print(heartRateBPM);
        Serial.println();
    }
}

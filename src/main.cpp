/**
 * ============================================================================
 * SoleGuard - Full Multi-Sensor Medical Monitoring System
 * With High-Sensitivity MAX30102 Pulse Engine & Live Numerical Dashboard
 * ============================================================================
 */

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <MAX30105.h>
#include <Wire.h>
#include <heartRate.h>
#include <math.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// Pins
#define PIN_I2C1_SCL PB6
#define PIN_I2C1_SDA PB7
#define PIN_I2C2_SCL PB10
#define PIN_I2C2_SDA PB3
#define PIN_FSR_FOREFOOT PA1
#define PIN_FSR_HEEL PA2
#define PIN_PULSE_ANA PA3
#define PIN_BUZZER        PB1   // Audio Alert Buzzer (+)
#define PIN_VIB_MOTOR     PB0   // Haptic Vibration Motor (SIG / Driver Base)
#define PIN_LED           PC13

#define FSR_THRESHOLD 2000

// Hardware I2C2 Instance
TwoWire Wire2(PIN_I2C2_SDA, PIN_I2C2_SCL);

// MAX30105 Sensor Instance
MAX30105 max30102;

// OLED Displays
Adafruit_SSD1306 disp1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET,
                       100000UL, 100000UL); // PB10/PB3
Adafruit_SSD1306 disp2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 100000UL,
                       100000UL); // PB6/PB7

// Live Status in RAM
volatile unsigned long frameCount = 0;
volatile uint8_t oled1_online = 0;
volatile uint8_t oled2_online = 0;

// Pressure Sensors (FSR 402 on PA1 & PA2) & Buzzer Alarm (PB1)
volatile int rawFSR1 = 0; // Forefoot (PA1)
volatile int rawFSR2 = 0; // Heel (PA2)
volatile float voltFSR1 = 0.0f;
volatile float voltFSR2 = 0.0f;
volatile uint8_t bothPressureHigh = 0;

// Heart Rate Sensor (MAX30102 at 0x57)
volatile uint8_t hr_online = 0;
volatile uint8_t hr_bus_num = 2; // Default on Wire2 (PB10/PB3)
volatile uint8_t hr_part_id = 0x15;
volatile uint32_t hr_red_val = 0;
volatile uint32_t hr_ir_val = 0;
volatile uint8_t finger_detected = 0;
volatile uint8_t beat_detected = 0;
volatile int heartRateBPM = 0;
volatile int rawPulseAnalog = 0;

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE] = {72, 75, 74, 76};
byte rateSpot = 0;
unsigned long last_beat_time = 0;
TwoWire *hr_wire_ptr = &Wire2;

// TMP117 Live Sensor Data
volatile uint8_t tmp_online = 0;
volatile uint8_t tmp_bus_num = 1; // Default on Wire (PB6/PB7)
volatile uint8_t tmp_addr = 0x48;
volatile float tmp_celsius = 29.3f;
volatile float tmp_fahrenheit = 84.7f;
volatile int16_t tmp_raw = 0;
TwoWire *tmp_wire_ptr = &Wire;

// MPU-6050 Live Sensor Data
volatile uint8_t mpu_online = 0;
volatile uint8_t mpu_bus_num = 1;
volatile uint8_t mpu_addr = 0x68;
volatile uint8_t mpu_chip_id = 0;

volatile float mpu_ax = 0.0f;
volatile float mpu_ay = 0.0f;
volatile float mpu_az = 9.8f;
volatile float mpu_gx = 0.0f;
volatile float mpu_gy = 0.0f;
volatile float mpu_gz = 0.0f;
volatile float mpu_temp = 25.0f;
volatile float mpu_mag = 9.8f;
volatile int16_t mpu_pitch = 0;
volatile int16_t mpu_roll = 0;
volatile uint8_t mpu_motion_detected = 0;
TwoWire *mpu_wire_ptr = &Wire;

char telemetryBuffer[96] = "";

// Diagnostic Bus State in RAM
volatile uint8_t diag_w1_devs[8] = {0};
volatile uint8_t diag_w2_devs[8] = {0};
volatile uint8_t diag_probe_w1_57 = 0xFF;
volatile uint8_t diag_probe_w2_57 = 0xFF;
volatile uint8_t diag_partid_w1 = 0xFF;
volatile uint8_t diag_partid_w2 = 0xFF;
volatile uint8_t diag_begin_w1 = 0xFF;
volatile uint8_t diag_begin_w2 = 0xFF;
volatile uint8_t w2_err_3c = 0xFF;
volatile uint8_t w2_err_57 = 0xFF;
volatile uint8_t w1_err_3c = 0xFF;
volatile uint8_t w1_err_48 = 0xFF;
volatile uint8_t w1_err_57 = 0xFF;

void recoverI2CBus(pin_size_t scl, pin_size_t sda) {
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, OUTPUT);
  digitalWrite(scl, HIGH);
  delayMicroseconds(10);
  for (int i = 0; i < 16; i++) {
    digitalWrite(scl, LOW);
    delayMicroseconds(5);
    digitalWrite(scl, HIGH);
    delayMicroseconds(5);
  }
  pinMode(sda, OUTPUT);
  digitalWrite(sda, LOW);
  delayMicroseconds(5);
  digitalWrite(scl, HIGH);
  delayMicroseconds(5);
  digitalWrite(sda, HIGH);
  delayMicroseconds(5);
  pinMode(scl, INPUT_PULLUP);
  pinMode(sda, INPUT_PULLUP);
  delayMicroseconds(10);
}

uint8_t probeBus(TwoWire &bus, uint8_t addr) {
  bus.beginTransmission(addr);
  return bus.endTransmission(true);
}

uint8_t readRegister(TwoWire &bus, uint8_t devAddr, uint8_t regAddr) {
  bus.beginTransmission(devAddr);
  bus.write(regAddr);
  if (bus.endTransmission(true) != 0)
    return 0xFF;
  if (bus.requestFrom((uint8_t)devAddr, (uint8_t)1) == 1) {
    return bus.read();
  }
  return 0xFF;
}

bool writeRegister(TwoWire &bus, uint8_t devAddr, uint8_t regAddr,
                   uint8_t data) {
  bus.beginTransmission(devAddr);
  bus.write(regAddr);
  bus.write(data);
  return (bus.endTransmission(true) == 0);
}

// ----------------------------------------------------------------------------
// MAX30102 High-Sensitivity Driver & Pulse Engine
// ----------------------------------------------------------------------------
static float dcFilterIR = 0.0f;
static float prevAC = 0.0f;
static bool pulseArmed = true;

bool initMAX30102Driver() {
  diag_probe_w1_57 = probeBus(Wire, 0x57);
  diag_probe_w2_57 = probeBus(Wire2, 0x57);
  diag_partid_w1 = readRegister(Wire, 0x57, 0xFF);
  diag_partid_w2 = readRegister(Wire2, 0x57, 0xFF);

  // Try Wire2 (PB10/PB3) first
  if (diag_probe_w2_57 == 0) {
    diag_begin_w2 = max30102.begin(Wire2, I2C_SPEED_STANDARD) ? 1 : 0;
    if (diag_begin_w2) {
      hr_online = 1;
      hr_bus_num = 2;
      hr_wire_ptr = &Wire2;
    }
  }
  // If not online, try Wire (PB6/PB7)
  if (!hr_online && diag_probe_w1_57 == 0) {
    diag_begin_w1 = max30102.begin(Wire, I2C_SPEED_STANDARD) ? 1 : 0;
    if (diag_begin_w1) {
      hr_online = 1;
      hr_bus_num = 1;
      hr_wire_ptr = &Wire;
    }
  }

  // Fallback: If probe ACKed (0x57 exists) but library begin check failed,
  // force setup
  if (!hr_online && (diag_probe_w2_57 == 0 || diag_probe_w1_57 == 0)) {
    TwoWire &bus = (diag_probe_w2_57 == 0) ? Wire2 : Wire;
    hr_bus_num = (diag_probe_w2_57 == 0) ? 2 : 1;
    hr_wire_ptr = &bus;
    hr_online = 1;
    max30102.begin(bus, I2C_SPEED_STANDARD);
  }

  if (hr_online) {
    max30102.setup(0x1F, 4, 2, 400, 411, 4096);
    max30102.setPulseAmplitudeRed(0x0A);
    max30102.setPulseAmplitudeGreen(0);
    delay(20);
    return true;
  }
  return false;
}

void readMAX30102Sample() {
  if (!hr_online)
    return;

  max30102.check(); // Pull all available FIFO samples

  while (max30102.available()) {
    uint32_t ir = max30102.getFIFOIR();
    uint32_t red = max30102.getFIFORed();
    max30102.nextSample();

    hr_ir_val = ir;
    hr_red_val = red;

    // When finger is touching, IR is > 20,000 (ambient noise is < 15,000)
    if (ir > 20000) {
      finger_detected = 1;

      // DC Baseline tracking (High-pass filter)
      if (dcFilterIR == 0.0f)
        dcFilterIR = (float)ir;
      dcFilterIR = 0.95f * dcFilterIR + 0.05f * (float)ir;
      float acSignal = (float)ir - dcFilterIR;

      // Dual beat detection: SparkFun PBA algorithm OR AC Zero-Crossing Peak
      // detector
      bool beat = checkForBeat(ir);

      unsigned long now = millis();
      if (acSignal > 40.0f && prevAC <= 40.0f && pulseArmed &&
          (now - last_beat_time > 320)) {
        beat = true;
        pulseArmed = false;
      }
      if (acSignal < -25.0f) {
        pulseArmed = true; // Rearm detector after wave trough
      }
      prevAC = acSignal;

      if (beat) {
        beat_detected = 1;
        unsigned long delta = now - last_beat_time;
        last_beat_time = now;

        if (delta > 320 && delta < 1500) {
          float instantBPM = 60000.0f / (float)delta;
          if (instantBPM >= 45.0f && instantBPM <= 190.0f) {
            rates[rateSpot++] = (byte)instantBPM;
            rateSpot %= RATE_SIZE;

            int sum = 0;
            for (byte k = 0; k < RATE_SIZE; k++)
              sum += rates[k];
            heartRateBPM = sum / RATE_SIZE;
          }
        }
      }
    } else {
      finger_detected = 0;
      dcFilterIR = 0.0f;
    }
  }

  if (millis() - last_beat_time > 1800) {
    beat_detected = 0;
  }
}

// ----------------------------------------------------------------------------
// TMP117 High Precision Temperature Driver
// ----------------------------------------------------------------------------
void readTMP117Sample() {
  if (!tmp_online || tmp_wire_ptr == nullptr)
    return;

  TwoWire &bus = *tmp_wire_ptr;
  bus.beginTransmission(tmp_addr);
  bus.write(0x00);
  if (bus.endTransmission(false) != 0)
    return;

  if (bus.requestFrom((uint8_t)tmp_addr, (uint8_t)2) == 2) {
    uint8_t msb = bus.read();
    uint8_t lsb = bus.read();
    tmp_raw = (int16_t)((msb << 8) | lsb);
    tmp_celsius = (float)tmp_raw * 0.0078125f;
    tmp_fahrenheit = tmp_celsius * 1.8f + 32.0f;
  }
}

// ----------------------------------------------------------------------------
// MPU-6050 Driver
// ----------------------------------------------------------------------------
bool initMPU(TwoWire &bus, uint8_t addr, uint8_t busNum) {
  mpu_chip_id = readRegister(bus, addr, 0x75);
  writeRegister(bus, addr, 0x6B, 0x00);
  delay(15);
  writeRegister(bus, addr, 0x19, 0x07);
  writeRegister(bus, addr, 0x1A, 0x04);
  writeRegister(bus, addr, 0x1B, 0x08);
  writeRegister(bus, addr, 0x1C, 0x08);

  mpu_online = 1;
  mpu_bus_num = busNum;
  mpu_addr = addr;
  mpu_wire_ptr = &bus;
  return true;
}

void autoDetectMPU() {
  if (mpu_online)
    return;

  if (probeBus(Wire, 0x68) == 0) {
    initMPU(Wire, 0x68, 1);
  } else if (probeBus(Wire, 0x69) == 0) {
    initMPU(Wire, 0x69, 1);
  } else if (probeBus(Wire2, 0x68) == 0) {
    initMPU(Wire2, 0x68, 2);
  } else if (probeBus(Wire2, 0x69) == 0) {
    initMPU(Wire2, 0x69, 2);
  }
}

void readMPUSample() {
  if (!mpu_online || mpu_wire_ptr == nullptr)
    return;

  TwoWire &bus = *mpu_wire_ptr;
  bus.beginTransmission(mpu_addr);
  bus.write(0x3B);
  if (bus.endTransmission(false) != 0)
    return;

  if (bus.requestFrom((uint8_t)mpu_addr, (uint8_t)14) == 14) {
    int16_t rawAX = (bus.read() << 8) | bus.read();
    int16_t rawAY = (bus.read() << 8) | bus.read();
    int16_t rawAZ = (bus.read() << 8) | bus.read();
    int16_t rawT = (bus.read() << 8) | bus.read();
    int16_t rawGX = (bus.read() << 8) | bus.read();
    int16_t rawGY = (bus.read() << 8) | bus.read();
    int16_t rawGZ = (bus.read() << 8) | bus.read();

    mpu_ax = ((float)rawAX / 8192.0f) * 9.80665f;
    mpu_ay = ((float)rawAY / 8192.0f) * 9.80665f;
    mpu_az = ((float)rawAZ / 8192.0f) * 9.80665f;

    mpu_gx = (float)rawGX / 65.5f;
    mpu_gy = (float)rawGY / 65.5f;
    mpu_gz = (float)rawGZ / 65.5f;

    mpu_temp = ((float)rawT / 340.0f) + 36.53f;
    mpu_mag = sqrt(mpu_ax * mpu_ax + mpu_ay * mpu_ay + mpu_az * mpu_az);

    mpu_pitch =
        (int16_t)(atan2(-mpu_ax, sqrt(mpu_ay * mpu_ay + mpu_az * mpu_az)) *
                  57.2958f);
    mpu_roll = (int16_t)(atan2(mpu_ay, mpu_az) * 57.2958f);

    float dynA = abs(mpu_mag - 9.8f);
    float dynG = abs(mpu_gx) + abs(mpu_gy) + abs(mpu_gz);
    mpu_motion_detected = (dynA > 1.5f || dynG > 30.0f) ? 1 : 0;
  }
}

// ----------------------------------------------------------------------------
// Display 1: Primary Medical Dashboard (Wire2 on PB10/PB3)
// ----------------------------------------------------------------------------
void drawDisplay1() {
  disp1.clearDisplay();

  // Top Header
  disp1.fillRect(0, 0, 128, 13, SSD1306_WHITE);
  disp1.setTextColor(SSD1306_BLACK);
  disp1.setTextSize(1);
  disp1.setCursor(6, 3);
  disp1.print(F("SOLEGUARD MEDICAL HUB"));

  disp1.setTextColor(SSD1306_WHITE);

  // Temperature Row
  disp1.setCursor(4, 16);
  disp1.print(F("TEMP : "));
  disp1.setTextSize(2);
  disp1.print(tmp_celsius, 1);
  disp1.print(F(" C"));
  disp1.setTextSize(1);
  disp1.print(F(" ("));
  disp1.print((int)tmp_fahrenheit);
  disp1.print(F("F)"));

  // Heart Rate Row
  disp1.setCursor(4, 34);
  disp1.print(F("HEART: "));
  disp1.setTextSize(2);
  if (!hr_online && rawPulseAnalog < 1200) {
    disp1.setTextSize(1);
    disp1.setCursor(46, 38);
    disp1.print(F("NO SENSOR"));
    disp1.setTextSize(2);
  } else if (finger_detected && heartRateBPM > 0) {
    disp1.print(heartRateBPM);
    disp1.print(F(" BPM"));
  } else if (finger_detected) {
    disp1.print(F("PULSE.."));
  } else {
    disp1.print(F("-- BPM"));
  }
  disp1.setTextSize(1);

  // Live Optical Sensor Stats (ALWAYS VISIBLE!)
  disp1.setCursor(4, 53);
  disp1.print(F("IR:"));
  disp1.print(hr_ir_val);
  disp1.print(F(" | "));
  if (beat_detected) {
    disp1.fillRect(80, 52, 44, 10, SSD1306_WHITE);
    disp1.setTextColor(SSD1306_BLACK);
    disp1.setCursor(82, 53);
    disp1.print(F("BEAT !"));
    disp1.setTextColor(SSD1306_WHITE);
  } else if (finger_detected) {
    disp1.print(F("[FINGER]"));
  } else {
    disp1.print(F("[TOUCH]"));
  }

  disp1.display();
}

// ----------------------------------------------------------------------------
// Display 2: Sensor Telemetry & Diagnostics (Wire on PB6/PB7)
// ----------------------------------------------------------------------------
void drawDisplay2() {
  disp2.clearDisplay();

  // Top Header with live rotating spinner & frame counter
  disp2.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  disp2.setTextColor(SSD1306_BLACK);
  disp2.setTextSize(1);
  disp2.setCursor(4, 2);
  disp2.print(F("SOLEGUARD ["));
  const char spinner[] = {'|', '/', '-', '\\'};
  disp2.print(spinner[(frameCount / 4) % 4]);
  disp2.print(F("] #"));
  disp2.print(frameCount % 100);

  disp2.setTextColor(SSD1306_WHITE);

  // Row 1: TMP117 Temperature & Heart Rate
  disp2.setCursor(2, 14);
  disp2.print(F("TMP:"));
  disp2.print(tmp_celsius, 1);
  disp2.print(F("C "));
  disp2.print(F("HR:"));
  if (finger_detected && heartRateBPM > 0) {
    disp2.print(heartRateBPM);
    disp2.print(F("b"));
  } else {
    disp2.print(F("--b"));
  }

  // Row 2: Forefoot Pressure Sensor (PA1)
  disp2.setCursor(2, 26);
  disp2.print(F("FF:"));
  disp2.print(rawFSR1);
  disp2.drawRect(58, 26, 68, 7, SSD1306_WHITE);
  int bar1 = (rawFSR1 * 64) / 4095;
  if (bar1 > 64)
    bar1 = 64;
  if (bar1 > 0)
    disp2.fillRect(60, 28, bar1, 3, SSD1306_WHITE);

  // Row 3: Heel Pressure Sensor (PA2)
  disp2.setCursor(2, 38);
  disp2.print(F("HL:"));
  disp2.print(rawFSR2);
  disp2.drawRect(58, 38, 68, 7, SSD1306_WHITE);
  int bar2 = (rawFSR2 * 64) / 4095;
  if (bar2 > 64)
    bar2 = 64;
  if (bar2 > 0)
    disp2.fillRect(60, 40, bar2, 3, SSD1306_WHITE);

  // Row 4: Buzzer & Vibration Alert Banner
  if (bothPressureHigh) {
    disp2.fillRect(0, 49, 128, 15, SSD1306_WHITE);
    disp2.setTextColor(SSD1306_BLACK);
    disp2.setCursor(2, 52);
    disp2.print(F("! BUZZ + VIB ACTIVE !"));
  } else if (rawFSR1 >= FSR_THRESHOLD || rawFSR2 >= FSR_THRESHOLD) {
    disp2.setCursor(2, 52);
    disp2.print(F("ALERT: VIB PULSE (SINGLE)"));
  } else {
    disp2.setCursor(2, 52);
    disp2.print(F("BUZZ/VIB: OFF (NORMAL)"));
  }

  disp2.display();
}

// ----------------------------------------------------------------------------
// Setup & Loop
// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  pinMode(PIN_I2C1_SCL, INPUT_PULLUP);
  pinMode(PIN_I2C1_SDA, INPUT_PULLUP);
  pinMode(PIN_I2C2_SCL, INPUT_PULLUP);
  pinMode(PIN_I2C2_SDA, INPUT_PULLUP);
  pinMode(PIN_PULSE_ANA, INPUT_ANALOG);
  pinMode(PIN_FSR_FOREFOOT, INPUT_ANALOG);
  pinMode(PIN_FSR_HEEL, INPUT_ANALOG);

  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_VIB_MOTOR, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_VIB_MOTOR, LOW);
  delay(50);

  recoverI2CBus(PIN_I2C1_SCL, PIN_I2C1_SDA);
  recoverI2CBus(PIN_I2C2_SCL, PIN_I2C2_SDA);

  Wire.setSCL(PIN_I2C1_SCL);
  Wire.setSDA(PIN_I2C1_SDA);
  Wire.begin();
  Wire.setClock(100000);

  Wire2.setSCL(PIN_I2C2_SCL);
  Wire2.setSDA(PIN_I2C2_SDA);
  Wire2.begin();
  Wire2.setClock(100000);
  delay(100);

  // Initial bus scan
  uint8_t c1 = 0, c2 = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (probeBus(Wire, a) == 0 && c1 < 8)
      diag_w1_devs[c1++] = a;
    if (probeBus(Wire2, a) == 0 && c2 < 8)
      diag_w2_devs[c2++] = a;
  }

  w2_err_3c = probeBus(Wire2, 0x3C);
  w2_err_57 = probeBus(Wire2, 0x57);
  w1_err_3c = probeBus(Wire, 0x3C);
  w1_err_48 = probeBus(Wire, 0x48);
  w1_err_57 = probeBus(Wire, 0x57);

  // OLED 1
  if (probeBus(Wire2, 0x3C) == 0) {
    if (disp1.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) {
      oled1_online = 1;
      disp1.ssd1306_command(SSD1306_SETCONTRAST);
      disp1.ssd1306_command(0xFF);
      disp1.clearDisplay();
      disp1.display();
    }
  }

  // OLED 2
  if (probeBus(Wire, 0x3C) == 0) {
    if (disp2.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) {
      oled2_online = 1;
      disp2.ssd1306_command(SSD1306_SETCONTRAST);
      disp2.ssd1306_command(0xFF);
      disp2.clearDisplay();
      disp2.display();
    }
  }

  // TMP117
  if (probeBus(Wire, 0x48) == 0) {
    tmp_online = 1;
    tmp_bus_num = 1;
    tmp_wire_ptr = &Wire;
  } else if (probeBus(Wire2, 0x48) == 0) {
    tmp_online = 1;
    tmp_bus_num = 2;
    tmp_wire_ptr = &Wire2;
  }

  // MAX30102
  initMAX30102Driver();

  autoDetectMPU();
}

void loop() {
  frameCount++;

  digitalWrite(PIN_LED, (frameCount % 6 < 3) ? LOW : HIGH);

  // Read TMP117 every 5 frames (~125ms)
  if (tmp_online && (frameCount % 5 == 0)) {
    readTMP117Sample();
  }

  // Read MAX30102 on every frame (40 Hz optical sampling)
  if (hr_online) {
    readMAX30102Sample();
  }

  // Read Plantar Pressure Sensors (PA1 = Forefoot, PA2 = Heel)
  rawFSR1 = analogRead(PIN_FSR_FOREFOOT);
  rawFSR2 = analogRead(PIN_FSR_HEEL);
  voltFSR1 = (float)rawFSR1 * 3.3f / 4095.0f;
  voltFSR2 = (float)rawFSR2 * 3.3f / 4095.0f;

  // Dual High Pressure Alarm Condition (> 2000 counts on BOTH sensors)
  bothPressureHigh =
      (rawFSR1 >= FSR_THRESHOLD && rawFSR2 >= FSR_THRESHOLD) ? 1 : 0;
  bool singleHigh = (rawFSR1 >= FSR_THRESHOLD || rawFSR2 >= FSR_THRESHOLD);

  // PB1: Audio Buzzer (Sounds when BOTH pressure sensors are high)
  digitalWrite(PIN_BUZZER, bothPressureHigh ? HIGH : LOW);

  // PB0: Vibration Motor (Continuous when BOTH high, Pulsed haptic tap when SINGLE high)
  if (bothPressureHigh) {
    digitalWrite(PIN_VIB_MOTOR, HIGH);
  } else if (singleHigh && (frameCount % 6 < 3)) {
    digitalWrite(PIN_VIB_MOTOR, HIGH);
  } else {
    digitalWrite(PIN_VIB_MOTOR, LOW);
  }

  // Also read analog pulse on PA3
  rawPulseAnalog = analogRead(PIN_PULSE_ANA);
  if (!finger_detected && rawPulseAnalog > 1200) {
    static int prevAnalog = 0;
    static unsigned long lastAnalogBeat = 0;
    unsigned long now = millis();
    if (rawPulseAnalog > 2100 && prevAnalog <= 2100 &&
        (now - lastAnalogBeat > 320)) {
      unsigned long delta = now - lastAnalogBeat;
      lastAnalogBeat = now;
      beat_detected = 1;
      if (delta > 320 && delta < 1500) {
        float instantBPM = 60000.0f / (float)delta;
        if (instantBPM >= 45.0f && instantBPM <= 190.0f) {
          rates[rateSpot++] = (byte)instantBPM;
          rateSpot %= RATE_SIZE;
          int sum = 0;
          for (byte k = 0; k < RATE_SIZE; k++)
            sum += rates[k];
          heartRateBPM = sum / RATE_SIZE;
        }
      }
    }
    prevAnalog = rawPulseAnalog;
  }

  // Dynamic hot-plug sensor auto-detection every 40 frames (~1.0s)
  if (frameCount % 40 == 0) {
    if (!hr_online) {
      initMAX30102Driver();
    }
    if (!oled1_online && probeBus(Wire2, 0x3C) == 0) {
      if (disp1.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) {
        oled1_online = 1;
        disp1.ssd1306_command(SSD1306_SETCONTRAST);
        disp1.ssd1306_command(0xFF);
        disp1.clearDisplay();
        disp1.display();
      }
    }
    autoDetectMPU();
  }

  // Render Displays
  if (oled1_online) {
    drawDisplay1();
  }
  if (oled2_online) {
    drawDisplay2();
  }

  // Format Telemetry in RAM buffer
  int t_int = (int)tmp_celsius;
  int t_frac = (int)(abs(tmp_celsius - t_int) * 100);
  snprintf(telemetryBuffer, sizeof(telemetryBuffer),
           "TMP:%d.%02dC | FF:%d HL:%d BUZ:%d | HR:%dBPM (IR:%lu) | N:%lu",
           t_int, t_frac, rawFSR1, rawFSR2, bothPressureHigh, heartRateBPM,
           hr_ir_val, frameCount);

  // Stream live telemetry over Serial (USB CDC) & Serial1 (USART1)
  if (frameCount % 4 == 0) {
    Serial.println(telemetryBuffer);
    Serial1.println(telemetryBuffer);
  }

  delay(25);
}

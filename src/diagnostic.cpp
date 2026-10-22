#include <Arduino.h>
#include <Wire.h>

// --- PIN ASSIGNMENTS ---
#define PIN_SDA 5
#define PIN_SCL 6

// Left Driver (DRV8833)
#define L_IN1 0
#define L_IN2 1
#define L_IN3 3
#define L_IN4 4

// Right Driver (DRV8833)
#define R_IN1 7
#define R_IN2 8
#define R_IN3 20
#define R_IN4 21

// --- MPU6050 REGISTERS ---
#define MPU_ADDR 0x68
#define REG_PWR_MGMT_1 0x6B
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_WHO_AM_I 0x75

// PWM Configuration
const int PWM_FREQ = 20000; // 20kHz: silent, smooth for coreless motors
const int PWM_RES = 10;     // 10-bit: 0 - 1023
const int TEST_DUTY = 200;  // ~20% power for safe testing

struct MotorChannel {
    const char* name;
    int pinA;
    int pinB;
    int ledcChan;
    bool reversed;
};

MotorChannel motors[4] = {
    {"M1 (Left IN1/IN2)",   L_IN1, L_IN2, 0, false},
    {"M2 (Right IN3/IN4)",  R_IN3, R_IN4, 1, false},
    {"M3 (Right IN1/IN2)",  R_IN1, R_IN2, 2, false},
    {"M4 (Left IN3/IN4)",   L_IN3, L_IN4, 3, false}
};

// Gyro calibration offsets
float gx_offset = 0, gy_offset = 0, gz_offset = 0;
float roll = 0, pitch = 0;
unsigned long last_time = 0;
bool stream_sensor = false;

// Hardware LEDC helper (compatible with Arduino-ESP32 v2 & v3)
void setupChannel(int chan, int pin) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttach(pin, PWM_FREQ, PWM_RES);
#else
    ledcSetup(chan, PWM_FREQ, PWM_RES);
    ledcAttachPin(pin, chan);
#endif
}

void writeChannel(int chan, int pin, int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(pin, duty);
#else
    ledcWrite(chan, duty);
#endif
}

void stopMotor(int idx) {
    MotorChannel &m = motors[idx];
    writeChannel(m.ledcChan, m.reversed ? m.pinB : m.pinA, 0);
    digitalWrite(m.pinA, LOW);
    digitalWrite(m.pinB, LOW);
}

void applyMotor(int idx, int duty) {
    MotorChannel &m = motors[idx];
    int pwmPin = m.reversed ? m.pinB : m.pinA;
    int lowPin = m.reversed ? m.pinA : m.pinB;
    digitalWrite(lowPin, LOW);
    writeChannel(m.ledcChan, pwmPin, duty);
}

void initMotors() {
    for (int i = 0; i < 4; i++) {
        pinMode(motors[i].pinA, OUTPUT);
        pinMode(motors[i].pinB, OUTPUT);
        digitalWrite(motors[i].pinA, LOW);
        digitalWrite(motors[i].pinB, LOW);
        setupChannel(motors[i].ledcChan, motors[i].pinA);
        stopMotor(i);
    }
}

// I2C MPU6050 minimal driver
bool initMPU() {
    Wire.begin(PIN_SDA, PIN_SCL, 400000);

    // Check WHO_AM_I
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_WHO_AM_I);
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
    if (!Wire.available()) return false;
    uint8_t who = Wire.read();
    Serial.printf("[MPU6050] WHO_AM_I = 0x%02X ", who);
    if (who == 0x68 || who == 0x70 || who == 0x72) {
        Serial.println("(Device recognized)");
    } else {
        Serial.println("(Unknown chip ID, continuing anyway)");
    }

    // Wake up (PWR_MGMT_1 = 0)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(10);

    // DLPF ~42Hz (0x03)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_CONFIG);
    Wire.write(0x03);
    Wire.endTransmission();

    // Gyro +/- 500 deg/s (0x08)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_GYRO_CONFIG);
    Wire.write(0x08);
    Wire.endTransmission();

    // Accel +/- 4g (0x08)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_CONFIG);
    Wire.write(0x08);
    Wire.endTransmission();

    return true;
}

void calibrateGyro() {
    Serial.println("\n[CALIBRATION] Keep drone completely still and flat...");
    float sx = 0, sy = 0, sz = 0;
    const int SAMPLES = 200;

    for (int i = 0; i < SAMPLES; i++) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(REG_ACCEL_XOUT_H + 8); // Gyro X H
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6);
        if (Wire.available() >= 6) {
            int16_t raw_gx = (Wire.read() << 8) | Wire.read();
            int16_t raw_gy = (Wire.read() << 8) | Wire.read();
            int16_t raw_gz = (Wire.read() << 8) | Wire.read();
            sx += raw_gx / 65.5f;
            sy += raw_gy / 65.5f;
            sz += raw_gz / 65.5f;
        }
        delay(5);
    }
    gx_offset = sx / SAMPLES;
    gy_offset = sy / SAMPLES;
    gz_offset = sz / SAMPLES;
    Serial.printf("[CALIBRATION] Done. Offsets: X=%.2f Y=%.2f Z=%.2f deg/s\n\n", gx_offset, gy_offset, gz_offset);
}

void readSensors(float &outRoll, float &outPitch, float &outGz) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);

    if (Wire.available() < 14) return;

    int16_t raw_ax = (Wire.read() << 8) | Wire.read();
    int16_t raw_ay = (Wire.read() << 8) | Wire.read();
    int16_t raw_az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // skip temp
    int16_t raw_gx = (Wire.read() << 8) | Wire.read();
    int16_t raw_gy = (Wire.read() << 8) | Wire.read();
    int16_t raw_gz = (Wire.read() << 8) | Wire.read();

    // Invert Y and Z because MPU-6050 is mounted upside-down
    float ax = raw_ax / 8192.0f;
    float ay = -(raw_ay / 8192.0f);
    float az = -(raw_az / 8192.0f);
    float gx = (raw_gx / 65.5f) - gx_offset;
    float gy = -((raw_gy / 65.5f) - gy_offset);
    float gz = -((raw_gz / 65.5f) - gz_offset);

    unsigned long now = micros();
    float dt = (now - last_time) / 1000000.0f;
    last_time = now;
    if (dt > 0.1f) dt = 0.004f;

    // Accel angles
    float acc_roll = atan2(ay, az) * 180.0f / PI;
    float acc_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;

    // Complementary filter (98% gyro, 2% accel)
    roll = 0.98f * (roll + gx * dt) + 0.02f * acc_roll;
    pitch = 0.98f * (pitch + gy * dt) + 0.02f * acc_pitch;

    outRoll = roll;
    outPitch = pitch;
    outGz = gz;
}

void printMenu() {
    Serial.println("\n========================================");
    Serial.println("     MINI DRONE HARDWARE CHECKOUT");
    Serial.println("========================================");
    Serial.println("Commands:");
    for (int i = 0; i < 4; i++) {
        Serial.printf("  %d -> Pulse %s for 1s\n", i + 1, motors[i].name);
    }
    Serial.println("  r1, r2, r3, r4 -> Reverse motor direction");
    Serial.println("  m -> Test all 4 motors (battery load check)");
    Serial.println("  s -> Toggle live sensor stream (Roll/Pitch)");
    Serial.println("  c -> Recalibrate gyro flat on table");
    Serial.println("  h -> Print this menu");
    Serial.println("========================================\n");
}

void testSingleMotor(int idx) {
    Serial.printf("\n>>> Testing %s (Reversed: %s) for 1.2s at 20%% duty...\n",
        motors[idx].name, motors[idx].reversed ? "YES" : "NO");
    applyMotor(idx, TEST_DUTY);
    delay(1200);
    stopMotor(idx);
    Serial.println(">>> Motor stopped.");
    Serial.println("Check: 1) Did it spin? 2) Which arm is it? 3) Does it blow air DOWN?");
}

void testAllMotorsRamp() {
    Serial.println("\n>>> Testing ALL 4 motors simultaneous ramp (Battery voltage check)...");
    for (int d = 50; d <= TEST_DUTY; d += 10) {
        for (int i = 0; i < 4; i++) applyMotor(i, d);
        delay(40);
    }
    delay(800);
    for (int i = 0; i < 4; i++) stopMotor(i);
    Serial.println(">>> All motors stopped. If ESP32 restarted, battery brownout occurred!");
}

void setup() {
    Serial.begin(115200);
    delay(2000); // Wait for USB CDC connection

    Serial.println("\n[BOOT] ESP32-C3 Mini Drone Initializing...");

    initMotors();
    Serial.println("[MOTORS] Pins & PWM initialized (All stopped).");

    if (!initMPU()) {
        Serial.println("[ERROR] MPU-6050 NOT detected on SDA=5, SCL=6! Check wiring & 3.3V power.");
    } else {
        Serial.println("[MPU6050] Connected successfully.");
        calibrateGyro();
    }

    last_time = micros();
    printMenu();
}

void handleCommand(String cmd) {
    cmd.trim();
    if (cmd == "1") testSingleMotor(0);
    else if (cmd == "2") testSingleMotor(1);
    else if (cmd == "3") testSingleMotor(2);
    else if (cmd == "4") testSingleMotor(3);
    else if (cmd == "r1") { motors[0].reversed = !motors[0].reversed; Serial.printf("M1 reversed = %d\n", motors[0].reversed); }
    else if (cmd == "r2") { motors[1].reversed = !motors[1].reversed; Serial.printf("M2 reversed = %d\n", motors[1].reversed); }
    else if (cmd == "r3") { motors[2].reversed = !motors[2].reversed; Serial.printf("M3 reversed = %d\n", motors[2].reversed); }
    else if (cmd == "r4") { motors[3].reversed = !motors[3].reversed; Serial.printf("M4 reversed = %d\n", motors[3].reversed); }
    else if (cmd == "m") testAllMotorsRamp();
    else if (cmd == "s") { stream_sensor = !stream_sensor; Serial.printf("Sensor stream: %s\n", stream_sensor ? "ON" : "OFF"); }
    else if (cmd == "c") calibrateGyro();
    else if (cmd == "h") printMenu();
    else if (cmd.length() > 0) Serial.println("Unknown command. Type 'h' for help.");
}

void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        handleCommand(cmd);
    }

    float r, p, gz;
    readSensors(r, p, gz);

    static unsigned long last_print = 0;
    if (stream_sensor && (millis() - last_print > 150)) {
        last_print = millis();
        // Read raw for axis verification
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(REG_ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6);
        float ax_raw = 0, ay_raw = 0, az_raw = 0;
        if (Wire.available() >= 6) {
            int16_t rax = (Wire.read() << 8) | Wire.read();
            int16_t ray = (Wire.read() << 8) | Wire.read();
            int16_t raz = (Wire.read() << 8) | Wire.read();
            ax_raw = rax / 8192.0f;
            ay_raw = ray / 8192.0f;
            az_raw = raz / 8192.0f;
        }
        Serial.printf("[ACCEL] ax: %+.2fg  ay: %+.2fg  az: %+.2fg | Est Roll: %6.1f  Pitch: %6.1f\n",
                      ax_raw, ay_raw, az_raw, r, p);
    }

    delay(2);
}

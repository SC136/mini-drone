#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>

// --- PIN DEFINITIONS ---
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

// --- PWM CONFIGURATION ---
const int PWM_FREQ = 20000; // 20kHz
const int PWM_RES = 10;     // 10-bit: 0 - 1023

struct Motor {
    int pinA;
    int pinB;
    int ledcChan;
    bool reversed;
};

// Motor mapping confirmed from physical layout:
// Top-Left (FL)     = M3 (Right IN1/IN2, GPIO 7/8)
// Top-Right (FR)    = M4 (Left IN3/IN4, GPIO 3/4)
// Bottom-Left (RL)  = M2 (Right IN3/IN4, GPIO 20/21)
// Bottom-Right (RR) = M1 (Left IN1/IN2, GPIO 0/1)
const int IDX_FL = 2;
const int IDX_FR = 3;
const int IDX_RL = 1;
const int IDX_RR = 0;

Motor motors[4] = {
    {L_IN1, L_IN2, 0, false},
    {R_IN3, R_IN4, 1, false},
    {R_IN1, R_IN2, 2, false},
    {L_IN3, L_IN4, 3, false}
};

// --- FLIGHT CONTROL STATE ---
bool armed = false;
float cmd_throttle = 0; // 0 - 1000
float cmd_roll = 0;     // -25 to +25 deg target
float cmd_pitch = 0;    // -25 to +25 deg target
float cmd_yaw_rate = 0; // -100 to +100 deg/s target

unsigned long last_packet_time = 0;
const unsigned long FAILSAFE_TIMEOUT_MS = 500;

// IMU State
float gx_offset = 0, gy_offset = 0, gz_offset = 0;
float roll_angle = 0, pitch_angle = 0;
float roll_rate = 0, pitch_rate = 0, yaw_rate = 0;
unsigned long last_loop_time = 0;

// PID Gains (tuned conservatively for micro coreless drone)
float Kp_roll = 1.6f, Ki_roll = 0.05f, Kd_roll = 0.4f;
float Kp_pitch = 1.6f, Ki_pitch = 0.05f, Kd_pitch = 0.4f;
float Kp_yaw = 2.0f, Ki_yaw = 0.0f, Kd_yaw = 0.1f;

float roll_integral = 0, pitch_integral = 0, yaw_integral = 0;
float last_roll_error = 0, last_pitch_error = 0;

WebServer server(80);
WebSocketsServer webSocket(81);

// --- MOTOR DRIVE HELPERS ---
void setupPwmChannel(int chan, int pin) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttach(pin, PWM_FREQ, PWM_RES);
#else
    ledcSetup(chan, PWM_FREQ, PWM_RES);
    ledcAttachPin(pin, chan);
#endif
}

void writePwmDuty(int chan, int pin, int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(pin, duty);
#else
    ledcWrite(chan, duty);
#endif
}

void stopAllMotors() {
    for (int i = 0; i < 4; i++) {
        writePwmDuty(motors[i].ledcChan, motors[i].reversed ? motors[i].pinB : motors[i].pinA, 0);
        digitalWrite(motors[i].pinA, LOW);
        digitalWrite(motors[i].pinB, LOW);
    }
}

void setMotorDuty(int idx, int duty) {
    if (duty < 0) duty = 0;
    if (duty > 1023) duty = 1023;
    Motor &m = motors[idx];
    int pwmPin = m.reversed ? m.pinB : m.pinA;
    int lowPin = m.reversed ? m.pinA : m.pinB;
    digitalWrite(lowPin, LOW);
    writePwmDuty(m.ledcChan, pwmPin, duty);
}

// --- MPU6050 DRIVER ---
bool initMPU() {
    Wire.begin(PIN_SDA, PIN_SCL, 400000);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x00); // Wake
    if (Wire.endTransmission() != 0) return false;

    delay(10);
    // DLPF ~42Hz
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_CONFIG);
    Wire.write(0x03);
    Wire.endTransmission();

    // Gyro +/- 500 deg/s
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_GYRO_CONFIG);
    Wire.write(0x08);
    Wire.endTransmission();

    // Accel +/- 4g
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_CONFIG);
    Wire.write(0x08);
    Wire.endTransmission();
    return true;
}

void calibrateGyro() {
    float sx = 0, sy = 0, sz = 0;
    const int N = 200;
    for (int i = 0; i < N; i++) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(REG_ACCEL_XOUT_H + 8);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6);
        if (Wire.available() >= 6) {
            int16_t gx = (Wire.read() << 8) | Wire.read();
            int16_t gy = (Wire.read() << 8) | Wire.read();
            int16_t gz = (Wire.read() << 8) | Wire.read();
            sx += gx / 65.5f;
            sy += gy / 65.5f;
            sz += gz / 65.5f;
        }
        delay(4);
    }
    gx_offset = sx / N;
    gy_offset = sy / N;
    gz_offset = sz / N;
}

void readSensors(float dt) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);
    if (Wire.available() < 14) return;

    int16_t raw_ax = (Wire.read() << 8) | Wire.read();
    int16_t raw_ay = (Wire.read() << 8) | Wire.read();
    int16_t raw_az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Skip temp
    int16_t raw_gx = (Wire.read() << 8) | Wire.read();
    int16_t raw_gy = (Wire.read() << 8) | Wire.read();
    int16_t raw_gz = (Wire.read() << 8) | Wire.read();

    // Invert Y and Z for upside-down MPU-6050 mounting
    float ax = raw_ax / 8192.0f;
    float ay = -(raw_ay / 8192.0f);
    float az = -(raw_az / 8192.0f);

    roll_rate = (raw_gx / 65.5f) - gx_offset;
    pitch_rate = -((raw_gy / 65.5f) - gy_offset);
    yaw_rate = -((raw_gz / 65.5f) - gz_offset);

    float acc_roll = atan2(ay, az) * 180.0f / PI;
    float acc_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;

    // Complementary filter
    roll_angle = 0.985f * (roll_angle + roll_rate * dt) + 0.015f * acc_roll;
    pitch_angle = 0.985f * (pitch_angle + pitch_rate * dt) + 0.015f * acc_pitch;
}

// --- WEB INTERFACE (HTML/CSS/JS) ---
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>Mini Drone</title>
<style>
  * { box-sizing: border-box; touch-action: none; -webkit-touch-callout: none; user-select: none; }
  body { margin: 0; background: #111; color: #eee; font-family: sans-serif; display: flex; flex-direction: column; height: 100vh; overflow: hidden; }
  header { padding: 10px 16px; background: #222; display: flex; justify-content: space-between; align-items: center; border-bottom: 1px solid #333; }
  #status { font-weight: bold; padding: 4px 8px; border-radius: 4px; font-size: 14px; background: #444; }
  .armed { background: #00aa44 !important; color: #fff; }
  .disarmed { background: #aa2222 !important; color: #fff; }
  #controls { display: flex; justify-content: space-around; padding: 10px; }
  button { padding: 12px 24px; font-size: 16px; font-weight: bold; border-radius: 8px; border: none; cursor: pointer; }
  #btn-arm { background: #2e7d32; color: #fff; }
  #btn-stop { background: #c62828; color: #fff; }
  #joysticks { flex: 1; display: flex; justify-content: space-around; align-items: center; padding: 10px; }
  .pad-box { width: 150px; height: 150px; background: #222; border-radius: 50%; border: 2px solid #444; position: relative; }
  .thumb { width: 50px; height: 50px; background: #0288d1; border-radius: 50%; position: absolute; left: 50px; top: 50px; }
  #telemetry { padding: 8px 16px; font-size: 13px; color: #aaa; text-align: center; background: #181818; border-top: 1px solid #333; }
</style>
</head>
<body>
<header>
  <div><b>Mini Drone Controller</b></div>
  <div id="status" class="disarmed">DISARMED</div>
</header>

<div id="controls">
  <button id="btn-arm" onclick="toggleArm()">ARM DRONE</button>
  <button id="btn-stop" onclick="emergencyStop()">EMERGENCY STOP</button>
</div>

<div id="joysticks">
  <div class="pad-box" id="pad-left">
    <div class="thumb" id="thumb-left" style="top: 100px;"></div>
  </div>
  <div class="pad-box" id="pad-right">
    <div class="thumb" id="thumb-right"></div>
  </div>
</div>

<div id="telemetry">
  Roll: <span id="tel-r">0</span>&deg; | Pitch: <span id="tel-p">0</span>&deg; | Throttle: <span id="tel-t">0</span>%
</div>

<script>
let ws, armed = false;
let throttle = 0, yaw = 0, roll = 0, pitch = 0;

function connectWS() {
  ws = new WebSocket('ws://' + location.hostname + ':81');
  ws.onopen = () => console.log('Connected');
  ws.onclose = () => { setTimeout(connectWS, 1000); setArmed(false); };
  ws.onmessage = (e) => {
    try {
      let d = JSON.parse(e.data);
      document.getElementById('tel-r').innerText = d.r.toFixed(1);
      document.getElementById('tel-p').innerText = d.p.toFixed(1);
      document.getElementById('tel-t').innerText = Math.round(throttle / 10);
      if (d.armed !== armed) setArmed(d.armed);
    } catch(err){}
  };
}
connectWS();

function setArmed(st) {
  armed = st;
  let s = document.getElementById('status');
  let b = document.getElementById('btn-arm');
  if (armed) {
    s.innerText = "ARMED"; s.className = "armed";
    b.innerText = "DISARM"; b.style.background = "#888";
  } else {
    s.innerText = "DISARMED"; s.className = "disarmed";
    b.innerText = "ARM DRONE"; b.style.background = "#2e7d32";
  }
}

function toggleArm() {
  if (!armed && throttle > 50) {
    alert("Lower throttle to 0 before arming!");
    return;
  }
  armed = !armed;
  sendCmd(armed ? 'ARM' : 'DISARM');
}

function emergencyStop() {
  armed = false;
  throttle = 0;
  sendCmd('STOP');
  setArmed(false);
  document.getElementById('thumb-left').style.top = '100px';
}

function sendCmd(act) {
  if (ws && ws.readyState === 1) {
    ws.send(JSON.stringify({
      act: act || 'stick',
      t: Math.round(throttle),
      r: Math.round(roll),
      p: Math.round(pitch),
      y: Math.round(yaw)
    }));
  }
}

setInterval(() => { if (armed) sendCmd('stick'); }, 30);

function bindPad(boxId, thumbId, isThrottle) {
  let box = document.getElementById(boxId);
  let thumb = document.getElementById(thumbId);
  let activeTouch = null;

  function handleMove(cx, cy) {
    let r = box.getBoundingClientRect();
    let x = (cx - (r.left + r.width / 2)) / (r.width / 2);
    let y = (cy - (r.top + r.height / 2)) / (r.height / 2);
    x = Math.max(-1, Math.min(1, x));
    y = Math.max(-1, Math.min(1, y));

    thumb.style.left = (50 + x * 50) + 'px';
    thumb.style.top = (50 + y * 50) + 'px';

    if (isThrottle) {
      throttle = (-y + 1) * 500; // 0 to 1000
      yaw = x * 100;            // -100 to +100 deg/s
    } else {
      roll = x * 25;            // -25 to +25 deg
      pitch = -y * 25;          // -25 to +25 deg
    }
  }

  box.addEventListener('touchstart', (e) => {
    activeTouch = e.changedTouches[0].identifier;
    handleMove(e.changedTouches[0].clientX, e.changedTouches[0].clientY);
  });

  window.addEventListener('touchmove', (e) => {
    for (let t of e.changedTouches) {
      if (t.identifier === activeTouch) handleMove(t.clientX, t.clientY);
    }
  });

  function resetRight() {
    if (!isThrottle) {
      thumb.style.left = '50px';
      thumb.style.top = '50px';
      roll = 0; pitch = 0;
    }
    activeTouch = null;
  }
  window.addEventListener('touchend', (e) => {
    for (let t of e.changedTouches) if (t.identifier === activeTouch) resetRight();
  });
}

bindPad('pad-left', 'thumb-left', true);
bindPad('pad-right', 'thumb-right', false);
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
    server.send_P(200, "text/html", INDEX_HTML);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if (type == WStype_TEXT) {
        String msg = String((char*)payload);
        last_packet_time = millis();

        if (msg.indexOf("\"act\":\"ARM\"") >= 0) {
            if (cmd_throttle < 50) {
                armed = true;
                roll_integral = 0; pitch_integral = 0; yaw_integral = 0;
                last_roll_error = 0; last_pitch_error = 0;
            }
        } else if (msg.indexOf("\"act\":\"DISARM\"") >= 0 || msg.indexOf("\"act\":\"STOP\"") >= 0) {
            armed = false;
            stopAllMotors();
        } else if (msg.indexOf("\"act\":\"stick\"") >= 0) {
            // Parse sticks
            int idxT = msg.indexOf("\"t\":");
            int idxR = msg.indexOf("\"r\":");
            int idxP = msg.indexOf("\"p\":");
            int idxY = msg.indexOf("\"y\":");
            if (idxT >= 0) cmd_throttle = msg.substring(idxT + 4).toInt();
            if (idxR >= 0) cmd_roll = msg.substring(idxR + 4).toFloat();
            if (idxP >= 0) cmd_pitch = msg.substring(idxP + 4).toFloat();
            if (idxY >= 0) cmd_yaw_rate = msg.substring(idxY + 4).toFloat();
        }
    } else if (type == WStype_DISCONNECTED) {
        armed = false;
        stopAllMotors();
    }
}

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("[DRONE] Starting Flight Controller...");

    for (int i = 0; i < 4; i++) {
        pinMode(motors[i].pinA, OUTPUT);
        pinMode(motors[i].pinB, OUTPUT);
        digitalWrite(motors[i].pinA, LOW);
        digitalWrite(motors[i].pinB, LOW);
        setupPwmChannel(motors[i].ledcChan, motors[i].pinA);
    }
    stopAllMotors();

    if (!initMPU()) {
        Serial.println("[ERROR] MPU-6050 failed to initialize!");
    } else {
        Serial.println("[MPU6050] Initialized. Calibrating gyro...");
        calibrateGyro();
        Serial.println("[MPU6050] Ready.");
    }

    // Start SoftAP
    WiFi.mode(WIFI_AP);
    WiFi.softAP("MiniDrone", "12345678");
    Serial.printf("[WIFI] AP Started. SSID: MiniDrone (pass: 12345678). IP: %s\n",
                  WiFi.softAPIP().toString().c_str());

    server.on("/", handleRoot);
    server.begin();

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);

    last_loop_time = micros();
    last_packet_time = millis();
}

void loop() {
    server.handleClient();
    webSocket.loop();

    // 250Hz Control Loop (every 4000 microseconds)
    unsigned long now = micros();
    if (now - last_loop_time >= 4000) {
        float dt = (now - last_loop_time) / 1000000.0f;
        last_loop_time = now;
        if (dt > 0.01f) dt = 0.004f;

        readSensors(dt);

        // Failsafe checks
        if (armed && (millis() - last_packet_time > FAILSAFE_TIMEOUT_MS)) {
            armed = false;
            stopAllMotors();
            Serial.println("[FAILSAFE] Connection lost! Disarming.");
        }

        // Tilt limit: disarm if tilted > 60 deg (crash/flip)
        if (armed && (abs(roll_angle) > 60.0f || abs(pitch_angle) > 60.0f)) {
            armed = false;
            stopAllMotors();
            Serial.println("[FAILSAFE] Extreme tilt detected! Disarming.");
        }

        if (!armed || cmd_throttle < 30) {
            stopAllMotors();
            roll_integral = 0;
            pitch_integral = 0;
            yaw_integral = 0;
        } else {
            // Roll PID
            float roll_error = cmd_roll - roll_angle;
            roll_integral += roll_error * dt;
            roll_integral = constrain(roll_integral, -50.0f, 50.0f);
            float roll_derivative = -roll_rate; // Derivative on measurement
            float roll_adj = (Kp_roll * roll_error) + (Ki_roll * roll_integral) + (Kd_roll * roll_derivative);

            // Pitch PID
            float pitch_error = cmd_pitch - pitch_angle;
            pitch_integral += pitch_error * dt;
            pitch_integral = constrain(pitch_integral, -50.0f, 50.0f);
            float pitch_derivative = -pitch_rate;
            float pitch_adj = (Kp_pitch * pitch_error) + (Ki_pitch * pitch_integral) + (Kd_pitch * pitch_derivative);

            // Yaw Rate PID
            float yaw_error = cmd_yaw_rate - yaw_rate;
            float yaw_adj = (Kp_yaw * yaw_error);

            // Mixer (Quad X)
            // FL (Top-Left):     T + Roll + Pitch + Yaw
            // FR (Top-Right):    T - Roll + Pitch - Yaw
            // RL (Bottom-Left):  T + Roll - Pitch - Yaw
            // RR (Bottom-Right): T - Roll - Pitch + Yaw
            int fl = (int)(cmd_throttle + roll_adj + pitch_adj + yaw_adj);
            int fr = (int)(cmd_throttle - roll_adj + pitch_adj - yaw_adj);
            int rl = (int)(cmd_throttle + roll_adj - pitch_adj - yaw_adj);
            int rr = (int)(cmd_throttle - roll_adj - pitch_adj + yaw_adj);

            setMotorDuty(IDX_FL, fl);
            setMotorDuty(IDX_FR, fr);
            setMotorDuty(IDX_RL, rl);
            setMotorDuty(IDX_RR, rr);
        }

        // Push telemetry to connected web clients at ~10Hz
        static unsigned long last_telemetry = 0;
        if (millis() - last_telemetry > 100) {
            last_telemetry = millis();
            char buf[64];
            snprintf(buf, sizeof(buf), "{\"r\":%.1f,\"p\":%.1f,\"armed\":%s}",
                     roll_angle, pitch_angle, armed ? "true" : "false");
            webSocket.broadcastTXT(buf);
        }
    }
}

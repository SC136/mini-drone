# ESP32-C3 Mini Drone

A minimalist, Wi-Fi web-controlled flight controller for an ESP32-C3 SuperMini, MPU-6050, and dual DRV8833 motor drivers.

---

## 1. Pin Map

| Component | Pin | ESP32-C3 GPIO | Note |
|---|---|---|---|
| **MPU-6050** | SDA | **GPIO 5** | I2C Data |
| **MPU-6050** | SCL | **GPIO 6** | I2C Clock (400kHz) |
| **Left Driver** | IN1 | **GPIO 0** | Motor 1 A |
| **Left Driver** | IN2 | **GPIO 1** | Motor 1 B |
| **Left Driver** | IN3 | **GPIO 3** | Motor 2 A |
| **Left Driver** | IN4 | **GPIO 4** | Motor 2 B |
| **Right Driver** | IN1 | **GPIO 7** | Motor 3 A |
| **Right Driver** | IN2 | **GPIO 8** | Motor 3 B *(Strapping pin / blue LED)* |
| **Right Driver** | IN3 | **GPIO 20** | Motor 4 A |
| **Right Driver** | IN4 | **GPIO 21** | Motor 4 B |

### Critical Hardware Checklist
1. **DRV8833 `nSLEEP` (or `EEP` / `SLP`) Pin:** DRV8833 modules have an active-low sleep pin. If left unconnected or floating, outputs remain OFF. Connect `nSLEEP` / `EEP` to 3.3V or battery (+).
2. **Common Ground:** All GND pins (Battery -, ESP32 GND, Left DRV8833 GND, Right DRV8833 GND, MPU6050 GND) must be firmly connected together.
3. **Power:** Power DRV8833 `VM` directly from a 1S LiPo (3.7V - 4.2V) using thick wires. Solder a 100uF - 470uF capacitor across VM and GND on each driver to prevent motor noise from resetting the ESP32.
4. **SAFETY: Remove all propellers for initial testing!**

---

## 2. Bring-Up Phase 1: Hardware Checkout

Upload the diagnostic tool to check sensors and motor directions safely:

```bash
pio run -e diagnostic -t upload
pio device monitor
```

### Serial Monitor Commands (115200 baud):
- `1` / `2` / `3` / `4`: Pulses Motor 1, 2, 3, or 4 for 1.2s at ~20% power.
  - Note which physical arm spins: **Front-Left (FL)**, **Front-Right (FR)**, **Rear-Left (RL)**, or **Rear-Right (RR)**.
  - Check airflow: when props are attached later, air must blow **DOWN**.
- `r1` / `r2` / `r3` / `r4`: Toggles software reverse for that motor if it spins backwards.
- `m`: Ramps all 4 motors together to verify battery voltage holds under load without brownout.
- `s`: Streams live IMU Roll / Pitch / Gyro rates.
- `c`: Recalibrates gyro level on a flat table.

---

## 3. Quad-X Motor & Propeller Rules

```
                       FRONT (Nose)
                            ▲
                            │
       [ M3 / FL ]                     [ M4 / FR ]
       Top-Left                        Top-Right
       Prop: CW (Type A)               Prop: CCW (Type B)
       Spin: CLOCKWISE ↻               Spin: COUNTER-CLOCKWISE ↺
              \                       /
               \                     /
                     [ DRONE ]
               /                     \
              /                       \
       [ M2 / RL ]                     [ M1 / RR ]
       Bottom-Left                     Bottom-Right
       Prop: CCW (Type B)              Prop: CW (Type A)
       Spin: COUNTER-CLOCKWISE ↺       Spin: CLOCKWISE ↻
                            │
                       [USB-C CABLE]
                            │
                          REAR
```

| Arm Position | Motor Label | Driver Channel | Propeller Type | Spin Direction |
|---|---|---|---|---|
| **Top-Left** (FL) | **M3** | Right IN1/IN2 (GPIO 7/8) | **CW** (Type A) | **Clockwise ↻** |
| **Top-Right** (FR) | **M4** | Left IN3/IN4 (GPIO 3/4) | **CCW** (Type B) | **Counter-Clockwise ↺** |
| **Bottom-Left** (RL) | **M2** | Right IN3/IN4 (GPIO 20/21) | **CCW** (Type B) | **Counter-Clockwise ↺** |
| **Bottom-Right** (RR) | **M1** | Left IN1/IN2 (GPIO 0/1) | **CW** (Type A) | **Clockwise ↻** |

---

## 4. Bring-Up Phase 2: Flight Controller

Flash the flight controller:

```bash
pio run -e flight -t upload
```

### Flying:
1. Power the drone on a flat surface. Leave it still for 2 seconds while the gyro auto-calibrates.
2. On your phone, connect to Wi-Fi network:
   - **SSID:** `MiniDrone`
   - **Password:** `12345678`
3. Open mobile browser to: `http://192.168.4.1`
4. Tap **ARM DRONE** (throttle must be at 0%).
5. Gently raise the left stick to spin up motors and hover.
6. **Failsafe:** Motors automatically cut off if Wi-Fi disconnects for >500ms or if tilt exceeds 60°.

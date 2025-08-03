
---

# Model Rocket TVC System (Arduino)

This code implements a Thrust Vector Control (TVC) system for a model rocket using an MPU9250 IMU and two servos to stabilise pitch and yaw via PID control.

---

## Features

* Real-time pitch and yaw correction
* Dual PID controllers
* Servo output for TVC mount
* Basic exponential smoothing
* Serial debug output

---

## Hardware

* Arduino (Uno, Nano, etc.)
* MPU9250 IMU (I2C)
* 2x Servo motors (e.g. SG90/MG90S)
* External servo power recommended

---

## Pinout

| Component   | Pin                |
| ----------- | ------------------ |
| MPU9250     | A4 (SDA), A5 (SCL) |
| Pitch Servo | D9                 |
| Yaw Servo   | D10                |

---

## Libraries

* `MPU9250_asukiaaa`
* `PID_v1`
* `Servo`
* `Wire` (built-in)

---

## How It Works

1. IMU reads acceleration
2. Pitch and yaw estimated from accel data
3. Filtered values sent to PID controllers
4. PID outputs adjust servo angles
5. Servos correct gimbal position in real time

---

## PID Tuning

Default values:

```cpp
double Kp_pitch = 2.0, Ki_pitch = 0.5, Kd_pitch = 0.1;
double Kp_yaw   = 2.0, Ki_yaw   = 0.5, Kd_yaw   = 0.1;
```

Adjust based on your rocket's dynamics.

---

## Limitations

* Yaw from accel is crude; use gyro fusion or filters for accuracy
* No compensation for fast rotational motion or thrust-induced noise
* No launch detection or arming logic
* Filter is basic (exponential smoothing only)

---

## Loop Rate

\~100Hz (fixed `delay(10)`)

---

## Serial Output

Example:

```
Pitch: -2.53 | Servo: 84.12 || Yaw: 3.81 | Servo: 95.03
```

---

## Notes

* Ensure servos are calibrated and centered
* System assumes vertical is 0° pitch/yaw
* Use shielded wires for cleaner IMU data
* Test on ground before flight

---

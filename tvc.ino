#include <MPU9250_asukiaaa.h>
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>

// IMU and Servos
MPU9250_asukiaaa imu;
Servo pitchServo;
Servo yawServo;

// PID variables for pitch
double pitchSetpoint = 0;
double pitchInput = 0;
double pitchOutput = 90;

// PID variables for yaw
double yawSetpoint = 0;
double yawInput = 0;
double yawOutput = 90;

// PID tuning (can tweak separately)
double Kp_pitch = 2.0, Ki_pitch = 0.5, Kd_pitch = 0.1;
double Kp_yaw = 2.0, Ki_yaw = 0.5, Kd_yaw = 0.1;

// Create PID controllers
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, Kp_pitch, Ki_pitch, Kd_pitch, DIRECT);
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, Kp_yaw, Ki_yaw, Kd_yaw, DIRECT);

void setup() {
  Serial.begin(9600);
  Wire.begin();

  imu.setWire(&Wire);
  imu.beginAccel();
  imu.beginGyro();
  imu.beginMag();

  pitchServo.attach(9); // Digital pin 9 for pitch
  yawServo.attach(10);  // Digital pin 10 for yaw

  pitchServo.write(90);
  yawServo.write(90);

  pitchPID.SetMode(AUTOMATIC);
  yawPID.SetMode(AUTOMATIC);

  pitchPID.SetOutputLimits(0, 180);
  yawPID.SetOutputLimits(0, 180);
}

void loop() {
  imu.accelUpdate();
  imu.gyroUpdate();
  imu.magUpdate();

  // Estimate pitch and yaw from accelerometer
  float rawPitch = atan2(imu.accelY(), imu.accelZ()) * 180.0 / PI;
  float rawYaw = atan2(imu.accelX(), imu.accelZ()) * 180.0 / PI; // crude yaw from accel

  // Basic smoothing
  static float filteredPitch = 0;
  static float filteredYaw = 0;
  filteredPitch = 0.9 * filteredPitch + 0.1 * rawPitch;
  filteredYaw = 0.9 * filteredYaw + 0.1 * rawYaw;

  pitchInput = filteredPitch;
  yawInput = filteredYaw;

  // Run both PID controllers
  pitchPID.Compute();
  yawPID.Compute();

  // Update servos
  pitchServo.write(pitchOutput);
  yawServo.write(yawOutput);

  // Debug
  Serial.print("Pitch: ");
  Serial.print(pitchInput);
  Serial.print(" | Servo: ");
  Serial.print(pitchOutput);

  Serial.print(" || Yaw: ");
  Serial.print(yawInput);
  Serial.print(" | Servo: ");
  Serial.println(yawOutput);

  delay(10); // ~100Hz loop
}

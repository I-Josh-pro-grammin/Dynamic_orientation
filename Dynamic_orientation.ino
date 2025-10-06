#include <Wire.h>

// MPU6050 registers
#define MPU_ADDR 0x68
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H  0x43

// Sensitivities (default full-scale): ±2g, ±250°/s
const float ACCEL_SENS = 16384.0; // LSB/g
const float GYRO_SENS  = 131.0;   // LSB/(°/s)

float roll = 0.0f;   // X
float pitch = 0.0f;  // Y
float yaw = 0.0f;    // Z (new)

float gx_bias = 0, gy_bias = 0, gz_bias = 0;
unsigned long lastMicros = 0;
const float alpha = 0.98f; // Complementary filter factor

int16_t read16(int reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  int16_t hi = Wire.read();
  int16_t lo = Wire.read();
  return (hi << 8) | lo;
}

void readMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  int16_t rawAX = read16(REG_ACCEL_XOUT_H);
  int16_t rawAY = read16(REG_ACCEL_XOUT_H + 2);
  int16_t rawAZ = read16(REG_ACCEL_XOUT_H + 4);
  int16_t rawGX = read16(REG_GYRO_XOUT_H);
  int16_t rawGY = read16(REG_GYRO_XOUT_H + 2);
  int16_t rawGZ = read16(REG_GYRO_XOUT_H + 4);

  ax = (float)rawAX / ACCEL_SENS;
  ay = (float)rawAY / ACCEL_SENS;
  az = (float)rawAZ / ACCEL_SENS;
  gx = (float)rawGX / GYRO_SENS;
  gy = (float)rawGY / GYRO_SENS;
  gz = (float)rawGZ / GYRO_SENS;

  gx -= gx_bias;
  gy -= gy_bias;
  gz -= gz_bias;
}

void calibrateGyro(int samples = 500) {
  long sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < samples; i++) {
    int16_t rawGX = read16(REG_GYRO_XOUT_H);
    int16_t rawGY = read16(REG_GYRO_XOUT_H + 2);
    int16_t rawGZ = read16(REG_GYRO_XOUT_H + 4);
    sumX += rawGX;
    sumY += rawGY;
    sumZ += rawGZ;
    delay(2);
  }
  gx_bias = (sumX / (float)samples) / GYRO_SENS;
  gy_bias = (sumY / (float)samples) / GYRO_SENS;
  gz_bias = (sumZ / (float)samples) / GYRO_SENS;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(100);
  calibrateGyro(600);

  // Seed roll/pitch from accelerometer
  float ax, ay, az, gx, gy, gz;
  readMPU(ax, ay, az, gx, gy, gz);
  float acc_roll  = atan2(ay, az) * 180.0 / PI;
  float acc_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  roll = acc_roll;
  pitch = acc_pitch;
  yaw = 0.0f; // Initialize yaw to 0 (no magnetometer for absolute heading)

  lastMicros = micros();
}

void loop() {
  float ax, ay, az, gx, gy, gz;
  readMPU(ax, ay, az, gx, gy, gz);

  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6; // seconds
  lastMicros = now;

  // Accel-only angles for roll and pitch
  float acc_roll  = atan2(ay, az) * 180.0 / PI;
  float acc_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  // Integrate gyro: gyroX≈roll-rate, gyroY≈pitch-rate, gyroZ≈yaw-rate
  float roll_gyro  = roll  + gx * dt;
  float pitch_gyro = pitch + gy * dt;
  float yaw_gyro   = yaw   + gz * dt;

  // Complementary filter for roll and pitch; yaw uses gyro only
  roll  = alpha * roll_gyro  + (1.0f - alpha) * acc_roll;
  pitch = alpha * pitch_gyro + (1.0f - alpha) * acc_pitch;
  yaw   = yaw_gyro; // No accelerometer correction for yaw (no magnetometer)

  // Stream CSV: pitch,roll,yaw (degrees)
  Serial.print(pitch, 3); Serial.print(",");
  Serial.print(roll, 3); Serial.print(",");
  Serial.println(yaw, 3);

  delay(10); // ~100 Hz loop
}
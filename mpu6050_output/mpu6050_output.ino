#include <Wire.h>

const uint8_t MPU6050_ADDRESS = 0x68;
const uint8_t MPU6050_PWR_MGMT_1 = 0x6B;
const uint8_t MPU6050_ACCEL_XOUT_H = 0x3B;
const uint8_t MPU6050_GYRO_CONFIG = 0x1B;
const uint8_t MPU6050_ACCEL_CONFIG = 0x1C;
const unsigned long SERIAL_BAUD = 115200;
const float ACCEL_SCALE = 16384.0f;
const float GYRO_SCALE = 131.0f;
const float COMPLEMENTARY_ALPHA = 0.98f;
const int GYRO_CALIBRATION_SAMPLES = 500;

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;
float rollDeg = 0.0f;
float pitchDeg = 0.0f;
float yawDeg = 0.0f;
unsigned long lastMicros = 0;

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

bool readRawMpuData(int16_t& accelX, int16_t& accelY, int16_t& accelZ,
                    int16_t& gyroX, int16_t& gyroY, int16_t& gyroZ) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t bytesToRead = 14;
  uint8_t received = Wire.requestFrom(
    static_cast<uint8_t>(MPU6050_ADDRESS),
    static_cast<uint8_t>(bytesToRead),
    static_cast<uint8_t>(true)
  );
  if (received != bytesToRead) {
    return false;
  }

  accelX = (Wire.read() << 8) | Wire.read();
  accelY = (Wire.read() << 8) | Wire.read();
  accelZ = (Wire.read() << 8) | Wire.read();
  Wire.read();
  Wire.read();
  gyroX = (Wire.read() << 8) | Wire.read();
  gyroY = (Wire.read() << 8) | Wire.read();
  gyroZ = (Wire.read() << 8) | Wire.read();
  return true;
}

void calibrateGyro() {
  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;

  for (int sample = 0; sample < GYRO_CALIBRATION_SAMPLES; sample++) {
    int16_t accelX = 0;
    int16_t accelY = 0;
    int16_t accelZ = 0;
    int16_t gyroX = 0;
    int16_t gyroY = 0;
    int16_t gyroZ = 0;

    if (readRawMpuData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ)) {
      sumX += gyroX;
      sumY += gyroY;
      sumZ += gyroZ;
    }
    delay(3);
  }

  gyroBiasX = static_cast<float>(sumX) / GYRO_CALIBRATION_SAMPLES;
  gyroBiasY = static_cast<float>(sumY) / GYRO_CALIBRATION_SAMPLES;
  gyroBiasZ = static_cast<float>(sumZ) / GYRO_CALIBRATION_SAMPLES;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(400000);

  writeRegister(MPU6050_PWR_MGMT_1, 0x00);
  writeRegister(MPU6050_ACCEL_CONFIG, 0x00);
  writeRegister(MPU6050_GYRO_CONFIG, 0x00);
  delay(100);

  calibrateGyro();
  lastMicros = micros();
}

void loop() {
  int16_t rawAccelX = 0;
  int16_t rawAccelY = 0;
  int16_t rawAccelZ = 0;
  int16_t rawGyroX = 0;
  int16_t rawGyroY = 0;
  int16_t rawGyroZ = 0;

  if (!readRawMpuData(rawAccelX, rawAccelY, rawAccelZ, rawGyroX, rawGyroY, rawGyroZ)) {
    delay(10);
    return;
  }

  unsigned long nowMicros = micros();
  float deltaTime = (nowMicros - lastMicros) * 0.000001f;
  lastMicros = nowMicros;

  if (deltaTime <= 0.0f || deltaTime > 0.5f) {
    deltaTime = 0.01f;
  }

  float accelX = static_cast<float>(rawAccelX) / ACCEL_SCALE;
  float accelY = static_cast<float>(rawAccelY) / ACCEL_SCALE;
  float accelZ = static_cast<float>(rawAccelZ) / ACCEL_SCALE;

  float gyroX = (static_cast<float>(rawGyroX) - gyroBiasX) / GYRO_SCALE;
  float gyroY = (static_cast<float>(rawGyroY) - gyroBiasY) / GYRO_SCALE;
  float gyroZ = (static_cast<float>(rawGyroZ) - gyroBiasZ) / GYRO_SCALE;

  float accelRollDeg = atan2(accelY, accelZ) * RAD_TO_DEG;
  float accelPitchDeg = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * RAD_TO_DEG;

  rollDeg = COMPLEMENTARY_ALPHA * (rollDeg + gyroX * deltaTime) + (1.0f - COMPLEMENTARY_ALPHA) * accelRollDeg;
  pitchDeg = COMPLEMENTARY_ALPHA * (pitchDeg + gyroY * deltaTime) + (1.0f - COMPLEMENTARY_ALPHA) * accelPitchDeg;
  yawDeg += gyroZ * deltaTime;

  float rollRad = rollDeg * DEG_TO_RAD;
  float pitchRad = pitchDeg * DEG_TO_RAD;

  float gravityX = -sin(pitchRad);
  float gravityY = sin(rollRad) * cos(pitchRad);
  float gravityZ = cos(rollRad) * cos(pitchRad);

  float linearAccelX = accelX - gravityX;
  float linearAccelY = accelY - gravityY;
  float linearAccelZ = accelZ - gravityZ;

  Serial.print("gyro(");
  Serial.print(gyroX, 3);
  Serial.print(",");
  Serial.print(gyroY, 3);
  Serial.print(",");
  Serial.print(gyroZ, 3);
  Serial.print(") accel(");
  Serial.print(linearAccelX, 3);
  Serial.print(",");
  Serial.print(linearAccelY, 3);
  Serial.print(",");
  Serial.print(linearAccelZ, 3);
  Serial.print(") angle(");
  Serial.print(rollDeg, 2);
  Serial.print(",");
  Serial.print(pitchDeg, 2);
  Serial.print(",");
  Serial.print(yawDeg, 2);
  Serial.println(")");

  delay(10);
}
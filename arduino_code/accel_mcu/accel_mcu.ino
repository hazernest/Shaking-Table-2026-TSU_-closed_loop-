// accel_mcu.ino — Arduino Uno
// GY-91 / MPU9250 + AK8963 accelerometer/gyro/magnetometer.
// Sends IMU telemetry over Serial at ~100 Hz.
// No stepper, no command queue.
//
// Output line format (sent continuously):
//   IMU,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,roll,pitch,yaw
//   (accel in g-units; gyro in deg/sec; angles in degrees)
//
// On Uno SDA=A4, SCL=A5 (default Wire pins — do NOT pass pins to Wire.begin()).

#include <Wire.h>
#include <math.h>

const unsigned long SERIAL_BAUD         = 115200;
const unsigned long TELEMETRY_INTERVAL_US = 10000;   // 100 Hz

// MPU9250 registers
const uint8_t MPU9250_ADDRESS    = 0x68;
const uint8_t AK8963_ADDRESS     = 0x0C;
const uint8_t MPU9250_PWR_MGMT_1 = 0x6B;
const uint8_t MPU9250_PWR_MGMT_2 = 0x6C;
const uint8_t MPU9250_SMPLRT_DIV = 0x19;
const uint8_t MPU9250_CONFIG     = 0x1A;
const uint8_t MPU9250_GYRO_CONFIG  = 0x1B;
const uint8_t MPU9250_ACCEL_CONFIG = 0x1C;
const uint8_t MPU9250_ACCEL_CONFIG2 = 0x1D;
const uint8_t MPU9250_INT_PIN_CFG  = 0x37;
const uint8_t MPU9250_ACCEL_XOUT_H = 0x3B;
const uint8_t MPU9250_WHO_AM_I   = 0x75;

// AK8963 registers
const uint8_t AK8963_WHO_AM_I = 0x00;
const uint8_t AK8963_ST1      = 0x02;
const uint8_t AK8963_HXL      = 0x03;
const uint8_t AK8963_CNTL1    = 0x0A;
const uint8_t AK8963_ASAX     = 0x10;

// Scaling
const float ACCEL_SCALE          = 8192.0f;   // ±4g → LSB/g
const float GYRO_SCALE           = 65.5f;     // ±500 dps → LSB/(deg/s)
const float MAG_SCALE            = 0.15f;     // µT/LSB
const float COMPLEMENTARY_ALPHA  = 0.98f;
const float YAW_MAG_ALPHA        = 0.02f;
const int   GYRO_CALIBRATION_SAMPLES = 500;


// State
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
float magOffsetX = 0.0f, magOffsetY = 0.0f, magOffsetZ = 0.0f;
float magScaleX  = 1.0f, magScaleY  = 1.0f, magScaleZ  = 1.0f;
float rollDeg    = 0.0f, pitchDeg   = 0.0f, yawDeg     = 0.0f;
unsigned long lastImuMicros       = 0;
unsigned long lastTelemetryMicros = 0;
bool imuReady = false;

// ── I2C helpers ───────────────────────────────────────────────────────────────

bool writeRegister(uint8_t deviceAddress, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

uint8_t readRegister(uint8_t deviceAddress, uint8_t reg) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(deviceAddress, (uint8_t)1, (uint8_t)true) != 1) return 0;
  return Wire.read();
}

bool readRegisters(uint8_t deviceAddress, uint8_t reg, uint8_t* buffer, uint8_t length) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t received = Wire.requestFrom(deviceAddress, length, (uint8_t)true);
  if (received != length) return false;
  for (uint8_t i = 0; i < length; i++) buffer[i] = Wire.read();
  return true;
}

// ── sensor reads ──────────────────────────────────────────────────────────────

bool readRawMpuData(int16_t& accelX, int16_t& accelY, int16_t& accelZ,
                    int16_t& gyroX,  int16_t& gyroY,  int16_t& gyroZ) {
  uint8_t buf[14];
  if (!readRegisters(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, buf, sizeof(buf))) return false;
  accelX = ((int16_t)buf[0]  << 8) | buf[1];
  accelY = ((int16_t)buf[2]  << 8) | buf[3];
  accelZ = ((int16_t)buf[4]  << 8) | buf[5];
  gyroX  = ((int16_t)buf[8]  << 8) | buf[9];
  gyroY  = ((int16_t)buf[10] << 8) | buf[11];
  gyroZ  = ((int16_t)buf[12] << 8) | buf[13];
  return true;
}

bool readMagData(float& magX, float& magY, float& magZ) {
  uint8_t status = readRegister(AK8963_ADDRESS, AK8963_ST1);
  if ((status & 0x01) == 0) return false;
  uint8_t buf[7];
  if (!readRegisters(AK8963_ADDRESS, AK8963_HXL, buf, sizeof(buf))) return false;
  if (buf[6] & 0x08) return false;
  int16_t rawX = ((int16_t)buf[1] << 8) | buf[0];
  int16_t rawY = ((int16_t)buf[3] << 8) | buf[2];
  int16_t rawZ = ((int16_t)buf[5] << 8) | buf[4];
  magX = (rawX * MAG_SCALE * magScaleX) - magOffsetX;
  magY = (rawY * MAG_SCALE * magScaleY) - magOffsetY;
  magZ = (rawZ * MAG_SCALE * magScaleZ) - magOffsetZ;
  return true;
}

// ── orientation ───────────────────────────────────────────────────────────────

float wrapAngleDeg(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

float blendAngleDeg(float current, float target, float alpha) {
  return wrapAngleDeg(current + alpha * wrapAngleDeg(target - current));
}

void updateOrientation(float gxRad, float gyRad, float gzRad,
                       float ax, float ay, float az,
                       float mx, float my, float mz,
                       float dt) {
  float accelRoll  = atan2(ay, az) * RAD_TO_DEG;
  float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  rollDeg  = COMPLEMENTARY_ALPHA * (rollDeg  + gxRad * dt * RAD_TO_DEG) + (1.0f - COMPLEMENTARY_ALPHA) * accelRoll;
  pitchDeg = COMPLEMENTARY_ALPHA * (pitchDeg + gyRad * dt * RAD_TO_DEG) + (1.0f - COMPLEMENTARY_ALPHA) * accelPitch;

  float rRad = rollDeg * DEG_TO_RAD;
  float pRad = pitchDeg * DEG_TO_RAD;
  float mxc  = mx * cos(pRad) + mz * sin(pRad);
  float myc  = mx * sin(rRad) * sin(pRad) + my * cos(rRad) - mz * sin(rRad) * cos(pRad);
  float heading = atan2(-myc, mxc) * RAD_TO_DEG;
  yawDeg = blendAngleDeg(yawDeg + gzRad * dt * RAD_TO_DEG, heading, YAW_MAG_ALPHA);
}

// ── calibration ───────────────────────────────────────────────────────────────

void calibrateGyro() {
  long sumX = 0, sumY = 0, sumZ = 0;
  for (int s = 0; s < GYRO_CALIBRATION_SAMPLES; s++) {
    int16_t ax, ay, az, gx, gy, gz;
    if (readRawMpuData(ax, ay, az, gx, gy, gz)) {
      sumX += gx; sumY += gy; sumZ += gz;
    }
    delay(3);
  }
  gyroBiasX = (float)sumX / GYRO_CALIBRATION_SAMPLES;
  gyroBiasY = (float)sumY / GYRO_CALIBRATION_SAMPLES;
  gyroBiasZ = (float)sumZ / GYRO_CALIBRATION_SAMPLES;
}

void calibrateMagnetometerFactoryAdjustments() {
  writeRegister(AK8963_ADDRESS, AK8963_CNTL1, 0x00); delay(10);
  writeRegister(AK8963_ADDRESS, AK8963_CNTL1, 0x0F); delay(10);
  uint8_t asaX = readRegister(AK8963_ADDRESS, AK8963_ASAX);
  uint8_t asaY = readRegister(AK8963_ADDRESS, AK8963_ASAX + 1);
  uint8_t asaZ = readRegister(AK8963_ADDRESS, AK8963_ASAX + 2);
  magScaleX = ((float)(asaX - 128) / 256.0f) + 1.0f;
  magScaleY = ((float)(asaY - 128) / 256.0f) + 1.0f;
  magScaleZ = ((float)(asaZ - 128) / 256.0f) + 1.0f;
  writeRegister(AK8963_ADDRESS, AK8963_CNTL1, 0x00); delay(10);
  writeRegister(AK8963_ADDRESS, AK8963_CNTL1, 0x16); delay(10);
}

void initializeImu() {
  uint8_t whoAmI = readRegister(MPU9250_ADDRESS, MPU9250_WHO_AM_I);
  if (whoAmI != 0x71 && whoAmI != 0x73) {
    Serial.print("IMU warning: WHO_AM_I=0x"); Serial.println(whoAmI, HEX);
  }
  writeRegister(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1,   0x80); delay(100);
  writeRegister(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1,   0x01);
  writeRegister(MPU9250_ADDRESS, MPU9250_PWR_MGMT_2,   0x00);
  writeRegister(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV,   0x04);
  writeRegister(MPU9250_ADDRESS, MPU9250_CONFIG,        0x03);
  writeRegister(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG,   0x08);
  writeRegister(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG,  0x08);
  writeRegister(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, 0x03);
  writeRegister(MPU9250_ADDRESS, MPU9250_INT_PIN_CFG,   0x02); delay(50);

  uint8_t magWhoAmI = readRegister(AK8963_ADDRESS, AK8963_WHO_AM_I);
  if (magWhoAmI != 0x48) {
    Serial.print("MAG warning: WHO_AM_I=0x"); Serial.println(magWhoAmI, HEX);
  }
  calibrateMagnetometerFactoryAdjustments();
  calibrateGyro();
  imuReady = true;
}

// ── telemetry output ──────────────────────────────────────────────────────────

void updateImuAndTelemetry() {
  if (!imuReady) return;

  unsigned long nowMicros = micros();
  if (nowMicros - lastTelemetryMicros < TELEMETRY_INTERVAL_US) return;

  int16_t rawAX = 0, rawAY = 0, rawAZ = 0;
  int16_t rawGX = 0, rawGY = 0, rawGZ = 0;
  if (!readRawMpuData(rawAX, rawAY, rawAZ, rawGX, rawGY, rawGZ)) {
    lastTelemetryMicros = nowMicros;
    return;
  }

  float mx = 0.0f, my = 0.0f, mz = 1.0f;
  readMagData(mx, my, mz);   // use last good value on failure

  float dt = (nowMicros - lastImuMicros) * 0.000001f;
  lastImuMicros       = nowMicros;
  lastTelemetryMicros = nowMicros;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;

  float ax = (float)rawAX / ACCEL_SCALE;
  float ay = (float)rawAY / ACCEL_SCALE;
  float az = (float)rawAZ / ACCEL_SCALE;
  float gx = ((float)rawGX - gyroBiasX) / GYRO_SCALE;
  float gy = ((float)rawGY - gyroBiasY) / GYRO_SCALE;
  float gz = ((float)rawGZ - gyroBiasZ) / GYRO_SCALE;

  updateOrientation(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD,
                    ax, ay, az, mx, my, mz, dt);

  float rRad = rollDeg  * DEG_TO_RAD;
  float pRad = pitchDeg * DEG_TO_RAD;
  float linX = ax - (-sin(pRad));
  float linY = ay - ( sin(rRad) * cos(pRad));
  float linZ = az - ( cos(rRad) * cos(pRad));

  // IMU,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,roll,pitch,yaw
  Serial.print("IMU,");
  Serial.print(gx, 3); Serial.print(",");
  Serial.print(gy, 3); Serial.print(",");
  Serial.print(gz, 3); Serial.print(",");
  Serial.print(linX, 3); Serial.print(",");
  Serial.print(linY, 3); Serial.print(",");
  Serial.print(linZ, 3); Serial.print(",");
  Serial.print(rollDeg,  2); Serial.print(",");
  Serial.print(pitchDeg, 2); Serial.print(",");
  Serial.println(yawDeg, 2);
}

// ── Arduino entry points ──────────────────────────────────────────────────────

void setup() {
  Serial.begin(SERIAL_BAUD);
  // Arduino Uno: SDA=A4, SCL=A5 — default Wire pins, no args needed.
  Wire.begin();
  Wire.setClock(400000);
  initializeImu();
  lastImuMicros       = micros();
  lastTelemetryMicros = lastImuMicros;
}

void loop() {
  updateImuAndTelemetry();
}

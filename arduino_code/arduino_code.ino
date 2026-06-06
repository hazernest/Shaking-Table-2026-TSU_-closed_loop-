#include <Wire.h>

const uint8_t PULSE_PIN = 2;
const uint8_t DIRECTION_PIN = 3;
const uint8_t SDA_PIN = 20;
const uint8_t SCL_PIN = 21;

const unsigned long SERIAL_BAUD = 115200;
const unsigned int PULSE_HIGH_US = 10;
const unsigned long TELEMETRY_INTERVAL_US = 10000;
const uint8_t COMMAND_QUEUE_SIZE = 16;

const uint8_t MPU9250_ADDRESS = 0x68;
const uint8_t AK8963_ADDRESS = 0x0C;
const uint8_t MPU9250_PWR_MGMT_1 = 0x6B;
const uint8_t MPU9250_PWR_MGMT_2 = 0x6C;
const uint8_t MPU9250_SMPLRT_DIV = 0x19;
const uint8_t MPU9250_CONFIG = 0x1A;
const uint8_t MPU9250_GYRO_CONFIG = 0x1B;
const uint8_t MPU9250_ACCEL_CONFIG = 0x1C;
const uint8_t MPU9250_ACCEL_CONFIG2 = 0x1D;
const uint8_t MPU9250_INT_PIN_CFG = 0x37;
const uint8_t MPU9250_ACCEL_XOUT_H = 0x3B;
const uint8_t MPU9250_WHO_AM_I = 0x75;

const uint8_t AK8963_WHO_AM_I = 0x00;
const uint8_t AK8963_ST1 = 0x02;
const uint8_t AK8963_HXL = 0x03;
const uint8_t AK8963_CNTL1 = 0x0A;
const uint8_t AK8963_ASAX = 0x10;

const float ACCEL_SCALE = 8192.0f;
const float GYRO_SCALE = 65.5f;
const float MAG_SCALE = 0.15f;
const float COMPLEMENTARY_ALPHA = 0.98f;
const float YAW_MAG_ALPHA = 0.02f;
const int GYRO_CALIBRATION_SAMPLES = 500;

struct MotionCommand {
  long steps;
  long direction;
  unsigned long feedrate;
};

String inputBuffer;
MotionCommand commandQueue[COMMAND_QUEUE_SIZE];
uint8_t commandQueueHead = 0;
uint8_t commandQueueTail = 0;
uint8_t commandQueueCount = 0;

long commandedStepsRemaining = 0;
long currentDirection = 1;
unsigned long stepIntervalUs = 0;
unsigned long nextStepMicros = 0;
unsigned long pulseStartedMicros = 0;
bool pulseIsHigh = false;

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

float magOffsetX = 0.0f;
float magOffsetY = 0.0f;
float magOffsetZ = 0.0f;
float magScaleX = 1.0f;
float magScaleY = 1.0f;
float magScaleZ = 1.0f;

float rollDeg = 0.0f;
float pitchDeg = 0.0f;
float yawDeg = 0.0f;
unsigned long lastImuMicros = 0;
unsigned long lastTelemetryMicros = 0;

bool imuReady = false;

bool writeRegister(uint8_t deviceAddress, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

uint8_t readRegister(uint8_t deviceAddress, uint8_t reg) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return 0;
  }

  if (Wire.requestFrom(deviceAddress, static_cast<uint8_t>(1), static_cast<uint8_t>(true)) != 1) {
    return 0;
  }

  return Wire.read();
}

bool readRegisters(uint8_t deviceAddress, uint8_t reg, uint8_t* buffer, uint8_t length) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t received = Wire.requestFrom(deviceAddress, length, static_cast<uint8_t>(true));
  if (received != length) {
    return false;
  }

  for (uint8_t index = 0; index < length; index++) {
    buffer[index] = Wire.read();
  }

  return true;
}

bool readRawMpuData(int16_t& accelX, int16_t& accelY, int16_t& accelZ,
                    int16_t& gyroX, int16_t& gyroY, int16_t& gyroZ) {
  uint8_t buffer[14];
  if (!readRegisters(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, buffer, sizeof(buffer))) {
    return false;
  }

  accelX = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
  accelY = (static_cast<int16_t>(buffer[2]) << 8) | buffer[3];
  accelZ = (static_cast<int16_t>(buffer[4]) << 8) | buffer[5];
  gyroX = (static_cast<int16_t>(buffer[8]) << 8) | buffer[9];
  gyroY = (static_cast<int16_t>(buffer[10]) << 8) | buffer[11];
  gyroZ = (static_cast<int16_t>(buffer[12]) << 8) | buffer[13];
  return true;
}

bool readMagData(float& magX, float& magY, float& magZ) {
  uint8_t status = readRegister(AK8963_ADDRESS, AK8963_ST1);
  if ((status & 0x01) == 0) {
    return false;
  }

  uint8_t buffer[7];
  if (!readRegisters(AK8963_ADDRESS, AK8963_HXL, buffer, sizeof(buffer))) {
    return false;
  }

  if (buffer[6] & 0x08) {
    return false;
  }

  int16_t rawX = (static_cast<int16_t>(buffer[1]) << 8) | buffer[0];
  int16_t rawY = (static_cast<int16_t>(buffer[3]) << 8) | buffer[2];
  int16_t rawZ = (static_cast<int16_t>(buffer[5]) << 8) | buffer[4];

  magX = (static_cast<float>(rawX) * MAG_SCALE * magScaleX) - magOffsetX;
  magY = (static_cast<float>(rawY) * MAG_SCALE * magScaleY) - magOffsetY;
  magZ = (static_cast<float>(rawZ) * MAG_SCALE * magScaleZ) - magOffsetZ;
  return true;
}

bool normalizeVector(float& x, float& y, float& z) {
  float magnitude = sqrt(x * x + y * y + z * z);
  if (magnitude <= 0.0f) {
    return false;
  }

  x /= magnitude;
  y /= magnitude;
  z /= magnitude;
  return true;
}

float wrapAngleDeg(float angleDeg) {
  while (angleDeg > 180.0f) {
    angleDeg -= 360.0f;
  }

  while (angleDeg < -180.0f) {
    angleDeg += 360.0f;
  }

  return angleDeg;
}

float blendAngleDeg(float currentDeg, float targetDeg, float alpha) {
  float delta = wrapAngleDeg(targetDeg - currentDeg);
  return wrapAngleDeg(currentDeg + (alpha * delta));
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

void calibrateMagnetometerFactoryAdjustments() {
  writeRegister(AK8963_ADDRESS, AK8963_CNTL1, 0x00);
  delay(10);
  writeRegister(AK8963_ADDRESS, AK8963_CNTL1, 0x0F);
  delay(10);

  uint8_t asaX = readRegister(AK8963_ADDRESS, AK8963_ASAX);
  uint8_t asaY = readRegister(AK8963_ADDRESS, AK8963_ASAX + 1);
  uint8_t asaZ = readRegister(AK8963_ADDRESS, AK8963_ASAX + 2);

  magScaleX = (((static_cast<float>(asaX) - 128.0f) / 256.0f) + 1.0f);
  magScaleY = (((static_cast<float>(asaY) - 128.0f) / 256.0f) + 1.0f);
  magScaleZ = (((static_cast<float>(asaZ) - 128.0f) / 256.0f) + 1.0f);

  writeRegister(AK8963_ADDRESS, AK8963_CNTL1, 0x00);
  delay(10);
  writeRegister(AK8963_ADDRESS, AK8963_CNTL1, 0x16);
  delay(10);
}

void initializeImu() {
  uint8_t whoAmI = readRegister(MPU9250_ADDRESS, MPU9250_WHO_AM_I);
  if (whoAmI != 0x71 && whoAmI != 0x73) {
    Serial.print("IMU init warning: unexpected MPU9250 WHO_AM_I = 0x");
    Serial.println(whoAmI, HEX);
  }

  writeRegister(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x80);
  delay(100);
  writeRegister(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x01);
  writeRegister(MPU9250_ADDRESS, MPU9250_PWR_MGMT_2, 0x00);
  writeRegister(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x04);
  writeRegister(MPU9250_ADDRESS, MPU9250_CONFIG, 0x03);
  writeRegister(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 0x08);
  writeRegister(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0x08);
  writeRegister(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, 0x03);
  writeRegister(MPU9250_ADDRESS, MPU9250_INT_PIN_CFG, 0x02);
  delay(50);

  uint8_t magWhoAmI = readRegister(AK8963_ADDRESS, AK8963_WHO_AM_I);
  if (magWhoAmI != 0x48) {
    Serial.print("IMU init warning: unexpected AK8963 WHO_AM_I = 0x");
    Serial.println(magWhoAmI, HEX);
  }

  calibrateMagnetometerFactoryAdjustments();
  calibrateGyro();
  imuReady = true;
}

void updateOrientation(float gyroXRad, float gyroYRad, float gyroZRad,
                       float accelX, float accelY, float accelZ,
                       float magX, float magY, float magZ,
                       float deltaTime) {
  float accelRollDeg = atan2(accelY, accelZ) * RAD_TO_DEG;
  float accelPitchDeg = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * RAD_TO_DEG;

  rollDeg = COMPLEMENTARY_ALPHA * (rollDeg + gyroXRad * deltaTime * RAD_TO_DEG) + (1.0f - COMPLEMENTARY_ALPHA) * accelRollDeg;
  pitchDeg = COMPLEMENTARY_ALPHA * (pitchDeg + gyroYRad * deltaTime * RAD_TO_DEG) + (1.0f - COMPLEMENTARY_ALPHA) * accelPitchDeg;

  float rollRad = rollDeg * DEG_TO_RAD;
  float pitchRad = pitchDeg * DEG_TO_RAD;
  float magXComp = magX * cos(pitchRad) + magZ * sin(pitchRad);
  float magYComp = magX * sin(rollRad) * sin(pitchRad) + magY * cos(rollRad) - magZ * sin(rollRad) * cos(pitchRad);

  float headingDeg = atan2(-magYComp, magXComp) * RAD_TO_DEG;
  yawDeg = blendAngleDeg(yawDeg + gyroZRad * deltaTime * RAD_TO_DEG, headingDeg, YAW_MAG_ALPHA);
}

void emitTelemetry(float gyroX, float gyroY, float gyroZ,
                   float linearAccelX, float linearAccelY, float linearAccelZ) {
  Serial.print("IMU,");
  Serial.print(gyroX, 3);
  Serial.print(",");
  Serial.print(gyroY, 3);
  Serial.print(",");
  Serial.print(gyroZ, 3);
  Serial.print(",");
  Serial.print(linearAccelX, 3);
  Serial.print(",");
  Serial.print(linearAccelY, 3);
  Serial.print(",");
  Serial.print(linearAccelZ, 3);
  Serial.print(",");
  Serial.print(rollDeg, 2);
  Serial.print(",");
  Serial.print(pitchDeg, 2);
  Serial.print(",");
  Serial.println(yawDeg, 2);
}

void configureMove(long steps, long direction, unsigned long feedrate) {
  if (steps <= 0 || feedrate == 0) {
    return;
  }

  currentDirection = direction;
  digitalWrite(DIRECTION_PIN, currentDirection > 0 ? HIGH : LOW);

  commandedStepsRemaining = steps;
  stepIntervalUs = 1000000UL / feedrate;
  if (stepIntervalUs <= PULSE_HIGH_US) {
    stepIntervalUs = PULSE_HIGH_US + 1;
  }

  pulseIsHigh = false;
  nextStepMicros = micros();
}

bool enqueueMove(long steps, long direction, unsigned long feedrate) {
  if (commandQueueCount >= COMMAND_QUEUE_SIZE) {
    return false;
  }

  commandQueue[commandQueueTail].steps = steps;
  commandQueue[commandQueueTail].direction = direction;
  commandQueue[commandQueueTail].feedrate = feedrate;
  commandQueueTail = (commandQueueTail + 1) % COMMAND_QUEUE_SIZE;
  commandQueueCount++;
  return true;
}

uint8_t getQueueFreeSlots() {
  return COMMAND_QUEUE_SIZE - commandQueueCount;
}

uint8_t isStepperActive() {
  return (commandedStepsRemaining > 0 || pulseIsHigh) ? 1 : 0;
}

void emitQueueStatus() {
  Serial.print("QFREE,");
  Serial.print(getQueueFreeSlots());
  Serial.print(",");
  Serial.println(isStepperActive());
}

bool dequeueMove(MotionCommand& command) {
  if (commandQueueCount == 0) {
    return false;
  }

  command = commandQueue[commandQueueHead];
  commandQueueHead = (commandQueueHead + 1) % COMMAND_QUEUE_SIZE;
  commandQueueCount--;
  return true;
}

void handleCommand(const String& command) {
  if (command == "Q?") {
    emitQueueStatus();
    return;
  }

  int firstComma = command.indexOf(',');
  int secondComma = command.indexOf(',', firstComma + 1);

  if (firstComma < 0 || secondComma < 0) {
    return;
  }

  String stepText = command.substring(0, firstComma);
  String dirText = command.substring(firstComma + 1, secondComma);
  String feedrateText = command.substring(secondComma + 1);

  stepText.trim();
  dirText.trim();
  feedrateText.trim();

  long steps = stepText.toInt();
  long direction = dirText.toInt();
  long feedrate = feedrateText.toInt();

  if (steps <= 0) {
    return;
  }

  if (!(direction == -1 || direction == 0 || direction == 1)) {
    return;
  }

  if (feedrate <= 0) {
    return;
  }

  if (enqueueMove(steps, direction, static_cast<unsigned long>(feedrate))) {
    Serial.print("ENQ,");
    Serial.println(getQueueFreeSlots());
  } else {
    Serial.println("ENQ,FULL");
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char incoming = static_cast<char>(Serial.read());

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      if (inputBuffer.length() > 0) {
        handleCommand(inputBuffer);
        inputBuffer = "";
      }
      continue;
    }

    inputBuffer += incoming;
  }
}

void updateStepper() {
  unsigned long nowMicros = micros();

  if (commandedStepsRemaining <= 0 && !pulseIsHigh) {
    MotionCommand nextCommand;
    if (dequeueMove(nextCommand)) {
      configureMove(nextCommand.steps, nextCommand.direction, nextCommand.feedrate);
    }
  }

  if (pulseIsHigh) {
    if (nowMicros - pulseStartedMicros >= PULSE_HIGH_US) {
      digitalWrite(PULSE_PIN, LOW);
      pulseIsHigh = false;
    }
    return;
  }

  if (commandedStepsRemaining <= 0) {
    return;
  }

  if (nowMicros - nextStepMicros < stepIntervalUs && nextStepMicros != 0) {
    return;
  }

  digitalWrite(DIRECTION_PIN, currentDirection > 0 ? HIGH : LOW);
  digitalWrite(PULSE_PIN, HIGH);
  pulseStartedMicros = nowMicros;
  pulseIsHigh = true;
  nextStepMicros = nowMicros;
  commandedStepsRemaining--;
}

void updateImuAndTelemetry() {
  if (!imuReady) {
    return;
  }

  unsigned long nowMicros = micros();
  if (nowMicros - lastTelemetryMicros < TELEMETRY_INTERVAL_US) {
    return;
  }

  int16_t rawAccelX = 0;
  int16_t rawAccelY = 0;
  int16_t rawAccelZ = 0;
  int16_t rawGyroX = 0;
  int16_t rawGyroY = 0;
  int16_t rawGyroZ = 0;
  if (!readRawMpuData(rawAccelX, rawAccelY, rawAccelZ, rawGyroX, rawGyroY, rawGyroZ)) {
    lastTelemetryMicros = nowMicros;
    return;
  }

  float magX = 0.0f;
  float magY = 0.0f;
  float magZ = 0.0f;
  if (!readMagData(magX, magY, magZ)) {
    magX = 0.0f;
    magY = 0.0f;
    magZ = 1.0f;
  }

  float deltaTime = (nowMicros - lastImuMicros) * 0.000001f;
  lastImuMicros = nowMicros;
  lastTelemetryMicros = nowMicros;

  if (deltaTime <= 0.0f || deltaTime > 0.5f) {
    deltaTime = 0.01f;
  }

  float accelX = static_cast<float>(rawAccelX) / ACCEL_SCALE;
  float accelY = static_cast<float>(rawAccelY) / ACCEL_SCALE;
  float accelZ = static_cast<float>(rawAccelZ) / ACCEL_SCALE;

  float gyroX = (static_cast<float>(rawGyroX) - gyroBiasX) / GYRO_SCALE;
  float gyroY = (static_cast<float>(rawGyroY) - gyroBiasY) / GYRO_SCALE;
  float gyroZ = (static_cast<float>(rawGyroZ) - gyroBiasZ) / GYRO_SCALE;

  updateOrientation(
    gyroX * DEG_TO_RAD,
    gyroY * DEG_TO_RAD,
    gyroZ * DEG_TO_RAD,
    accelX,
    accelY,
    accelZ,
    magX,
    magY,
    magZ,
    deltaTime
  );

  float rollRad = rollDeg * DEG_TO_RAD;
  float pitchRad = pitchDeg * DEG_TO_RAD;
  float gravityX = -sin(pitchRad);
  float gravityY = sin(rollRad) * cos(pitchRad);
  float gravityZ = cos(rollRad) * cos(pitchRad);

  float linearAccelX = accelX - gravityX;
  float linearAccelY = accelY - gravityY;
  float linearAccelZ = accelZ - gravityZ;

  emitTelemetry(gyroX, gyroY, gyroZ, linearAccelX, linearAccelY, linearAccelZ);
}

void setup() {
  pinMode(PULSE_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  digitalWrite(PULSE_PIN, LOW);
  digitalWrite(DIRECTION_PIN, LOW);

  Serial.begin(SERIAL_BAUD);
  inputBuffer.reserve(48);

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_SAMD)
  Wire.begin(SDA_PIN, SCL_PIN);
#else
  Wire.begin();
#endif
  Wire.setClock(400000);

  initializeImu();

  lastImuMicros = micros();
  lastTelemetryMicros = lastImuMicros;
}

void loop() {
  readSerialCommands();
  updateStepper();
  updateImuAndTelemetry();
}
#include <Wire.h>

const uint8_t PULSE_PIN = 2;
const uint8_t DIRECTION_PIN = 3;
const uint8_t MPU6050_ADDRESS = 0x68;
const uint8_t MPU6050_PWR_MGMT_1 = 0x6B;
const uint8_t MPU6050_ACCEL_XOUT_H = 0x3B;
const uint8_t MPU6050_GYRO_CONFIG = 0x1B;
const uint8_t MPU6050_ACCEL_CONFIG = 0x1C;
const unsigned long SERIAL_BAUD = 115200;
const unsigned int PULSE_HIGH_US = 10;
const unsigned long TELEMETRY_INTERVAL_US = 10000;
const float ACCEL_SCALE = 16384.0f;
const float GYRO_SCALE = 131.0f;
const float COMPLEMENTARY_ALPHA = 0.98f;
const int GYRO_CALIBRATION_SAMPLES = 500;
const uint8_t COMMAND_QUEUE_SIZE = 16;
const int NOTE_C4 = 262;
const int NOTE_G3 = 196;
const int NOTE_A3 = 220;
const int NOTE_AS3 = 233;
const int NOTE_E4 = 330;
const int NOTE_F4 = 349;

const int STARTUP_MELODY[] = {
  NOTE_A3, NOTE_A3, NOTE_A3, NOTE_F4, NOTE_C4, NOTE_A3, NOTE_F4, NOTE_C4, NOTE_A3,
  NOTE_E4, NOTE_E4, NOTE_E4, NOTE_F4, NOTE_C4, NOTE_G3, NOTE_F4, NOTE_C4, NOTE_A3
};

const int STARTUP_NOTE_DURATIONS[] = {
  4, 4, 4, 8, 16, 4, 8, 16, 2,
  4, 4, 4, 8, 16, 4, 8, 16, 2
};

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
float rollDeg = 0.0f;
float pitchDeg = 0.0f;
float yawDeg = 0.0f;
unsigned long lastImuMicros = 0;
unsigned long lastTelemetryMicros = 0;

void playStartupTune() {
  digitalWrite(DIRECTION_PIN, HIGH);

  const int melodySize = sizeof(STARTUP_MELODY) / sizeof(STARTUP_MELODY[0]);
  for (int noteIndex = 0; noteIndex < melodySize; noteIndex++) {
    int noteDuration = 1000 / STARTUP_NOTE_DURATIONS[noteIndex];
    tone(PULSE_PIN, STARTUP_MELODY[noteIndex], noteDuration);

    int pauseBetweenNotes = static_cast<int>(noteDuration * 1.30f);
    delay(pauseBetweenNotes);
    noTone(PULSE_PIN);
  }

  digitalWrite(PULSE_PIN, LOW);
}

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

  enqueueMove(steps, direction, static_cast<unsigned long>(feedrate));
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

void updateImuAndTelemetry() {
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

  emitTelemetry(gyroX, gyroY, gyroZ, linearAccelX, linearAccelY, linearAccelZ);
}

void setup() {
  pinMode(PULSE_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  digitalWrite(PULSE_PIN, LOW);
  digitalWrite(DIRECTION_PIN, LOW);

  Serial.begin(SERIAL_BAUD);
  inputBuffer.reserve(48);

  Wire.begin();
  Wire.setClock(400000);
  writeRegister(MPU6050_PWR_MGMT_1, 0x00);
  writeRegister(MPU6050_ACCEL_CONFIG, 0x00);
  writeRegister(MPU6050_GYRO_CONFIG, 0x00);
  delay(100);

  calibrateGyro();
  playStartupTune();
  lastImuMicros = micros();
  lastTelemetryMicros = lastImuMicros;
}

void loop() {
  readSerialCommands();
  updateStepper();
  updateImuAndTelemetry();
}
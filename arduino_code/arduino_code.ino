const uint8_t PULSE_PIN = 2;
const uint8_t DIRECTION_PIN = 3;
const unsigned long SERIAL_BAUD = 115200;
const unsigned int PULSE_HIGH_US = 10;

String inputBuffer;

void setup() {
  pinMode(PULSE_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  digitalWrite(PULSE_PIN, LOW);
  digitalWrite(DIRECTION_PIN, LOW);

  Serial.begin(SERIAL_BAUD);
  inputBuffer.reserve(48);
}

void loop() {
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

  if (!(direction == 0 || direction == 1 || direction == -1)) {
    return;
  }

  if (feedrate <= 0) {
    return;
  }

  bool directionHigh = direction > 0;
  digitalWrite(DIRECTION_PIN, directionHigh ? HIGH : LOW);
  runSteps(steps, static_cast<unsigned long>(feedrate));
}

void runSteps(long steps, unsigned long feedrate) {
  unsigned long stepPeriodUs = 1000000UL / feedrate;
  if (stepPeriodUs <= PULSE_HIGH_US) {
    stepPeriodUs = PULSE_HIGH_US + 1;
  }

  unsigned long pulseLowUs = stepPeriodUs - PULSE_HIGH_US;

  for (long currentStep = 0; currentStep < steps; currentStep++) {
    digitalWrite(PULSE_PIN, HIGH);
    delayMicroseconds(PULSE_HIGH_US);
    digitalWrite(PULSE_PIN, LOW);
    delayMicroseconds(pulseLowUs);
  }
}
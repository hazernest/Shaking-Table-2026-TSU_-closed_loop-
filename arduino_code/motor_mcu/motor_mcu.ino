// motor_mcu.ino — Arduino Mega
// Stepper motor control only. No IMU.
// Protocol:
//   Receive:  steps,direction,feedrate\n   (feedrate = steps/sec)
//   Receive:  Q?\n
//   Send:     QFREE,<free_slots>,<is_active>\n
//   Send:     ENQ,<free_slots>\n  or  ENQ,FULL\n
//   Send:     ok\n   (when a command finishes executing)

const uint8_t PULSE_PIN      = 2;
const uint8_t DIRECTION_PIN  = 3;

const unsigned long SERIAL_BAUD     = 115200;
const unsigned int  PULSE_HIGH_US   = 10;
const uint8_t       COMMAND_QUEUE_SIZE = 16;

struct MotionCommand {
  long          steps;
  long          direction;
  unsigned long feedrate;
};

String        inputBuffer;
MotionCommand commandQueue[COMMAND_QUEUE_SIZE];
uint8_t       commandQueueHead  = 0;
uint8_t       commandQueueTail  = 0;
uint8_t       commandQueueCount = 0;

long          commandedStepsRemaining = 0;
long          currentDirection        = 1;
unsigned long stepIntervalUs          = 0;
unsigned long nextStepMicros          = 0;
unsigned long pulseStartedMicros      = 0;
bool          pulseIsHigh             = false;
bool          commandRunning          = false;  // true while a command is executing

// ── queue helpers ─────────────────────────────────────────────────────────────

bool enqueueMove(long steps, long direction, unsigned long feedrate) {
  if (commandQueueCount >= COMMAND_QUEUE_SIZE) {
    return false;
  }
  commandQueue[commandQueueTail].steps     = steps;
  commandQueue[commandQueueTail].direction = direction;
  commandQueue[commandQueueTail].feedrate  = feedrate;
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

// ── stepper execution ─────────────────────────────────────────────────────────

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
  pulseIsHigh      = false;
  nextStepMicros   = micros();
  commandRunning   = true;
}

void updateStepper() {
  unsigned long nowMicros = micros();

  if (commandedStepsRemaining <= 0 && !pulseIsHigh) {
    // A command just finished — emit ok before starting the next one
    if (commandRunning) {
      Serial.println("ok");
      commandRunning = false;
    }
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
  pulseIsHigh        = true;
  nextStepMicros     = nowMicros;
  commandedStepsRemaining--;
}

// ── serial command parser ─────────────────────────────────────────────────────

void handleCommand(const String& command) {
  if (command == "Q?") {
    emitQueueStatus();
    return;
  }

  int firstComma  = command.indexOf(',');
  int secondComma = command.indexOf(',', firstComma + 1);

  if (firstComma < 0 || secondComma < 0) {
    return;
  }

  String stepText     = command.substring(0, firstComma);
  String dirText      = command.substring(firstComma + 1, secondComma);
  String feedrateText = command.substring(secondComma + 1);

  stepText.trim();
  dirText.trim();
  feedrateText.trim();

  long steps     = stepText.toInt();
  long direction = dirText.toInt();
  long feedrate  = feedrateText.toInt();

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

// ── Arduino entry points ──────────────────────────────────────────────────────

void setup() {
  pinMode(PULSE_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  digitalWrite(PULSE_PIN, LOW);
  digitalWrite(DIRECTION_PIN, LOW);

  Serial.begin(SERIAL_BAUD);
  inputBuffer.reserve(48);
}

void loop() {
  readSerialCommands();
  updateStepper();
}

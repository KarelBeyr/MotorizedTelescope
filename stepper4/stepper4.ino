
#include <Arduino.h>

const byte STEP_PIN = 2;
const byte DIR_PIN = 3;

// TB6600-style driver, common-anode wiring
// LOW = active pulse, HIGH = idle
const unsigned long PULSE_WIDTH_US = 10;
const unsigned long DIR_SETUP_US = 20;

// Movement configuration
const long SMALL_MOVEMENT = 200;
const long LARGE_MOVEMENT = 2000;

const unsigned int SLOW_SPEED = 400;
const unsigned int FAST_SPEED = 2000;

// Motion state
long stepsRemaining = 0;
long position = 0;

unsigned int currentSpeed = SLOW_SPEED;

bool moving = false;
bool pulseActive = false;

unsigned long nextStepTime = 0;
unsigned long pulseStartTime = 0;


void stopMotor() {
  digitalWrite(STEP_PIN, HIGH);

  pulseActive = false;
  moving = false;
  stepsRemaining = 0;

  Serial.println(F("STOPPED"));
}


void printStatus() {
  Serial.print(F("Position: "));
  Serial.println(position);

  Serial.print(F("Speed: "));
  Serial.print(currentSpeed);
  Serial.println(F(" steps/s"));

  Serial.print(F("Remaining: "));
  Serial.println(stepsRemaining);

  Serial.println(moving ? F("MOVING") : F("IDLE"));
}


void startRotation(long steps, unsigned int speed) {
  if (moving) {
    Serial.println(F("ERR: Motor busy"));
    return;
  }

  if (steps == 0 || speed == 0)
    return;

  currentSpeed = speed;

  // Set direction
  digitalWrite(DIR_PIN, steps > 0 ? HIGH : LOW);

  stepsRemaining = steps;
  moving = true;
  pulseActive = false;

  // Allow direction signal to settle
  nextStepTime = micros() + DIR_SETUP_US;

  Serial.print(F("ROTATE "));
  Serial.print(steps);
  Serial.print(F(" at "));
  Serial.print(speed);
  Serial.println(F(" steps/s"));
}


void readSerial() {
  while (Serial.available() > 0) {
    char command = Serial.read();

    switch (command) {

      case 'l':
        startRotation(SMALL_MOVEMENT, SLOW_SPEED);
        break;

      case 'L':
        startRotation(LARGE_MOVEMENT, FAST_SPEED);
        break;

      case 'r':
        startRotation(-SMALL_MOVEMENT, SLOW_SPEED);
        break;

      case 'R':
        startRotation(-LARGE_MOVEMENT, FAST_SPEED);
        break;

      case 's':
        stopMotor();
        break;

      case 'c':
        if (!moving) {
          position = 0;
          Serial.println(F("POSITION RESET"));
        } else {
          Serial.println(F("ERR: Motor busy"));
        }
        break;

      case '?':
        printStatus();
        break;

      default:
        // Ignore other characters
        break;
    }
  }
}


void updateMotor() {
  if (!moving)
    return;

  unsigned long now = micros();

  // Finish active pulse
  if (pulseActive) {
    if ((unsigned long)(now - pulseStartTime)
        >= PULSE_WIDTH_US) {

      digitalWrite(STEP_PIN, HIGH);
      pulseActive = false;

      if (stepsRemaining == 0) {
        moving = false;

        Serial.print(F("DONE Position: "));
        Serial.println(position);
      }
    }
    return;
  }

  // Wait for next pulse
  if ((long)(now - nextStepTime) < 0)
    return;

  // Start new pulse
  digitalWrite(STEP_PIN, LOW);

  pulseStartTime = now;
  pulseActive = true;

  if (stepsRemaining > 0) {
    stepsRemaining--;
    position++;
  } else {
    stepsRemaining++;
    position--;
  }

  // Schedule next pulse from current time
  unsigned long interval = 1000000UL / currentSpeed;
  nextStepTime = now + interval;
}


void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(STEP_PIN, HIGH);
  digitalWrite(DIR_PIN, HIGH);

  Serial.begin(115200);

  Serial.println(F("Telescope controller ready"));
  Serial.println(F("Commands:"));
  Serial.println(F("  l  Left 200 steps (slow)"));
  Serial.println(F("  r  Right 200 steps (slow)"));
  Serial.println(F("  L  Left 2000 steps (fast)"));
  Serial.println(F("  R  Right 2000 steps (fast)"));
  Serial.println(F("  s  Stop"));
  Serial.println(F("  c  Reset position"));
  Serial.println(F("  ?  Status"));
}


void loop() {
  readSerial();
  updateMotor();
}

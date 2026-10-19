
#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>

// =====================================================
// MOVEMENT SETTINGS
// =====================================================

constexpr int SMALL_MOVEMENT = 200;
constexpr int LARGE_MOVEMENT = 2000;

constexpr uint32_t SLOW_SPEED = 400;
constexpr uint32_t FAST_SPEED = 2000;
constexpr uint32_t ACCELERATION = 2500;

constexpr uint16_t AZ_CURRENT_MA = 350;
constexpr uint16_t EL_CURRENT_MA = 350;

constexpr float AZ_HOLD_MULT = 0.7f;
constexpr float EL_HOLD_MULT = 1.0f;

constexpr uint16_t MICROSTEPS = 16;
constexpr float R_SENSE = 0.11f;
constexpr uint32_t UART_BAUD = 115200;

// Reverse these if movement direction is incorrect.
constexpr bool AZ_REVERSED = true;
constexpr bool EL_REVERSED = false;

// =====================================================
// ESP32 PIN ASSIGNMENTS
// =====================================================

// Azimuth
constexpr int AZ_STEP = 26;
constexpr int AZ_DIR  = 25;
constexpr int AZ_EN   = 27;
constexpr int AZ_RX   = 32;
constexpr int AZ_TX   = 33;

// Elevation
constexpr int EL_STEP = 18;
constexpr int EL_DIR  = 19;
constexpr int EL_EN   = 23;
constexpr int EL_RX   = 16;
constexpr int EL_TX   = 17;

// =====================================================
// DRIVERS
// =====================================================

HardwareSerial azUart(1);
HardwareSerial elUart(2);

TMC2209Stepper azDriver(&azUart, R_SENSE, 0);
TMC2209Stepper elDriver(&elUart, R_SENSE, 0);

FastAccelStepperEngine engine;

FastAccelStepper* azMotor = nullptr;
FastAccelStepper* elMotor = nullptr;

// =====================================================
// INITIALIZATION
// =====================================================

void configureDriver(
    TMC2209Stepper& driver,
    uint16_t current,
    float holdMultiplier) {
  driver.begin();

  driver.pdn_disable(true);
  driver.I_scale_analog(false);

  // Configure microstepping through UART
  driver.mstep_reg_select(true);
  driver.microsteps(MICROSTEPS);

  driver.rms_current(current, holdMultiplier);

  // Quiet StealthChop operation
  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);

  // Enable chopper
  driver.toff(4);
}

FastAccelStepper* configureMotor(
    int stepPin, int dirPin, int enablePin,
    bool reversed) {

  FastAccelStepper* motor =
      engine.stepperConnectToPin(stepPin);

  if (!motor) {
    Serial.println("ERROR: Cannot create stepper!");
    return nullptr;
  }

  motor->setDirectionPin(dirPin, !reversed);
  motor->setEnablePin(enablePin, true);

  // Keep holding torque after movement.
  // Enable manually before the first movement.
  motor->setAutoEnable(false);

  motor->setSpeedInHz(SLOW_SPEED);
  motor->setAcceleration(ACCELERATION);
  motor->setCurrentPosition(0);

  return motor;
}

// =====================================================
// MOVEMENT
// =====================================================

void moveMotor(
    FastAccelStepper* motor,
    int32_t steps,
    uint32_t speed) {

  if (!motor) return;

  motor->setSpeedInHz(speed);
  motor->setAcceleration(ACCELERATION);

  // Enable motor, then allow the driver to settle.
  motor->enableOutputs();
  delay(2);

  auto result = motor->move(steps);

  if (result != MOVE_OK) {
    Serial.printf("Move error: %d\n", (int)result);
  }
}

void printPosition() {
  Serial.printf(
    "AZ=%ld (%s), EL=%ld (%s)\n",
    (long)azMotor->getCurrentPosition(),
    azMotor->isRunning() ? "moving" : "idle",
    (long)elMotor->getCurrentPosition(),
    elMotor->isRunning() ? "moving" : "idle"
  );
}

void setCenter() {
  if (azMotor->isRunning() || elMotor->isRunning()) {
    Serial.println("Wait for motors to stop before centering.");
    return;
  }

  azMotor->setCurrentPosition(0);
  elMotor->setCurrentPosition(0);

  Serial.println("Center set: AZ=0, EL=0");
}

void returnToCenter() {
  if (azMotor->isRunning() || elMotor->isRunning()) {
    Serial.println("Stop both motors first.");
    return;
  }

  azMotor->setSpeedInHz(FAST_SPEED);
  elMotor->setSpeedInHz(FAST_SPEED);

  azMotor->enableOutputs();
  elMotor->enableOutputs();
  delay(2);

  azMotor->moveTo(0);
  elMotor->moveTo(0);

  Serial.println("Returning to center...");
}

// =====================================================
// SERIAL COMMANDS
// =====================================================

void handleCommand(char cmd) {
  int32_t small = SMALL_MOVEMENT;
  int32_t large = LARGE_MOVEMENT;

  switch (cmd) {

    // AZIMUTH
    case 'l':
      moveMotor(azMotor, -small, SLOW_SPEED);
      break;

    case 'L':
      moveMotor(azMotor, -large, FAST_SPEED);
      break;

    case 'r':
      moveMotor(azMotor, small, SLOW_SPEED);
      break;

    case 'R':
      moveMotor(azMotor, large, FAST_SPEED);
      break;

    // ELEVATION
    case 'u':
      moveMotor(elMotor, small, SLOW_SPEED);
      break;

    case 'U':
      moveMotor(elMotor, large, FAST_SPEED);
      break;

    case 'd':
      moveMotor(elMotor, -small, SLOW_SPEED);
      break;

    case 'D':
      moveMotor(elMotor, -large, FAST_SPEED);
      break;

    // POSITION COMMANDS
    case 'c':
      setCenter();
      break;

    case 'h':
      returnToCenter();
      break;

    case 'p':
      printPosition();
      break;

    case 's':
      azMotor->stopMove();
      elMotor->stopMove();
      Serial.println("Stopping both motors...");
      break;

    case '\n':
    case '\r':
    case ' ':
      return;

    default:
      Serial.printf("Unknown command: %c\n", cmd);
      return;
  }

  printPosition();
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== Panorama Controller ===");

  // Keep motors disabled during initialization.
  digitalWrite(AZ_EN, HIGH);
  digitalWrite(EL_EN, HIGH);
  pinMode(AZ_EN, OUTPUT);
  pinMode(EL_EN, OUTPUT);

  // Initialize UARTs
  azUart.begin(UART_BAUD, SERIAL_8N1, AZ_RX, AZ_TX);
  elUart.begin(UART_BAUD, SERIAL_8N1, EL_RX, EL_TX);

  delay(100);

  // Configure TMC2209 drivers
  configureDriver(azDriver, AZ_CURRENT_MA, AZ_HOLD_MULT);
  configureDriver(elDriver, EL_CURRENT_MA, EL_HOLD_MULT);

  Serial.printf("AZ UART: 0x%02X\n", azDriver.version());
  Serial.printf("EL UART: 0x%02X\n", elDriver.version());

  if (azDriver.test_connection() != 0 ||
      elDriver.test_connection() != 0) {
    Serial.println("ERROR: Driver UART failure!");
    Serial.println("Motors will remain disabled.");
    while (true) delay(1000);
  }

  // Initialize step generation
  engine.init();

  azMotor = configureMotor(
      AZ_STEP, AZ_DIR, AZ_EN, AZ_REVERSED);

  elMotor = configureMotor(
      EL_STEP, EL_DIR, EL_EN, EL_REVERSED);

  if (!azMotor || !elMotor) {
    Serial.println("ERROR: Motor initialization failed!");
    while (true) delay(1000);
  }

  Serial.println("Both motors ready!");
  Serial.println("l/L r/R u/U d/D: Move");
  Serial.println("c: Set center");
  Serial.println("h: Return to center");
  Serial.println("s: Stop");
  Serial.println("p: Position");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  while (Serial.available()) {
    char cmd = Serial.read();
    handleCommand(cmd);
  }

  // Motors run independently in the background.
  // No stepping code needed here!
}

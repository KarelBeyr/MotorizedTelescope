
#include <Arduino.h>
#include <TMCStepper.h>

// ESP32 pins
constexpr uint8_t STEP_PIN = 13;
constexpr uint8_t DIR_PIN  = 14;
constexpr uint8_t EN_PIN   = 27;

// TMC2209 UART
constexpr uint8_t UART_RX = 26;
constexpr uint8_t UART_TX = 25;

// BigTreeTech TMC2209 V1.3
constexpr float R_SENSE = 0.11f;
constexpr uint8_t DRIVER_ADDRESS = 0;

// Motor settings
constexpr uint16_t MOTOR_CURRENT = 500; // mA RMS
constexpr uint16_t MICROSTEPS = 32;

// Test movement
constexpr int SPEED = 8000; // microsteps per second
constexpr int STEPS_PER_MOVE = 6400;

HardwareSerial TMCSerial(2);

TMC2209Stepper driver(
  &TMCSerial,
  R_SENSE,
  DRIVER_ADDRESS
);

bool driverReady = false;

void rotate(bool direction) {
  digitalWrite(DIR_PIN, direction ? HIGH : LOW);
  delay(10);

  const unsigned long halfPeriod =
      1000000UL / SPEED / 2;

  for (int i = 0; i < STEPS_PER_MOVE; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(halfPeriod);

    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(halfPeriod);
  }
}

void setup() {
  // Keep motor disabled during initialization
  digitalWrite(EN_PIN, HIGH);
  pinMode(EN_PIN, OUTPUT);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== Telescope motor test ===");

  // Initialize UART
  TMCSerial.begin(
    115200,
    SERIAL_8N1,
    UART_RX,
    UART_TX
  );

  driver.begin();

  // Check communication before configuring motor
  uint8_t result = driver.test_connection();

  Serial.print("UART result: ");
  Serial.println(result);

  Serial.print("Driver version: 0x");
  Serial.println(driver.version(), HEX);

  if (result != 0 || driver.version() != 0x21) {
    Serial.println("ERROR: TMC2209 not responding!");
    Serial.println("Motor remains disabled.");
    return;
  }

  Serial.println("UART communication OK");

  // Configure driver via UART
  driver.pdn_disable(true);
  driver.I_scale_analog(false);
  driver.mstep_reg_select(true);

  // Enable driver chopper
  driver.toff(4);

  // Motor current - no potentiometer needed!
  driver.rms_current(MOTOR_CURRENT);

  // Microstepping
  driver.microsteps(MICROSTEPS);

  // StealthChop - quiet operation
  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);

  // Verify settings through UART
  if (driver.microsteps() != MICROSTEPS ||
      driver.test_connection() != 0) {
    Serial.println("ERROR: Driver configuration failed!");
    Serial.println("Motor remains disabled.");
    return;
  }

  Serial.print("Motor current: ");
  Serial.print(MOTOR_CURRENT);
  Serial.println(" mA RMS");

  Serial.print("Microstepping: 1/");
  Serial.println(MICROSTEPS);

  Serial.println("Enabling motor...");

  // EN is active LOW
  digitalWrite(EN_PIN, LOW);

  driverReady = true;

  Serial.println("Ready!");
}


void loop() {
  if (!driverReady) {
    Serial.println("Not Ready!");
    delay(1000);
    return;
  }

  Serial.println("Clockwise - 360 degrees");
  rotate(true);

  delay(1000);

  Serial.println("Counterclockwise - 360 degrees");
  rotate(false);

  delay(1000);
}

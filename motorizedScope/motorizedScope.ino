
#include <Arduino.h>
#include <TMCStepper.h>

constexpr float R_SENSE = 0.11f;
constexpr uint32_t UART_BAUD = 115200;

// Motor 1: Azimuth
constexpr int AZ_STEP = 26;
constexpr int AZ_DIR  = 25;
constexpr int AZ_EN   = 27;
constexpr int AZ_RX   = 32;
constexpr int AZ_TX   = 33;

// Motor 2: Elevation
constexpr int EL_STEP = 18;
constexpr int EL_DIR  = 19;
constexpr int EL_EN   = 23;
constexpr int EL_RX   = 16;
constexpr int EL_TX   = 17;

// Two independent hardware UARTs
HardwareSerial azUart(1);
HardwareSerial elUart(2);

// Both use address 0 because UARTs are separate
TMC2209Stepper azDriver(&azUart, R_SENSE, 0);
TMC2209Stepper elDriver(&elUart, R_SENSE, 0);

void initPins(int step, int dir, int enable) {
  // Disable the motor before configuring movement pins
  digitalWrite(enable, HIGH);
  pinMode(enable, OUTPUT);

  digitalWrite(step, LOW);
  pinMode(step, OUTPUT);
  pinMode(dir, OUTPUT);
}

void initDriver(TMC2209Stepper &driver) {
  driver.begin();
  driver.pdn_disable(true);
  driver.I_scale_analog(false);
  driver.mstep_reg_select(true);
}

void checkDriver(const char* name, TMC2209Stepper &driver) {
  uint8_t version = driver.version();

  Serial.printf("%s: TMC version = 0x%02X\n",
                name, version);

  if (version == 0x21) {
    Serial.printf("%s: UART OK!\n", name);
  } else {
    Serial.printf("%s: Check UART wiring and power.\n", name);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  initPins(AZ_STEP, AZ_DIR, AZ_EN);
  initPins(EL_STEP, EL_DIR, EL_EN);

  // UART1 and UART2 on our chosen GPIO pins
  azUart.begin(UART_BAUD, SERIAL_8N1, AZ_RX, AZ_TX);
  elUart.begin(UART_BAUD, SERIAL_8N1, EL_RX, EL_TX);

  // Both drivers must have VIO and VS power
  delay(100);

  initDriver(azDriver);
  initDriver(elDriver);

  checkDriver("AZIMUTH", azDriver);
  checkDriver("ELEVATION", elDriver);

  Serial.println("Initialization complete.");
}

void loop() {
  Serial.printf(
    "AZ: version=0x%02X, connection=%d | "
    "EL: version=0x%02X, connection=%d\n",
    azDriver.version(),
    azDriver.test_connection(),
    elDriver.version(),
    elDriver.test_connection()
  );

  delay(1000);
}

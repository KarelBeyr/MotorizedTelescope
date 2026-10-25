#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include <BleKeyboard.h>
#include <stdarg.h>
#include <stdio.h>

// ============================================================
// USER SETTINGS
// ============================================================
// IMPORTANT: Calibrate these separately from your camera's field of view
// and desired photo overlap. These are EXAMPLE pitches, NOT calculated
// for your telescope; the automatic scan will use these as maximum spacings.
constexpr uint32_t FRAME_AZ_PITCH = 70;    // microsteps between columns
constexpr uint32_t FRAME_EL_PITCH = 35;    // microsteps between rows

// Manual movement sizes, independent for azimuth and elevation.
constexpr int32_t AZ_SMALL_MOVEMENT = FRAME_AZ_PITCH;
constexpr int32_t AZ_LARGE_MOVEMENT = 2000;
constexpr int32_t EL_SMALL_MOVEMENT = FRAME_EL_PITCH;
constexpr int32_t EL_LARGE_MOVEMENT = 400;

// Manual movement speeds. Uppercase retains the faster movement.
constexpr uint32_t SLOW_SPEED = 400;        // microsteps / second
constexpr uint32_t FAST_SPEED = 2000;       // microsteps / second
constexpr uint32_t ACCELERATION = 2500;     // microsteps / second^2

// Frame-to-frame scanning is deliberately gentle.
// Initial return to home (g) and manual return (h) use FAST_SPEED.
constexpr uint32_t AZ_SCAN_SPEED = 400;     // microsteps / second
constexpr uint32_t EL_SCAN_SPEED = 300;     // microsteps / second
constexpr uint32_t AZ_SCAN_ACCELERATION = 500; // microsteps / second^2
constexpr uint32_t EL_SCAN_ACCELERATION = 400; // microsteps / second^2

constexpr uint32_t SETTLE_MS = 3000;        // after motion, before shutter
constexpr uint32_t DRY_RUN_PAUSE_MS = 250;  // per-frame pause in dry-run mode
constexpr uint32_t AFTER_SHOT_MS = 1000;    // after shutter, before movement
constexpr uint32_t MAX_FRAMES = 1500;       // accidental-scan safety cap

constexpr uint16_t AZ_CURRENT_MA = 350;
constexpr uint16_t EL_CURRENT_MA = 350;
constexpr float AZ_HOLD_MULT = 1.00f;
constexpr float EL_HOLD_MULT = 1.00f;

constexpr uint16_t MICROSTEPS = 16;
constexpr float R_SENSE = 0.11f;
constexpr uint32_t UART_BAUD = 115200;

// Right was reversed during initial tests.
constexpr bool AZ_REVERSED = true;
constexpr bool EL_REVERSED = false;

// ============================================================
// PINOUT -- unchanged from the working two-UART firmware
// ============================================================
constexpr int AZ_STEP = 26, AZ_DIR = 25, AZ_EN = 27;
constexpr int AZ_RX = 32, AZ_TX = 33;
constexpr int EL_STEP = 18, EL_DIR = 19, EL_EN = 23;
constexpr int EL_RX = 16, EL_TX = 17;

HardwareSerial azUart(1), elUart(2);
TMC2209Stepper azDriver(&azUart, R_SENSE, 0);
TMC2209Stepper elDriver(&elUart, R_SENSE, 0);
FastAccelStepperEngine engine;
FastAccelStepper *azMotor = nullptr, *elMotor = nullptr;

// Keep the name that is already paired with the Android phone.
BleKeyboard bleKeyboard("Panorama Shut2", "DIY", 100);

// ============================================================
// SERIAL LOGGING -- always CR + LF (suitable for PuTTY)
// ============================================================
void logf(const char *fmt, ...) {
  char message[192];
  va_list args;
  va_start(args, fmt);
  vsnprintf(message, sizeof(message), fmt, args);
  va_end(args);
  Serial.print(message);
  Serial.print("\r\n");
}

bool motorsMoving() {
  return azMotor->isRunning() || elMotor->isRunning();
}

void printPosition() {
  logf("AZ=%ld (%s), EL=%ld (%s)",
       (long)azMotor->getCurrentPosition(),
       azMotor->isRunning() ? "moving" : "idle",
       (long)elMotor->getCurrentPosition(),
       elMotor->isRunning() ? "moving" : "idle");
}

void configureDriver(TMC2209Stepper &driver,
                     uint16_t currentMa, float holdFraction) {
  driver.begin();
  driver.pdn_disable(true);
  driver.I_scale_analog(false);
  driver.mstep_reg_select(true);
  driver.microsteps(MICROSTEPS);
  driver.rms_current(currentMa, holdFraction);
  driver.en_spreadCycle(false);       // StealthChop
  driver.pwm_autoscale(true);
  driver.toff(4);
}

FastAccelStepper *configureMotor(int stepPin, int dirPin,
                                 int enablePin, bool reversed) {
  FastAccelStepper *motor = engine.stepperConnectToPin(stepPin);
  if (!motor) return nullptr;
  motor->setDirectionPin(dirPin, !reversed);
  motor->setEnablePin(enablePin, true);
  motor->setAutoEnable(false);        // retain holding torque
  motor->setSpeedInHz(SLOW_SPEED);
  motor->setAcceleration(ACCELERATION);
  motor->setCurrentPosition(0);
  return motor;
}

void enableMotors() {
  azMotor->enableOutputs();
  elMotor->enableOutputs();
  delay(2);
}

// ============================================================
// BLUETOOTH SHUTTER
// ============================================================
bool triggerShutter() {
  if (!bleKeyboard.isConnected()) {
    logf("BLE not connected: photo NOT taken.");
    return false;
  }

  // In the Pixel Camera settings, configure volume button as Shutter.
  bleKeyboard.press(KEY_MEDIA_VOLUME_UP);
  delay(100);                         // reliable short press
  bleKeyboard.releaseAll();
  return true;
}

// ============================================================
// PANORAMA STATE MACHINE
// ============================================================
enum class ScanState : uint8_t {
  Idle, Moving, Settling, WaitingForBluetooth, AfterShot
};

ScanState scanState = ScanState::Idle;
bool homeDefined = false;
bool targetDefined = false;
int32_t targetAz = 0, targetEl = 0;
uint32_t columns = 0, rows = 0;
uint32_t photosTaken = 0;
uint32_t stageStartedAt = 0;
bool manualMovementPending = false;
bool dryRunMode = false;                  // 'y' toggles this while idle


bool scanActive() { return scanState != ScanState::Idle; }

uint32_t pointsForRange(int32_t extent, uint32_t maxPitch) {
  const uint64_t distance = extent < 0 ? uint64_t(-(int64_t)extent)
                                       : uint64_t(extent);
  return (uint32_t)((distance + maxPitch - 1) / maxPitch + 1);
}

int32_t interpolatedPosition(int32_t endpoint, uint32_t index,
                             uint32_t pointCount) {
  if (pointCount <= 1) return 0;
  return (int32_t)(((int64_t)endpoint * index) / (pointCount - 1));
}

// A photo index represents a snake traversal of a rectangular grid.
void getFramePosition(uint32_t photoIndex, int32_t &az, int32_t &el) {
  uint32_t row = photoIndex / columns;
  uint32_t travelCol = photoIndex % columns;
  uint32_t physicalCol = (row % 2 == 0)
                             ? travelCol
                             : columns - 1 - travelCol;
  az = interpolatedPosition(targetAz, physicalCol, columns);
  el = interpolatedPosition(targetEl, row, rows);
}

bool commandAbsoluteMove(int32_t az, int32_t el, bool fastTravel = false) {
  enableMotors();
  // Fast for homing journeys, gentle between panorama frames.
  azMotor->setSpeedInHz(fastTravel ? FAST_SPEED : AZ_SCAN_SPEED);
  elMotor->setSpeedInHz(fastTravel ? FAST_SPEED : EL_SCAN_SPEED);
  azMotor->setAcceleration(fastTravel ? ACCELERATION : AZ_SCAN_ACCELERATION);
  elMotor->setAcceleration(fastTravel ? ACCELERATION : EL_SCAN_ACCELERATION);

  auto resultAz = azMotor->moveTo(az);
  auto resultEl = elMotor->moveTo(el);
  if (resultAz != MOVE_OK || resultEl != MOVE_OK) {
    logf("MOVE ERROR: AZ=%d EL=%d", (int)resultAz, (int)resultEl);
    azMotor->stopMove();
    elMotor->stopMove();
    return false;
  }
  return true;
}

void cancelScan(bool stopMotors) {
  bool wasScanning = scanActive();
  scanState = ScanState::Idle;
  if (stopMotors) {
    azMotor->stopMove();
    elMotor->stopMove();
  }
  if (wasScanning) {
    logf("Scan canceled after %lu photos.", (unsigned long)photosTaken);
  }
}

void goToFrame(uint32_t index) {
  int32_t az, el;
  getFramePosition(index, az, el);
  if (!commandAbsoluteMove(az, el, index == 0)) {
    cancelScan(false);
    return;
  }
  scanState = ScanState::Moving;
  logf("Frame %lu/%lu: move to AZ=%ld EL=%ld",
       (unsigned long)(index + 1),
       (unsigned long)(columns * rows),
       (long)az, (long)el);
}

void startScan() {
  if (scanActive()) {
    logf("Scan is already active.");
    return;
  }
  if (!homeDefined || !targetDefined) {
    logf("First press c at top-left, then b at bottom-right.");
    return;
  }
  if (motorsMoving()) {
    logf("Wait for motors to stop before starting scan.");
    return;
  }
  if (!dryRunMode && !bleKeyboard.isConnected()) {
    logf("BLE shutter disconnected; connect your phone first (or press y for dry run).");
    return;
  }
  columns = pointsForRange(targetAz, FRAME_AZ_PITCH);
  rows = pointsForRange(targetEl, FRAME_EL_PITCH);
  uint64_t total = (uint64_t)columns * rows;
  if (total < 2 || total > MAX_FRAMES) {
    logf("Invalid grid %lu x %lu (%llu frames). Check pitches/target.",
         (unsigned long)columns, (unsigned long)rows,
         (unsigned long long)total);
    return;
  }

  photosTaken = 0;
  manualMovementPending = false;
  logf("Starting %s: %lu columns x %lu rows (%llu frames).",
       dryRunMode ? "DRY RUN" : "PHOTO SCAN",
       (unsigned long)columns, (unsigned long)rows,
       (unsigned long long)total);
  logf("Frame wait: %lu ms%s",
       (unsigned long)(dryRunMode ? DRY_RUN_PAUSE_MS : SETTLE_MS),
       dryRunMode ? "; BLE shutter disabled" : "; BLE shutter enabled");
  logf("Bounds AZ=[%ld..%ld] EL=[%ld..%ld] (commanded microsteps)",
       (long)(targetAz < 0 ? targetAz : 0),
       (long)(targetAz > 0 ? targetAz : 0),
       (long)(targetEl < 0 ? targetEl : 0),
       (long)(targetEl > 0 ? targetEl : 0));
  logf("Scan speeds AZ=%lu/s EL=%lu/s; accelerations AZ=%lu EL=%lu",
       (unsigned long)AZ_SCAN_SPEED, (unsigned long)EL_SCAN_SPEED,
       (unsigned long)AZ_SCAN_ACCELERATION,
       (unsigned long)EL_SCAN_ACCELERATION);
  // From bottom-right (where b was marked), return at FAST_SPEED
  // to top-left for the first photo; then scan at gentle speed.
  goToFrame(0);
}

void updateScan() {
  if (!scanActive()) return;

  switch (scanState) {
    case ScanState::Moving:
      if (!motorsMoving()) {
        stageStartedAt = millis();
        scanState = ScanState::Settling;
      }
      break;

    case ScanState::Settling: {
      const uint32_t pauseMs = dryRunMode ? DRY_RUN_PAUSE_MS : SETTLE_MS;
      if ((uint32_t)(millis() - stageStartedAt) >= pauseMs) {
        if (dryRunMode) {
          // One visit per frame; never access BLE or send a shutter key.
          ++photosTaken;
          logf("DRY frame %lu/%lu: AZ=%ld EL=%ld (no shutter)",
               (unsigned long)photosTaken,
               (unsigned long)(columns * rows),
               (long)azMotor->getCurrentPosition(),
               (long)elMotor->getCurrentPosition());

          // In dry mode, skip AFTER_SHOT_MS as well, so there is
          // only one configurable pause per frame.
          if (photosTaken == columns * rows) {
            scanState = ScanState::Idle;
            logf("DRY RUN COMPLETE: %lu frames visited; no photos taken.",
                 (unsigned long)photosTaken);
            printPosition();
          } else {
            goToFrame(photosTaken);
          }
        } else if (!bleKeyboard.isConnected()) {
          logf("BLE disconnected: waiting (no photo skipped).");
          scanState = ScanState::WaitingForBluetooth;
        } else if (triggerShutter()) {
          ++photosTaken;
          logf("SHUTTER sent %lu/%lu", (unsigned long)photosTaken,
               (unsigned long)(columns * rows));
          stageStartedAt = millis();
          scanState = ScanState::AfterShot;
        }
      }
      break;
    }

    case ScanState::WaitingForBluetooth:
      if (bleKeyboard.isConnected()) {
        logf("BLE reconnected; settling again.");
        stageStartedAt = millis();
        scanState = ScanState::Settling;
      }
      break;

    case ScanState::AfterShot:
      if ((uint32_t)(millis() - stageStartedAt) >= AFTER_SHOT_MS) {
        if (photosTaken == columns * rows) {
          scanState = ScanState::Idle;
          logf("SCAN COMPLETE: %lu shutter commands sent.",
               (unsigned long)photosTaken);
          printPosition();
        } else {
          goToFrame(photosTaken);
        }
      }
      break;

    case ScanState::Idle:
      break;
  }
}

// ============================================================
// MANUAL CONTROLS
// ============================================================
void manualMove(FastAccelStepper *motor, int32_t steps,
                uint32_t speed, const char *axis) {
  if (scanActive()) {
    logf("Scan active: press s to cancel first.");
    return;
  }
  motor->enableOutputs();
  delay(2);
  motor->setSpeedInHz(speed);
  motor->setAcceleration(ACCELERATION);
  auto result = motor->move(steps);
  if (result == MOVE_OK) {
    manualMovementPending = true;
    logf("%s %+ld steps at %lu/s", axis, (long)steps,
         (unsigned long)speed);
  } else {
    logf("Manual move rejected: %d", (int)result);
  }
}

void printHelp() {
  logf("l/L r/R u/U d/D = small/large move");
  logf("c = set top-left HOME to (0,0); b = mark bottom-right");
  logf("y = toggle DRY RUN (no BLE, shorter pause); g = start scan");
  logf("f = manual BLE photo (even in dry mode)");
  logf("h = go HOME; s = stop/cancel; p = position; ? = help");
}

void handleCommand(char cmd) {
  switch (cmd) {
    case 'l': manualMove(azMotor, -AZ_SMALL_MOVEMENT, SLOW_SPEED, "AZ"); break;
    case 'L': manualMove(azMotor, -AZ_LARGE_MOVEMENT, FAST_SPEED, "AZ"); break;
    case 'r': manualMove(azMotor,  AZ_SMALL_MOVEMENT, SLOW_SPEED, "AZ"); break;
    case 'R': manualMove(azMotor,  AZ_LARGE_MOVEMENT, FAST_SPEED, "AZ"); break;
    case 'u': manualMove(elMotor,  EL_SMALL_MOVEMENT, SLOW_SPEED, "EL"); break;
    case 'U': manualMove(elMotor,  EL_LARGE_MOVEMENT, FAST_SPEED, "EL"); break;
    case 'd': manualMove(elMotor, -EL_SMALL_MOVEMENT, SLOW_SPEED, "EL"); break;
    case 'D': manualMove(elMotor, -EL_LARGE_MOVEMENT, FAST_SPEED, "EL"); break;

    case 'c':
      if (scanActive() || motorsMoving()) {
        logf("Cannot redefine home during movement/scan.");
      } else {
        azMotor->setCurrentPosition(0);
        elMotor->setCurrentPosition(0);
        homeDefined = true;
        targetDefined = false;        // previous corner no longer valid
        logf("HOME defined at current position: AZ=0 EL=0");
      }
      break;

    case 'b':
      if (scanActive() || motorsMoving() || !homeDefined) {
        logf("Set home with c, then stop at bottom-right before b.");
      } else {
        targetAz = azMotor->getCurrentPosition();
        targetEl = elMotor->getCurrentPosition();
        targetDefined = (targetAz != 0 || targetEl != 0);
        logf("Bottom-right: AZ=%ld EL=%ld %s", (long)targetAz,
             (long)targetEl, targetDefined ? "(saved)" : "(invalid)");
        if (targetDefined) {
          uint32_t c = pointsForRange(targetAz, FRAME_AZ_PITCH);
          uint32_t r = pointsForRange(targetEl, FRAME_EL_PITCH);
          logf("Planned grid: %lu columns x %lu rows = %llu photos",
               (unsigned long)c, (unsigned long)r,
               (unsigned long long)c * r);
        }
      }
      break;

    case 'y':
      if (scanActive()) {
        logf("Cannot change dry-run mode during an active scan; press s first.");
      } else {
        dryRunMode = !dryRunMode;
        logf("Mode: %s | per-frame pause=%lu ms",
             dryRunMode ? "DRY RUN (no BLE/shutter)" : "PHOTO (BLE shutter)",
             (unsigned long)(dryRunMode ? DRY_RUN_PAUSE_MS : SETTLE_MS));
      }
      break;

    case 'g': startScan(); break;

    case 'h':
      if (!homeDefined || scanActive() || motorsMoving()) {
        logf("Cannot go home yet (undefined home or busy).");
      } else if (commandAbsoluteMove(0, 0, true)) {
        manualMovementPending = true;
        logf("Returning to HOME (0,0) at fast manual speed...");
      }
      break;

    case 'f':
      if (scanActive()) {
        logf("Scan active; no extra shutter during scan.");
      } else if (triggerShutter()) {
        logf("Manual shutter triggered.");
      }
      break;

    case 'p':
      printPosition();
      logf("Mode: %s | BLE: %s | Target: %s | Scan: %s | %s: %lu",
           dryRunMode ? "DRY RUN" : "PHOTO",
           bleKeyboard.isConnected() ? "connected" : "disconnected",
           targetDefined ? "set" : "unset",
           scanActive() ? "active" : "idle",
           dryRunMode ? "Frames visited" : "Shutters sent",
           (unsigned long)photosTaken);
      break;

    case 's':
      cancelScan(true);
      manualMovementPending = true;
      logf("Stopping both motors (decelerating)...");
      break;

    case '?': printHelp(); break;
    case '\r': case '\n': case ' ': break;
    default: logf("Unknown command '%c'. Press ? for help.", cmd); break;
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  logf("=== Panorama Controller: 2 motors + BLE + snake ===");

  digitalWrite(AZ_EN, HIGH);
  digitalWrite(EL_EN, HIGH);
  pinMode(AZ_EN, OUTPUT);
  pinMode(EL_EN, OUTPUT);

  azUart.begin(UART_BAUD, SERIAL_8N1, AZ_RX, AZ_TX);
  elUart.begin(UART_BAUD, SERIAL_8N1, EL_RX, EL_TX);
  delay(100);

  configureDriver(azDriver, AZ_CURRENT_MA, AZ_HOLD_MULT);
  configureDriver(elDriver, EL_CURRENT_MA, EL_HOLD_MULT);
  logf("TMC UART: AZ=0x%02X, EL=0x%02X", azDriver.version(),
       elDriver.version());
  if (azDriver.test_connection() != 0 ||
      elDriver.test_connection() != 0) {
    logf("ERROR: TMC UART failure; motor outputs remain disabled.");
    while (true) delay(1000);
  }

  engine.init();
  azMotor = configureMotor(AZ_STEP, AZ_DIR, AZ_EN, AZ_REVERSED);
  elMotor = configureMotor(EL_STEP, EL_DIR, EL_EN, EL_REVERSED);
  if (!azMotor || !elMotor) {
    logf("ERROR: stepper initialization failure.");
    while (true) delay(1000);
  }

  bleKeyboard.begin();
  logf("BLE: advertising as Panorama Shut2");
  logf("Current: AZ=%u mA, EL=%u mA; 16 microsteps",
       AZ_CURRENT_MA, EL_CURRENT_MA);
  logf("Grid pitch (change in sketch): AZ=%lu, EL=%lu microsteps",
       (unsigned long)FRAME_AZ_PITCH,
       (unsigned long)FRAME_EL_PITCH);
  logf("Manual steps: AZ small=%ld large=%ld; EL small=%ld large=%ld",
       (long)AZ_SMALL_MOVEMENT, (long)AZ_LARGE_MOVEMENT,
       (long)EL_SMALL_MOVEMENT, (long)EL_LARGE_MOVEMENT);
  logf("Home/scan speeds: AZ=%lu/s EL=%lu/s",
       (unsigned long)AZ_SCAN_SPEED, (unsigned long)EL_SCAN_SPEED);
  logf("Mode: PHOTO (default); press y to toggle DRY RUN, pause %lu ms",
       (unsigned long)DRY_RUN_PAUSE_MS);
  printHelp();
}

void loop() {
  while (Serial.available() > 0) {
    handleCommand((char)Serial.read());
  }

  updateScan();

  if (manualMovementPending && !motorsMoving()) {
    manualMovementPending = false;
    logf("Movement finished.");
    printPosition();
  }
}

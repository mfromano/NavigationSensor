/*

  main.cpp

*/
// Include Config-file (moved for code clarity)
#include "main_config.h"
const String fileVersion = __TIMESTAMP__;

// Create Sensor Objects with Specified Slave-Select Pins
ADNS adnsA(CS_PIN_A);
ADNS adnsB(CS_PIN_B);
sensor_pair_t sensor = {adnsA, adnsB};

// Create Timing Objects
const int32_t samplePeriodMicros = 1000000L / (int32_t)NAVSENSOR_FPS;

// Periodic Tasks
Ticker cmdTicker(receiveCommand, 100, 0, MILLIS);
// Ticker transmitTicker(sendData, samplePeriodMicros, 0, MICROS);

// Capture Task (on interrupt)
IntervalTimer captureTimer;

// Initialize sample buffer
CircularBuffer<sensor_sample_t, 10> sensorSampleBuffer;

// Counter and Timestamp Generator
elapsedMicros usSinceStart;
volatile time_t currentSampleTimestamp;
volatile int32_t sampleCountRemaining = 0;

volatile bool isRunning = false;

// =============================================================================
//   SETUP & LOOP
// =============================================================================
void setup() {
  bool success;
  success = initializeCommunication();
  if (success) {
    Serial.println("Communication initialized");
    Serial.println(fileVersion);
  }
  success = initializeSensors();
  if (success) {
    Serial.println("Sensors initialized");
  }
  success = initializeTriggering();
  if (success) {
    Serial.println("Triggers initialized");
  }
  // sampleCountRemaining = 480;
  cmdTicker.start();
  // transmitTicker.start();
}

void loop() {
  if (sampleCountRemaining > 0 || !fastDigitalRead(MANUAL_TRIGGER_PIN)) {
    if (!isRunning) {
      startAcquisition();
    }
  } else {
    if (isRunning) {
      stopAcquisition();
    }
  }
  // transmitTicker.update();
  cmdTicker.update();
  // hbTicker.update();
}

// =============================================================================
//   TASKS: INITIALIZE
// =============================================================================
bool initializeCommunication() {
  // Begin Serial
  Serial.begin(115200);
  while (!Serial) {
    ;  // may only be needed for native USB port
  }
  delay(10);
  return true;
};
bool initializeSensors() {
  // Begin Sensors
  sensor.left.begin();
  delay(30);
  sensor.right.begin();
  delay(30);
  return true;
};
bool initializeClocks() { return true; }
bool initializeTriggering() {
  // Set Sync In Pin Mode
  fastPinMode(TRIGGER_IN_PIN, INPUT);
  fastPinMode(MANUAL_TRIGGER_PIN, INPUT_PULLUP);
  // Set Sync Out Pin Modes
  fastDigitalWrite(TRIGGER_OUT_1_PIN, !TRIGGER_ACTIVE_STATE);
  fastDigitalWrite(TRIGGER_OUT_2_PIN, !TRIGGER_ACTIVE_STATE);
  fastDigitalWrite(TRIGGER_OUT_3_PIN, !TRIGGER_ACTIVE_STATE);
  fastPinMode(TRIGGER_OUT_1_PIN, OUTPUT);
  fastPinMode(TRIGGER_OUT_2_PIN, OUTPUT);
  fastPinMode(TRIGGER_OUT_3_PIN, OUTPUT);
  // Setup Sync/Trigger-Output Timing
  // FrequencyTimer2::setPeriod(1e6 / DISPLACEMENT_SAMPLE_RATE)
  return true;
};

// =============================================================================
// TASKS: IDLE
// =============================================================================
static inline void receiveCommand() {
  // Read Serial to see if request for more frames has been sent
  if (Serial.available()) {
    // delayMicroseconds(1000);
    int32_t moreSamplesCnt = Serial.parseInt();
    sampleCountRemaining += moreSamplesCnt;
  }
}

static inline void startAcquisition() {
  // Print units and Fieldnames (header)
  sendHeader();

  // Trigger start using class methods in ADNS library
  sensor.left.triggerAcquisitionStart();
  sensor.right.triggerAcquisitionStart();

  // Change State
  isRunning = true;

  // Reset Elapsed Time Counter
  usSinceStart = 0;
  currentSampleTimestamp = usSinceStart;

  // Begin IntervalTimer
  captureTimer.begin(captureDisplacement, samplePeriodMicros);

  // Set Trigger-Out-1 to indicate Running
  fastDigitalWrite(TRIGGER_OUT_1_PIN, TRIGGER_ACTIVE_STATE);
  fastDigitalWrite(TRIGGER_OUT_2_PIN, TRIGGER_ACTIVE_STATE);
  fastDigitalWrite(TRIGGER_OUT_3_PIN, TRIGGER_ACTIVE_STATE);
}

static inline void stopAcquisition() {
  // Trigger start using class methods in ADNS library
  sensor.left.triggerAcquisitionStop();
  sensor.right.triggerAcquisitionStop();

  // Begin IntervalTimer
  captureTimer.end();

  // Change state
  isRunning = false;

  // UnSet Trigger-Out to indicate Running
  fastDigitalWrite(TRIGGER_OUT_1_PIN, !TRIGGER_ACTIVE_STATE);
  fastDigitalWrite(TRIGGER_OUT_2_PIN, !TRIGGER_ACTIVE_STATE);
  fastDigitalWrite(TRIGGER_OUT_3_PIN, !TRIGGER_ACTIVE_STATE);
}

// =============================================================================
// TASKS: TRIGGERED_ACQUISITION
// =============================================================================
void captureDisplacement() {
  // Unset Trigger Outputs
  fastDigitalWrite(TRIGGER_OUT_2_PIN, !TRIGGER_ACTIVE_STATE);
  fastDigitalWrite(TRIGGER_OUT_3_PIN, !TRIGGER_ACTIVE_STATE);

  // Initialize container for combined & stamped sample
  sensor_sample_t currentSample;
  currentSample.timestamp = currentSampleTimestamp;

  // Trigger capture from each sensor
  sensor.left.triggerSampleCapture();
  sensor.right.triggerSampleCapture();

  currentSample.left = {'L', sensor.left.readDisplacement(units)};
  currentSample.right = {'R', sensor.right.readDisplacement(units)};
  // todo: no units

  // Push current sample into buffer for transmission
  sensorSampleBuffer.push(currentSample);

  // Decrement sample counter and time counter
  if (sampleCountRemaining >= 0) {
    sampleCountRemaining--;
  }
  currentSampleTimestamp = usSinceStart;

  // Reset Trigger Outputs
  fastDigitalWrite(TRIGGER_OUT_2_PIN, TRIGGER_ACTIVE_STATE);
  static int triggerOut3Cnt = TRIGGER_OUT_3_DIVISOR;
  if (--triggerOut3Cnt <= 0) {
    fastDigitalWrite(TRIGGER_OUT_3_PIN, TRIGGER_ACTIVE_STATE);
    triggerOut3Cnt = TRIGGER_OUT_3_DIVISOR;
  }

  // Send Data
  sendData();
}

// =============================================================================
// TASKS: DATA_TRANSFER
// =============================================================================

void sendHeader() {
  const String dunit = getAbbreviation(units.distance);
  const String tunit = getAbbreviation(units.time);
  Serial.flush();
  Serial.print(String(
      "timestamp [us]" + delimiter + flatFieldNames[0] + " [" + dunit + "]" +
      delimiter + flatFieldNames[1] + " [" + dunit + "]" + delimiter +
      flatFieldNames[2] + " [" + tunit + "]" + delimiter + flatFieldNames[3] +
      " [" + dunit + "]" + delimiter + flatFieldNames[4] + " [" + dunit + "]" +
      delimiter + flatFieldNames[5] + " [" + tunit + "]" + "\n"));
}

void sendData() {
  // Print Velocity
  while (!(sensorSampleBuffer.isEmpty())) {
    sensor_sample_t sample = sensorSampleBuffer.shift();

    // Convert to String class
    const String timestamp = String(sample.timestamp);
    const String dxL = String(sample.left.p.dx, decimalPlaces);
    const String dyL = String(sample.left.p.dy, decimalPlaces);
    const String dtL = String(sample.left.p.dt, decimalPlaces);
    const String dxR = String(sample.right.p.dx, decimalPlaces);
    const String dyR = String(sample.right.p.dy, decimalPlaces);
    const String dtR = String(sample.right.p.dt, decimalPlaces);
    const String endline = String("\n");

    // Serial.availableForWrite
    // Print ASCII Strings
    Serial.print(timestamp + delimiter + dxL + delimiter + dyL + delimiter +
                 dtL + delimiter + dxR + delimiter + dyR + delimiter + dtR +
                 endline);
  }
}

/*

  main_config.h

*/

// Arduino Includes
#include <Arduino.h>
#include <CircularBuffer.h>
#include <DigitalIO.h>
#include <SPI.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

// Standard Template Library Includes (uCLibC++ port or ETL)
// #include <"ArduinoSTL.h">
// #include <vector>

// Include ADNS Library for ADNS-9800 Sensor
#include "ADNS9800\ADNS.h"

// Pin Settings
const int CS_PIN_A = 20;
const int CS_PIN_B = 21;
const int TRIGGER_IN_PIN = 0;
const int TRIGGER_OUT_1_PIN = 4;
const int TRIGGER_OUT_2_PIN = 5;
const int TRIGGER_OUT_3_PIN = 6;
const int MANUAL_TRIGGER_PIN = 7;
const bool TRIGGER_ACTIVE_STATE = HIGH;

// Pre-Compute semi-synchronous sample rates for navigation sensors and camera
const int NAVSENSOR_FPS = 120;
const int TRIGGER_OUT_3_DIVISOR = 12;

// =============================================================================
// Timing & Trigger-Output Settings and Implementation
// =============================================================================
// // Use zero-jitter & cross-platform Frequency-Timer-2 library for main clock
// #include <FrequencyTimer2.h>

// Use Interval Timer for Triggering
#include <IntervalTimer.h>
#include <Ticker.h>

// Use ElapsedMillis for time-keeping
#include <FPSTimer.h>
#include <elapsedMillis.h>

// // Use AsyncDelay library for simple pulse reset
// #include <AsyncDelay.h>
// Use TeensyDelay library for trigger-out pulse reset
// #include <TeensyDelay.h>

// Embedded Template Library Timer
// #include <timer.h>

// =============================================================================
// Enumeration and Type Definitions
// =============================================================================
// Data-descriptor type (string or char, variable or fixed-width)
typedef String sensor_name_t;
typedef String field_name_t;

// Message Frame Format
typedef struct {
  uint32_t length;
  enum FrameType : uint8_t { DATA, HEADER, SETTINGS };
  FrameType type;
  uint8_t flags;
  int8_t id;
} message_frame_t;  // todo

// Define data structure for a sample from a single sensor
typedef struct {
  char id = 'L';
  displacement_t p;  // todo: use generic point_t or vec2
} labeled_sample_t;

// Define Left-Right Sensor Pair Structure
typedef struct {
  ADNS &left;
  ADNS &right;
} sensor_pair_t;

typedef struct {
  time_t timestamp;
  labeled_sample_t left;
  labeled_sample_t right;
} sensor_sample_t;

// Delimiter & Precision for Conversion to String
const unit_specification_t units = {Unit::Distance::MICROMETER,
                                    Unit::Time::MICROSECOND};  // todo: remove
constexpr char delimiter = ',';
constexpr unsigned char decimalPlaces = 3;

// Sensor and Field Names
const sensor_name_t sensorNames[] = {"left", "right"};
const field_name_t fieldNames[] = {"dx", "dy", "dt"};
const String flatFieldNames[] = {
    sensorNames[0] + '_' + fieldNames[0], sensorNames[0] + '_' + fieldNames[1],
    sensorNames[0] + '_' + fieldNames[2], sensorNames[1] + '_' + fieldNames[0],
    sensorNames[1] + '_' + fieldNames[1], sensorNames[1] + '_' + fieldNames[2]};

// =============================================================================
// Task Declarations
// =============================================================================

// Task: INITIALIZE
bool initializeSensors();
bool initializeClocks();
bool initializeTriggering();
bool initializeCommunication();

// Task: IDLE
static inline void receiveCommand();
static inline void startAcquisition();
static inline void stopAcquisition();

// Task: TRIGGERED_ACQUISITION
// (capture/acquire/read-loop)
static void captureDisplacement();

// Task: DATA_TRANSFER
static void sendHeader();
static void sendData();

// todo: influxdb format:
//      --> {key, [field], timestamp}
//      or  {<measurement>,[tags], [field], timestamp}
//                field ->  {field-name, field-value}
//                tag ->    {tag-name, tag-value}

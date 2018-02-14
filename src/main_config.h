/*

  main_config.h

*/

// Arduino Includes
#include <Arduino.h>
#include <CircularBuffer.h>
#include <DigitalIO.h>
#include <SPI.h>

// Standard Template Library Includes (uCLibC++ port or ETL)
// #include <"ArduinoSTL.h">
#include <vector>

// Include ADNS Library for ADNS-9800 Sensor
#include "ADNS9800\ADNS.h"

// Pin Settings
#define CS_PIN_A 4
#define CS_PIN_B 5
#define SYNC_OUT_PIN 3
#define SYNC_EVERY_N_PIN 6
#define SYNC_PULSE_WIDTH_MICROS 500
#define SYNC_PULSE_STATE HIGH

// Timing Settings (shooting for nav-sensor sample-rate 480 fps minimum)
#define MIN_SAMPLE_RATE 40
#define SAMPLES_PER_CAMERA_FRAME 12

// Pre-Compute semi-synchronous sample rates for navigation sensors and camera
#define CAMERA_FPS MIN_SAMPLE_RATE
constexpr int32_t NAVSENSOR_FPS() {
  return (MIN_SAMPLE_RATE * SAMPLES_PER_CAMERA_FRAME);
}

// Heart-Beat Settings
#define HEARTBEAT_PERIOD_MILLIS 1000
#define HEARTBEAT_OUTPUT '\0'

// Buffer Size (comment out CHAR_BUFFER_SIZE_FIXED to try variable size mode)
#define CHAR_BUFFER_NUM_BYTES 44

// Delimiters and Terminators
#define FIXED_SIZE_ID_DELIM ':'
#define FIXED_SIZE_DATA_DELIM ','
#define FIXED_SIZE_MSG_TERMINATOR '\t'

// =============================================================================
// Timing & Trigger-Output Settings and Implementation
// =============================================================================
// // Use zero-jitter & cross-platform Frequency-Timer-2 library for main clock
#include <FrequencyTimer2.h>

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

// Define Left-Right Sensor Pair Structure
typedef struct {
  ADNS &left;
  ADNS &right;
} sensor_pair_t;

// Message Frame Format
typedef struct {
  uint32_t length;
  enum FrameType : uint8_t { DATA, HEADER, SETTINGS };
  FrameType type;
  uint8_t flags;
  int8_t id;

} message_frame_t;

// Define data structure for a sample from a single sensor
typedef struct {
  char id = 'L';
  displacement_t p;  // todo: use generic point_t or vec2
} labeled_sample_t;
// todo: influxdb format:
//      --> {key, [field], timestamp}
//      or  {<measurement>,[tags], [field], timestamp}
//                field ->  {field-name, field-value}
//                tag ->    {tag-name, tag-value}

enum ControllerState { SETUP, WAIT, RUN } controllerState;
enum SampleState { NONE, CAPTURE, ACQUIRE, TRANSMIT } sampleState;
enum class TransmitFormat { FIXED, DELIMITED, JSON, BINARY };
// todo binary stream

// Declare Test Functions
static inline void checkState();
static inline void sendHeartBeat();
static inline void startAcquisition() static inline void sendAnyUpdate();
static inline void checkCmd();
static inline void captureDisplacement();
static inline void sendHeader();
void transmitDisplacementBinary(const labeled_sample_t);
void transmitDisplacementDelimitedString(const labeled_sample_t);
void transmitDisplacementFixedSize(const labeled_sample_t);

// Delimiter & Precision for Conversion to String
const TransmitFormat format = TransmitFormat::FIXED;
const unit_specification_t units = {Unit::Distance::MICROMETER,
                                    Unit::Time::MICROSECOND};
constexpr char delimiter = '\t';
constexpr unsigned char decimalPlaces = 3;
constexpr bool waitBeforeStart = true;

// Sensor and Field Names
const sensor_name_t sensorNames[] = {"left", "right"};
const field_name_t fieldNames[] = {"dx", "dy", "dt"};
const String flatFieldNames[] = {
    sensorNames[0] + '_' + fieldNames[0], sensorNames[0] + '_' + fieldNames[1],
    sensorNames[0] + '_' + fieldNames[2], sensorNames[1] + '_' + fieldNames[0],
    sensorNames[1] + '_' + fieldNames[1], sensorNames[1] + '_' + fieldNames[2]};

// typedef struct {
//   int32_t id;
//   sampleType
// } streamSource;
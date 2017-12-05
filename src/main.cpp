/*

  main.cpp

*/
#include <Arduino.h>

// Arduino Includes
#include <SPI.h>
#include <elapsedMillis.h>
#include <CircularBuffer.h>
// #include <IntervalTimer.h>
// IntervalTimer timer;
// timer.priority(0);
// timer.begin(fcn,us);
//
// #include <usb_audio.h>
// #include <AudioStream.h>

// Include ADNS Library for ADNS-9800 Sensor
#include "ADNS9800\ADNS.h"

// Pin Settings
#define CHIPSELECT_PIN 4

// Timing Settings (shooting for 480 fps minimum)
#define CAMERA_FPS 40
#define SAMPLES_PER_CAMERA_FRAME 12
#define CHAR_BUFFER_NUM_BYTES 44
#define CHAR_BUFFER_SIZE_FIXED

// Delimiters and Terminators
#define ID_DATA_DELIM ':'
#define DATA_DELIM ','
#define MSG_TERMINATOR '\r'

// Create a Sensor Object with Specified Slave-Select Pin
ADNS sensor(CHIPSELECT_PIN);
const unit_specification_t units = {Unit::Distance::MICROMETER, Unit::Time::MILLISECOND};

const uint32_t usLoop = 1e6 / (CAMERA_FPS * SAMPLES_PER_CAMERA_FRAME);
elapsedMicros usCnt;

CircularBuffer<displacement_t, 12> buf;

// Declare Test Functions
void transmitDisplacementString(const displacement_t p, String id = "L");
void transmitDisplacementChar(const displacement_t p, const char id = 'L');
void captureDisplacement();

// =============================================================================
//   INITIALIZATION
// =============================================================================
void setup()
{
    // Begin Serial
    Serial.begin(115200);
    while (!Serial)
    {
        ; // may only be needed for native USB port
    }
    delay(10);

    // Begin Sensor
    sensor.begin();
    delay(30);

    // Start Acquisition
    sensor.triggerAcquisitionStart();
    usCnt = 0;
};

// =============================================================================
//   LOOP
// =============================================================================
void loop()
{
    while (usCnt < usLoop)
    {
        // Print Velocity
        while (!buf.isEmpty())
        {
            displacement_t p = buf.shift();
#ifdef CHAR_BUFFER_SIZE_FIXED
            transmitDisplacementChar(p);
#else
            transmitDisplacementString(p);
#endif
        }
    }
    captureDisplacement();
    usCnt -= usLoop; // usCnt = totalLag?
}

//  Variable-Size Conversion to ASCII Strings
void transmitDisplacementString(const displacement_t p, String id)
{
    // Precision
    const unsigned char decimalPlaces = 3;

    // Convert to String class
    const String dx = String(p.dx, decimalPlaces);
    const String dy = String(p.dy, decimalPlaces);
    const String dt = String(p.dt, decimalPlaces);

    // Print ASCII Strings
    Serial.print(id + ID_DATA_DELIM + dx + DATA_DELIM + dy + DATA_DELIM + dt + MSG_TERMINATOR);
}

// Fixed-Size Buffer Conversion to ASCII char array
void transmitDisplacementChar(const displacement_t p, const char id)
{
    // Precision and Max-Width of String Representation of Floats
    static const unsigned char decimalPlaces = 3;
    static const signed char width = ((CHAR_BUFFER_NUM_BYTES - 2) / 3) - 2;
    static const size_t increment = width + 2;

    // Initialize Char-array and Char-Pointer Representation of Buffer
    char cbufArray[CHAR_BUFFER_NUM_BYTES];
    char *cbuf = (char *)cbufArray;

    // Initialize with ASCII Space (32)
    memset(cbuf, (int)(' '), CHAR_BUFFER_NUM_BYTES);

    // Set first Char with ID
    cbufArray[0] = id;
    cbufArray[1] = ID_DATA_DELIM;

    // Jump by Increment and Fill with Limited Width Float->ASCII
    size_t offset = 2;
    dtostrf(p.dx, width, decimalPlaces, cbuf + offset);
    cbufArray[offset + width] = DATA_DELIM;
    offset += increment;
    dtostrf(p.dy, width, decimalPlaces, cbuf + offset);
    cbufArray[offset + width] = DATA_DELIM;
    offset += increment;
    dtostrf(p.dt, width, decimalPlaces, cbuf + offset);

    // Print Buffered Array to Serial
    // cbufArray[CHAR_BUFFER_NUM_BYTES - 1] = '\0';
    // cbufArray[CHAR_BUFFER_NUM_BYTES] = MSG_TERMINATOR;
    cbufArray[offset + width] = MSG_TERMINATOR;
    cbufArray[offset + width + 1] = '\0';
    Serial.write(cbufArray, CHAR_BUFFER_NUM_BYTES - 1);
}

void captureDisplacement()
{
    sensor.triggerSampleCapture();
    buf.push(sensor.readDisplacement(units));
}
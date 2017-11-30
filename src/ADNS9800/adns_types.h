/*
  adns_types.h
  -> Defines data structures and types used for interaction with ADNS9800 chip
*/

#ifndef ADNS_TYPES_h
#define ADNS_TYPES_h

#include <Arduino.h>
#include <stdint.h>
#include "adns_config.h"
#include "NavSensorLib\navsensor.h"

// =============================================================================
// Structured Data-Storage Typedefs with Internal (ADNS-Specific) Units
// =============================================================================

// Time
typedef uint32_t adns_time_t;
typedef uint32_t adns_duration_t;

// Position <x,y> in 'Counts' and Elapsed-Time in Microseconds
typedef struct
{
    int32_t x;     // counts
    int32_t y;     // counts
    adns_time_t t; // microseconds
} adns_position_t; //todo change adns_time_t to {sec,nsec}

// Raw-Readout Array
const size_t adns_readout_max_size = 14; // size in bytes
typedef uint8_t adns_readout_buffer_t[adns_readout_max_size];
typedef union {
    adns_readout_buffer_t data;
    struct
    {
        uint8_t motion;
        uint8_t observation;
        uint8_t dxL;
        uint8_t dxH;
        uint8_t dyL;
        uint8_t dyH;
        uint8_t surfaceQuality;
        uint8_t pixelSum;
        uint8_t maxPixel;
        uint8_t minPixel;
        uint8_t shutterPeriodL;
        uint8_t shutterPeriodH;
        uint8_t framePeriodL;
        uint8_t framePeriodH;
    };
} adns_readout_t;

// Structure with Raw Data and Time of Start
typedef struct
{
    adns_time_t startTime;  // microseconds
    adns_readout_t readout; // byte-array
} adns_capture_t;

// Displacement <dx,dy> in 'Counts' and <dy> in Microseconds
typedef struct
{
    int16_t dx;         // counts
    int16_t dy;         // counts
    adns_duration_t dt; // microseconds
} adns_displacement_t;

typedef struct
{
    adns_time_t timestamp;
    adns_displacement_t displacement;
} adns_sample_t;
//todo add 'seq' or 'index' or 'n' or 'count'
typedef struct
{
    int32_t numFeatures; // actual num-features = numFeatures * 4
    int32_t sumH;        // mean = sumH * 512/900  or sumH/1.76
    int32_t max;
    int32_t min;
} pixel_statistics_t; //todo

#endif

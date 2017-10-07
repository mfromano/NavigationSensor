/*
  adns_types.h
  -> Defines data structures and types used for interaction with ADNS9800 chip
*/

#ifndef ADNS_TYPES_h
#define ADNS_TYPES_h

#include <Arduino.h>
#include <stdint.h>
#include "adns_config.h"

//==================================================================================
// Structured Data Storage Typedefs
//==================================================================================

typedef union {
    uint8_t data[ADNS_BURST_READ_MAX_BYTES];
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
} readout_data_raw_t;

// Raw Sample with Timing Information
typedef struct
{
    uint32_t startTime; //TODO seconds and fraction of seconds
    uint32_t duration;  // microseconds
    readout_data_raw_t readout;
} adns_sample_t;

// Position in 'Counts' (real-value dependent on sensor resolution)
typedef struct
{
    int32_t x;
    int32_t y;
} position_t;

// Displacement <dx,dy> in 'counts' and <dy> in microseconds
typedef struct
{
    int16_t dx;
    int16_t dy;
    uint32_t dt;
} displacement_t;

typedef struct
{
    uint8_t numFeatures; // actual num-features = numFeatures * 4
    uint8_t sumH;        // mean = sumH * 512/900  or sumH/1.76
    uint8_t max;
    uint8_t min;
} pixel_statistics_t;

typedef struct
{
    byte b1 : 1;
    byte b2 : 1;
    byte b3 : 1;
    byte b4 : 1;
    byte b5 : 1;
    byte b6 : 1;
    byte b7 : 1;
    byte b8 : 1;
} EightBitfield;

#endif
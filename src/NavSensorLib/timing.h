/*
  navsensor.h
  -> Common include for NavigationSensor-type devices
*/

#ifndef TIMING_h
#define TIMING_h

//todo Replace delayMicroseconds for SPI comm with non-blocking TimeAlarms
#include <elapsedMillis.h>
#include <glcd_delay.h>
#include <Time.h>
//todo #include <TimeAlarms.h>
// #include <Chrono.h>
//todo #include <FreqMeasure.h>
//    Frequency-Input-Pin:
//          Uno:        8
//          Mega:       49
//          Teensy3.x:  3
//todo IntervalTimer timer; timer.begin(fcn,us); // precision fcn call
//todo #include <TeensyDelay.h> // alternative

inline uint32_t getMicrosElapse(uint32_t t1, uint32_t t2);
uint32_t getMicrosElapse(uint32_t t1, uint32_t t2)
{
    uint32_t dt = t1 > t2 ? 1 + t1 + ~t2 : t2 - t1;
    return dt;
}

// =============================================================================
// High-Precision Time & Timestamping Typedefs
// =============================================================================

// OSC timestamp data-type for High-Resolution Timestamp
typedef struct
{
    uint32_t seconds;
    uint32_t fractionOfSeconds;
} osc_time_t;

typedef uint32_t time_t; // todo time_t

#endif

// preprocessor macros
// __DATE__
// __TIME__
// __FILE__
// __FUNCTION__
// __LINE__
//
// gcc builtin functions
// (equivalents to preprocessor macros) e.g.  __builtin_FILE
// bcmp, alloca, bzero, dremf, dreml, drem, gettext, index, mempcpy
// rindex, scalbf, signbit, stpcpy, toascii, acosh, ccos, cpow, csinf,
//  fdim, fmaf, fmaxf, fmin, isblank, nearbyintf, remainderf, remainderl, 
//  truncf, nextafter, 
// abort, acos, asin, exit, fmod, fscanf, isdigit, tolower, malloc, memchr, 
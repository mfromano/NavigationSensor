/*
  navsensor.h
  -> Common include for NavigationSensor-type devices
*/

#ifndef TIMING_h
#define TIMING_h

//todo Nanoseconds Delay
#include <elapsedMillis.h>
#include <glcd_delay.h>

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

// // extern volatile unsigned long timer0_overflow_count;
// // extern volatile unsigned long timer0_millis_count;
// // extern unsigned char timer0_fract;
// // typedef struct
// // {
// //     const volatile unsigned long &overflow = timer0_overflow_count;
// //     const volatile unsigned long &ms = timer0_millis_count;
// //     const unsigned char &fract = timer0_fract;
// // };

#endif
/*
  navsensor.h
  -> Common include for NavigationSensor-type devices
*/

#ifndef NAVSENSOR_h
#define NAVSENSOR_h

#include <Arduino.h>
#include <stdint.h>

// =============================================================================
// Unit Enumerations
// =============================================================================
namespace Unit
{
enum class Distance
{
    NANOMETER,
    MICROMETER,
    MILLIMETER,
    CENTIMETER,
    METER,
    KILOMETER,
    THOU,
    INCH,
    FOOT,
    YARD,
    MILE
};
// // String getAbbreviation(Distance unit)
// // {
// //     switch (unit)
// //     {
// //     case Distance::NANOMETER:
// //         return "nm";
// //     case Distance::MICROMETER:
// //         return "um";
// //     case Distance::MILLIMETER:
// //         return "mm";
// //     case Distance::CENTIMETER:
// //         return "cm";
// //     case Distance::METER:
// //         return "m";
// //     case Distance::KILOMETER:
// //         return "km";
// //     case Distance::THOU:
// //         return "th";
// //     case Distance::INCH:
// //         return "in";
// //     case Distance::FOOT:
// //         return "ft";
// //     case Distance::YARD:
// //         return "yd";
// //     case Distance::MILE:
// //         return "mi";
// //     default:
// //         return "";
// //     }
// // }

enum class Time
{
    NANOSECOND,
    MICROSECOND,
    MILLISECOND,
    SECOND,
    MINUTE,
    HOUR
};
// // String getAbbreviation(Time unit)
// // {
// //     switch (unit)
// //     {
// //     case Time::NANOSECOND:
// //         return "ns";
// //     case Time::MICROSECOND:
// //         return "us";
// //     case Time::MILLISECOND:
// //         return "ms";
// //     case Time::SECOND:
// //         return "sec";
// //     case Time::MINUTE:
// //         return "min";
// //     case Time::HOUR:
// //         return "hr";
// //     default:
// //         return "";
// //     }
// // }
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

// =============================================================================
// Data-Sample Template Structures
// =============================================================================
template <class T>
struct vec2_cartesian
{
    T x;
    T y;
};

template <class T>
struct vec2_polar
{
    T r;
    float w;
};

template <class T>
struct data_sample
{
    T data;
    time_t time;
};

// =============================================================================
// Structured Data Sample types with values in SI-Units
// =============================================================================
typedef struct
{
    float x;
    float y;
    time_t t; //todo: 64-bit {sec,nsec}
} position_t;

typedef struct
{
    float dx;
    float dy;
    time_t dt; //todo change to duration_t
} displacement_t;

typedef vec2_cartesian<float> velocity_cartesian_t;
typedef vec2_polar<float> velocity_polar_t;
typedef velocity_cartesian_t velocity_t;
typedef data_sample<vec2_cartesian<float>> displacement_sample_t;
typedef data_sample<vec2_cartesian<float>> velocity_sample_t;

typedef struct
{
    time_t timestamp;
    position_t position;
    displacement_t displacement;
} sample_t;

#endif

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
#define DEFAULT_DISTANCE_UNIT Unit::Distance::MILLIMETER
#define DEFAULT_TIME_UNIT Unit::Time::MILLISECOND

// // typedef unit_specification_t;
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
inline const float perInch(const Distance unit)
{
    switch (unit)
    {
    case Distance::NANOMETER:
        return 25400000.000;
    case Distance::MICROMETER:
        return 25400.000000;
    case Distance::MILLIMETER:
        return 25.400000000;
    case Distance::CENTIMETER:
        return 2.5400000000;
    case Distance::METER:
        return .025400000000;
    case Distance::KILOMETER:
        return .000025400000000;
    case Distance::THOU:
        return 1000.0;
    case Distance::INCH:
        return 1.000;
    case Distance::FOOT:
        return 0.0833333;
    case Distance::YARD:
        return 0.0277778;
    case Distance::MILE:
        return 1.5783E-5;
    default:
        return 1.0;
    }
};
// todo unitPerCentimeter() or unitPerMillimeter
enum class Time
{
    NANOSECOND,
    MICROSECOND,
    MILLISECOND,
    SECOND,
    MINUTE,
    HOUR
};
inline const float perSecond(const Time unit) //todo remove const output?
{
    switch (unit)
    {
    case Time::NANOSECOND:
        return 1000000000.000;
    case Time::MICROSECOND:
        return 1000000.000000;
    case Time::MILLISECOND:
        return 1000.000000000;
    case Time::SECOND:
        return 1.000000000000;
    case Time::MINUTE:
        return 0.0166667;
    case Time::HOUR:
        return 0.00027777833333;
    default:
        return 1000000.0;
    }
};
inline const float perMicrosecond(const Time unit)
{
    switch (unit)
    {
    case Time::NANOSECOND:
        return 1000.000000000;
    case Time::MICROSECOND:
        return 1.000000000000;
    case Time::MILLISECOND:
        return 0.001000000000000;
    case Time::SECOND:
        return 0.000001000000000000;
    case Time::MINUTE:
        return 0.00000001666666667;
    case Time::HOUR:
        return 0.00000000027777833333;
    default:
        return 1.0;
    }
}
inline const String getAbbreviation(const Distance unit)
{
    switch (unit)
    {
    case Distance::NANOMETER:
        return "nm";
    case Distance::MICROMETER:
        return "um";
    case Distance::MILLIMETER:
        return "mm";
    case Distance::CENTIMETER:
        return "cm";
    case Distance::METER:
        return "m";
    case Distance::KILOMETER:
        return "km";
    case Distance::THOU:
        return "th";
    case Distance::INCH:
        return "in";
    case Distance::FOOT:
        return "ft";
    case Distance::YARD:
        return "yd";
    case Distance::MILE:
        return "mi";
    default:
        return "";
    }
}
inline const String getAbbreviation(const Time unit)
{
    switch (unit)
    {
    case Time::NANOSECOND:
        return "ns";
    case Time::MICROSECOND:
        return "us";
    case Time::MILLISECOND:
        return "ms";
    case Time::SECOND:
        return "sec";
    case Time::MINUTE:
        return "min";
    case Time::HOUR:
        return "hr";
    default:
        return "";
    }
}
}
typedef struct
{
    Unit::Distance distance = DEFAULT_DISTANCE_UNIT;
    Unit::Time time = DEFAULT_TIME_UNIT;
} unit_specification_t;
const unit_specification_t DEFAULT_UNIT(DEFAULT_DISTANCE_UNIT, DEFAULT_TIME_UNIT);

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
    union {
        float v[3];
        struct
        {
            float x;
            float y;
            float t;
        };
    };
} float3_xyt_t;

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

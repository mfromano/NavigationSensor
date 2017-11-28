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
    Unit::Distance distance;
    Unit::Time time;
} unit_specification_t;
static constexpr unit_specification_t DEFAULT_UNIT = {DEFAULT_DISTANCE_UNIT, DEFAULT_TIME_UNIT};

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

// DURATION
// =============================================================================

// #ifndef _ROS_DURATION_H_
// #define _ROS_DURATION_H_

// #include <math.h>
// #include <stdint.h>

// namespace ros {

//   void normalizeSecNSecSigned(int32_t& sec, int32_t& nsec);

//   class Duration
//   {
//     public:
//       int32_t sec, nsec;

//       Duration() : sec(0), nsec(0) {}
//       Duration(int32_t _sec, int32_t _nsec) : sec(_sec), nsec(_nsec)
//       {
//         normalizeSecNSecSigned(sec, nsec);
//       }

//       double toSec() const { return (double)sec + 1e-9*(double)nsec; };
//       void fromSec(double t) { sec = (uint32_t) floor(t); nsec = (uint32_t) round((t-sec) * 1e9); };

//       Duration& operator+=(const Duration &rhs);
//       Duration& operator-=(const Duration &rhs);
//       Duration& operator*=(double scale);
//   };

// }

// #endif

// #include <math.h>
// #include "ros/duration.h"

// namespace ros
// {
//   void normalizeSecNSecSigned(int32_t &sec, int32_t &nsec)
//   {
//     int32_t nsec_part = nsec;
//     int32_t sec_part = sec;

//     while (nsec_part > 1000000000L)
//     {
//       nsec_part -= 1000000000L;
//       ++sec_part;
//     }
//     while (nsec_part < 0)
//     {
//       nsec_part += 1000000000L;
//       --sec_part;
//     }
//     sec = sec_part;
//     nsec = nsec_part;
//   }

//   Duration& Duration::operator+=(const Duration &rhs)
//   {
//     sec += rhs.sec;
//     nsec += rhs.nsec;
//     normalizeSecNSecSigned(sec, nsec);
//     return *this;
//   }

//   Duration& Duration::operator-=(const Duration &rhs){
//     sec += -rhs.sec;
//     nsec += -rhs.nsec;
//     normalizeSecNSecSigned(sec, nsec);
//     return *this;
//   }

//   Duration& Duration::operator*=(double scale){
//     sec *= scale;
//     nsec *= scale;
//     normalizeSecNSecSigned(sec, nsec);
//     return *this;
//   }

// }

// TIME
// =============================================================================
// #ifndef ROS_TIME_H_
// #define ROS_TIME_H_

// #include <math.h>
// #include <stdint.h>

// #include "ros/duration.h"

// namespace ros
// {
//   void normalizeSecNSec(uint32_t &sec, uint32_t &nsec);

//   class Time
//   {
//     public:
//       uint32_t sec, nsec;

//       Time() : sec(0), nsec(0) {}
//       Time(uint32_t _sec, uint32_t _nsec) : sec(_sec), nsec(_nsec)
//       {
//         normalizeSecNSec(sec, nsec);
//       }

//       double toSec() const { return (double)sec + 1e-9*(double)nsec; };
//       void fromSec(double t) { sec = (uint32_t) floor(t); nsec = (uint32_t) round((t-sec) * 1e9); };

//       uint32_t toNsec() { return (uint32_t)sec*1000000000ull + (uint32_t)nsec; };
//       Time& fromNSec(int32_t t);

//       Time& operator +=(const Duration &rhs);
//       Time& operator -=(const Duration &rhs);

//       static Time now();
//       static void setNow( Time & new_now);
//   };

// }

// #endif

// #include "ros/time.h"

// namespace ros
// {
//   void normalizeSecNSec(uint32_t& sec, uint32_t& nsec){
//     uint32_t nsec_part= nsec % 1000000000UL;
//     uint32_t sec_part = nsec / 1000000000UL;
//     sec += sec_part;
//     nsec = nsec_part;
//   }

//   Time& Time::fromNSec(int32_t t)
//   {
//     sec = t / 1000000000;
//     nsec = t % 1000000000;
//     normalizeSecNSec(sec, nsec);
//     return *this;
//   }

//   Time& Time::operator +=(const Duration &rhs)
//   {
//     sec += rhs.sec;
//     nsec += rhs.nsec;
//     normalizeSecNSec(sec, nsec);
//     return *this;
//   }

//   Time& Time::operator -=(const Duration &rhs){
//     sec += -rhs.sec;
//     nsec += -rhs.nsec;
//     normalizeSecNSec(sec, nsec);
//     return *this;
//   }
// }
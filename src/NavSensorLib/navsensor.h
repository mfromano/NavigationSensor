/*
  navsensor.h
  -> Common include for NavigationSensor-type devices
*/

#ifndef NAVSENSOR_h
#define NAVSENSOR_h

#include <Arduino.h>
#include <stdint.h>
#include "timing.h"

// =============================================================================
// Unit Enumerations
// =============================================================================
#define DEFAULT_DISTANCE_UNIT Unit::Distance::MILLIMETER
#define DEFAULT_TIME_UNIT Unit::Time::MILLISECOND

#define ARRAY_SIZE(a) sizeof(a) / sizeof(a[0])

// todo teensy interrupt pin
// uno,nano,mini: 2, 3
// mega: 2, 3, 21, 20, 19, 18
// teensy:

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
inline float perInch(const Distance unit)
{
    switch (unit)
    {
    case Distance::NANOMETER:
        return 25400000.0;
    case Distance::MICROMETER:
        return 25400.0;
    case Distance::MILLIMETER:
        return 25.4;
    case Distance::CENTIMETER:
        return 2.54;
    case Distance::METER:
        return .0254;
    case Distance::KILOMETER:
        return .0000254;
    case Distance::THOU:
        return 1000.0;
    case Distance::INCH:
        return 1.000;
    case Distance::FOOT:
        return .0833333;
    case Distance::YARD:
        return .0277778;
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
inline float perSecond(const Time unit) //todo remove const output?
{
    switch (unit)
    {
    case Time::NANOSECOND:
        return 1000000000.0;
    case Time::MICROSECOND:
        return 1000000.0;
    case Time::MILLISECOND:
        return 1000.0;
    case Time::SECOND:
        return 1.0;
    case Time::MINUTE:
        return 0.0166667;
    case Time::HOUR:
        return 0.00027777833333;
    default:
        return 1000000.0;
    }
};
inline float perMicrosecond(const Time unit)
{
    switch (unit)
    {
    case Time::NANOSECOND:
        return 1000.0;
    case Time::MICROSECOND:
        return 1.0;
    case Time::MILLISECOND:
        return 0.001;
    case Time::SECOND:
        return 0.000001;
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
    int32_t count;
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

// ADAFRUIT_SENSOR UNIFIED
// bool getEvent(sensors_event_t *event)
// void getSensor(sensor_t *sensor)
// enable
// disable
// void displaySensorDetails(void)
// void configureSensor(void)

// // #if defined(ARDUINO_ARCH_ESP8266) | defined(ARDUINO_ESP8266_ESP01)
// // #elif defined(__AVR__)
// // #elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__) || defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega328_) || defined(__AVR_ATmega128__)
// // #elif defined(ESP8266) || defined(ESP32)
// // #elif defined(ARDUINO_ESP8266_NODEMCU) || defined(ARDUINO_ESP8266_WEMOS_D1MINI)
// // #elif defined(CORE_TEENSY)
// // #elif defined(__arm__) && defined(TEENSYDUINO)
// // #if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MKL26Z64__) || defined(__MK66FX1M0__)
// // // Teensy 3.1 -> __MK20DX256__
// // // Teensy 3.0  3.1 (PIC32's might have some day) 3.1LC 3.2 3.5
// // //todo add teensy support // #include <TeensyDelay.h>
// // #endif
// // #endif
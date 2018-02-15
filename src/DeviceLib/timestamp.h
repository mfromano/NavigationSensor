/*
  timestamp.h
  -> Common include for data timestamp
*/

#ifndef TIMESTAMP_h
#define TIMESTAMP_h

#include <Arduino.h>
#include <Time.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <chrono>
#include <memory>

// struct bintime bt;
typedef timeval timestamp_t;

timestamp_t stamp;

// struct timespec ts;
// struct timespec {
// 	time_t	tv_sec;		/* seconds */
// 	long	tv_nsec;	/* and nanoseconds */
// };
// struct timeval {
// 	time_t		tv_sec;		/* seconds */
// 	suseconds_t	tv_usec;	/* and microseconds */
// };

#endif
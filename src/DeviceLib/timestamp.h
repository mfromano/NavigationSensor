/*
  timestamp.h
  -> Common include for data timestamp
*/

#ifndef TIMESTAMP_h
#define TIMESTAMP_h

#include <Arduino.h>

// uint64_t launchTime = 0;

// uint64_t getCurTime() {
// static uint64_t lastTime = launchTime;
// uint32_t diffTime = micros() - lastTime;
// uint64_t t = micros() + launchTime;
// return t;
// todo
//  Serial.println(t/);
// sprintf(tsBuffer, "%lu.%06lu", (uint32_t)(t / 1000000),
//         (uint32_t)(t % 1000000));
// }

// #include <Time.h>

// #include <inttypes.h>
// #include <stdint.h>
// #include <stdlib.h>
// #include <chrono>
// #include <ctime>
// #include <thread>

// auto start_ts = std::chrono::clock::now();

// typedef struct timespec timestamp_t;
// timestamp_t t;

// namespace ns {
// typedef union {
//   suseconds_t usec;
//   struct {
//     int32_t sec;
//     int32_t nsec;
//   }
// } duration_t;

// typedef union {
//   struct {
//     uint32_t sec;
//     uint32_t nsec;
//   };
//   useconds_t usec;
//   struct bintime bt;
//   struct timeval tv;
//   struct timespec ts;
//   struct itimerspec its;
// } timestamp_t;
// }  // namespace ns

// }  // namespace ns

// std::chrono::time_point<std::chrono::system_clock> start, end;
// start = std::chrono::system_clock::now();
// //
// // Some computations...
// //
// end = std::chrono::system_clock::now();
// std::chrono::duration<double> elapsed_seconds = end - start;
// elapsed_seconds.count();

// struct bintime bt;
// typedef timeval timestamp_t;
// timestamp_t stamp;

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
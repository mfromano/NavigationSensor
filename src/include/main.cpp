/*

  main.cpp

*/

// Arduino Includes
#include <Arduino.h>
#include <SPI.h>
// Include ADNS Library for ADNS-9800 Sensor
#include <ADNS.h>
// Pin Settings
#define CHIPSELECT_PIN 4

// Create a Sensor Object with Specified Slave-Select Pin
ADNS sensor(CHIPSELECT_PIN, 0);
int k = 0;

position_t lastpos = {0, 0};

//==================================================================================
//   INITIALIZATION
//==================================================================================
void setup()
{
  Serial.begin(9600);
  //Serial.begin(115200);
  while (!Serial)
  {
    ; // may only be needed for native USB port
  }
  delay(10);
  Serial.println("about to begin sensor");
  sensor.begin(200, 1000);
  Serial.println("sensor started");
  delay(30);
};

//==================================================================================
//   LOOP
//==================================================================================
void loop()
{
  motion_sample_full_t block;
  delay(100);
  sensor.motionBurstRead(&block);
  Serial.println("[\t" + String(block.dx) + "\t,\t" + String(block.dy) + "\t] ");
}

// void loop()
// {
//   uint16_t resolution, rate, minrate;
//   if (Serial.available())
//   {

//     // resolution = (int16_t)Serial.parseInt();
//     // sensor.setResolution(resolution);
//     minrate = (uint16_t)Serial.parseInt();
//     sensor.setMinSampleRate(minrate);
//     Serial.println("Minimum Sample-Rate: " + String(minrate));
//   }
//   // Update X and Y
//   boolean motionFlag = sensor.update();
//   position_t sd = sensor.read();
//   if (motionFlag)
//   {
//     Serial.println("[\t" + String((int32_t)sd.x - lastpos.x) + "\t,\t" + String((int32_t)sd.y - lastpos.y) + "\t] ");
//     lastpos = sd;
//   }
//   else
//   {
//     if (k == 0)
//     {
//       // Serial.println("{\t" + String((int32_t)sd.x) + "\t,\t" + String((int32_t)sd.y)); // + "\t}" + "Motion-Count: " + String((uint32_t)sensor.motionCount));

//       // read resolution
//       resolution = sensor.getResolution();
//       Serial.println("Resolution: " + String(resolution));
//       rate = sensor.getSampleRate();
//       Serial.println("Current Sample-Rate: " + String(rate));
//       // reset count
//       k = 100;
//     }
//     k -= 1;
//   }
//   delay(10);
// };

//! Alert Comment

//? Question Comment

//// Removed Comment

//TODO Todo Comment

//* Highlight Comment
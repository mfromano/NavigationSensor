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
#define MOUSENUMBER 1

// Create a Sensor Object with Specified Slave-Select Pin
ADNS sensor(CHIPSELECT_PIN);
int k = 0;

//==================================================================================
//   INITIALIZATION
//==================================================================================
void setup()
{
  Serial.begin(9600);
  sensor.initialize();
  delay(30);
};

//==================================================================================
//   LOOP
//==================================================================================
void loop()
{

  // Check for Commands
  if (Serial.available() > 0)
  {
    String command = Serial.readString();
  }

  // Update X and Y
  boolean motionFlag = sensor.updateXY();
  if (motionFlag)
  {
    Serial.println("[" + String((int)sensor.dx) 
      + "," + String((int)sensor.dy) + "] ");
  }
  else
  {
    k += 1;
    if (k > 1000)
    {
      Serial.print("x" + String((int)sensor.x) 
        + "y" + String((int)sensor.y));
      Serial.println("(" + String((int)sensor.motionCount) + ")");
      k = 0;
    }
  }

  delay(10);
};

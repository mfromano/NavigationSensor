

#ifndef DEVICE_h
#define DEVICE_h

#if ARDUINO >= 100
#include "Arduino.h"
#include "Print.h"
#else
#include "WProgram.h"
#endif
#include < stdint.h >

namespace Device
{
class ConnectedDevice
{
public:
  void initialize(void);
  void start(void);
  void stop(void);
  void pause(void);
  void resume(void);
  void reset(void);
  void *read(void);
  uint8_t read(void *, uint8_t);
};

class Sensor
{
public:
  virtual const char *getName();
  static Sensor &setConfig(void *);
  virtual
};

class SPISlaveDevice : public ConnectedDevice
{
public:
  SPISlaveDevice(int chipSelectPin) : _chipSelectPin(chipSelectPin){};

protected:
  int _chipSelectPin;
};

class I2CSlaveDevice : public ConnectedDevice
{
};
};

class DeviceController
{
}

#endif
/*
  adns.h - Library for communicating with ADNS-9800 laser mouse sensor.
  Created by Mark Bucklin, May 21, 2014.
*/

#ifndef ADNS_h
#define ADNS_h

#if ARDUINO >= 100
#include <Arduino.h>
#include <Print.h>
#else
#include <WProgram.h>
#endif
#include < stdint.h >

// Sensor Register Include - ADNS-9800 (hopefully only this file will need to be modified to adapt for similar sensors)
#include "adns_config.h"
#include "adns_types.h"
#include <elapsedMillis.h>

//==================================================================================
// ADNS Class Declaration
//==================================================================================
class ADNS
{
  public:
	// Constructor
	ADNS(int chipSelectPin) : _chipSelectPin(chipSelectPin){};

	// User Interface
	bool begin();
	bool begin(const uint16_t resolutionCPI, const uint16_t minSampleRateHz = ADNS_DEFAULT_SENSOR_MINSAMPLERATE);
	void start(void);
	void stop(void);
	void capture(void);
	bool available(void);
	void acquireDisplacement(int16_t &dx, int16_t &dy);
	void triggerSampleSync(void);				 //todo
	bool triggerSampleReadout(displacement_t &); //todo

	// Operation (triggered reads)
	bool checkMotionSinceLastReadout();
	void readLatchedDisplacementRegisters(displacement_t &);
	void motionBurstRead(readout_data_raw_t &);
	void latchMotionDataForBurstRead(); //todo call checkMotionLastBurst or similar
	bool readLatchedDisplacementDataBurst(byte *buf, size_t numBytes = ADNS_BURST_READ_MAX_BYTES);

	// Sensor Communication (SPI)
	inline void select();
	inline void deselect();
	uint8_t readRegister(const RegisterAddress address);
	void writeRegister(const RegisterAddress address, const uint8_t data);

	// Sensor Settings
	void setResolution(uint16_t cpi);
	uint16_t getResolution();
	void setMaxSamplePeriod(uint16_t us);
	uint16_t getMaxSamplePeriod();
	void setMinSampleRate(uint16_t hz);
	uint16_t getMinSampleRate();
	uint16_t getSamplePeriod();
	uint16_t getSampleRate();
	void setSampleRateRange();					  //todo
	void setMotionSensePinInterruptMode(int pin); //todo mode: accumulate or buffer, rising? falling?

  protected:
	// Configuration
	void initialize();
	void resetCounters();
	void powerUpSensor();
	void shutDownSensor();
	void resetSensor();
	void uploadFirmware();
	void enableLaser();
	void delaySleepTimeout();
	void disableSleepTimeout();
	void setMaxLiftDetectionThreshold();

	//-------------------------------------
	// Private Variables
	//-------------------------------------

	// Status Flags
	bool _sensorConfiguredFlag = false;
	bool _initializedFlag = false;
	volatile bool _selectFlag = false;
	volatile bool _unreadDisplacementLatchedFlag = false;

	// Hardware Configuration (Pins) //todo should be made all const
	const int _chipSelectPin;
	int _motionSensePin;
	size_t _numBytes = ADNS_BURST_READ_MAX_BYTES; //todo setNumBytesBurstRead

	// Running Records and Current Sensor Settings
	String _firmwareRevision;
	uint32_t _beginMillisCount;
	uint32_t _beginMicrosCount;
	uint32_t _motionCount;
	uint32_t _pollCount;
	uint16_t _resolution = ADNS_DEFAULT_SENSOR_RESOLUTION;
	uint16_t _maxSamplePeriod;
	uint16_t _minSampleRate = ADNS_DEFAULT_SENSOR_MINSAMPLERATE;

	// Readout Data
	volatile elapsedMicros _sampleStartTime; //todo
	volatile readout_data_raw_t _currentBurstData;
	displacement_t *_nextCapture;
	volatile displacement_t _capturedDisplacement;
	volatile position_t _lastPosition = {0, 0};
	volatile elapsedMicros _captureTime; //todo
};

//==================================================================================
// Inline Convenience Functions
//==================================================================================
inline int16_t reg2Int16(uint8_t, uint8_t);
inline int16_t convertTwosComplement(int16_t b);
inline uint32_t getMicrosElapse(uint32_t t1, uint32_t t2);

// ADAFRUIT_SENSOR UNIFIED
// bool getEvent(sensors_event_t *event)
// void getSensor(sensor_t *sensor)
// enable
// disable
// void displaySensorDetails(void)
// void configureSensor(void)

#endif

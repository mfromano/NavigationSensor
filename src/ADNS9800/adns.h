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

	// Setup
	bool begin();
	bool begin(const uint16_t resolutionCPI, const uint16_t minSampleRateHz = ADNS_DEFAULT_SENSOR_MINSAMPLERATE);

	// Trigger Start, Capture, & Readout
	void triggerAcquisitionStart();
	void triggerSampleCapture();
	bool triggerSampleReadout();

	// Burst Motion Check & Readout
	void motionBurstRead(adns_raw_readout_t &);
	void latchMotionDataForBurstRead(); //todo call checkMotionLastBurst or similar
	bool readLatchedDisplacementDataBurst(byte *buf, size_t numBytes = ADNS_BURST_READ_MAX_BYTES);

	// Timing & Conversion
	void setAcquisitionStartTime(const adns_time_t timeOrigin = 0x00000000);

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
	bool _configuredFlag = false;
	bool _initializedFlag = false;
	volatile bool _runningFlag = false;
	volatile bool _selectedFlag = false;
	volatile bool _sampleCapturedFlag = false;

	// Hardware Configuration (Pins) //todo should be made all const
	const int _chipSelectPin;
	int _motionSensePin;
	size_t _numBytes = ADNS_BURST_READ_MAX_BYTES; //todo

	// Current Sensor Settings
	String _firmwareRevision;
	uint16_t _resolution = ADNS_DEFAULT_SENSOR_RESOLUTION;
	uint16_t _maxSamplePeriod;
	uint16_t _minSampleRate = ADNS_DEFAULT_SENSOR_MINSAMPLERATE;

	// Readout Data
	adns_time_t _acquisitionStartTime;
	uint32_t _startTimeMicrosOffset;
	volatile adns_sample_t _currentSample;
	// // volatile adns_time_t _sampleCaptureTime;
	// // volatile adns_raw_readout_t _currentReadout;
	volatile position_t _currentPosition = {0, 0};
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

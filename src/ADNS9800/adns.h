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
#include <SPI.h>
#include <stdint.h>

// Sensor Register Include - ADNS-9800
#include "adns_config.h"
#include "adns_types.h"

// Common Include
#include "NavSensorLib\navsensor.h"

// =============================================================================
// ADNS Class Declaration
// =============================================================================
class ADNS
{
  public:
	// Constructor
	ADNS(const int chipSelectPin = 4) : _chipSelectPin(chipSelectPin){};

	// Setup
	bool begin(const uint16_t resolutionCPI = ADNS_DEFAULT_SENSOR_RESOLUTION,
			   const uint16_t minSampleRateHz = ADNS_DEFAULT_SENSOR_MINSAMPLERATE);

	// Trigger Start, Capture, & Readout
	void triggerAcquisitionStart();
	void triggerSampleCapture();
	void triggerPositionUpdate();
	void triggerAcquisitionStop();

	// Convert & Return Captured Data
	displacement_t readDisplacement(const unit_specification_t = DEFAULT_UNIT) const;
	position_t readPosition(const unit_specification_t = DEFAULT_UNIT) const;
	velocity_t readVelocity(const unit_specification_t = DEFAULT_UNIT) const;
	void printLast();


	// Sensor Settings
	void setResolutionCountsPerInch(const uint16_t cpi);
	uint16_t getResolutionCountsPerInch();
	void setMaxSamplePeriodUs(const uint16_t us);
	uint16_t getMaxSamplePeriodUs();
	void setMinSamplePeriodUs(const uint16_t us);
	uint16_t getMinSamplePeriodUs();

	// Sensor Status
	uint16_t getSamplePeriodUs();
	uint16_t getSampleRateHz();
	
	// Sensor Communication (SPI)
	void select();
	void deselect();
	uint8_t readRegister(const RegisterAddress address);
	void writeRegister(const RegisterAddress address, const uint8_t data);

	// Mode Settings
	void setMotionSensePinInterruptMode(const int pin);
	// todo mode: accumulate or buffer, rising? falling?

	// Unit Settings for read___() functions
	void setDistanceUnit(const Unit::Distance unit) { _unitSpec.distance = unit; };
	void setTimeUnit(const Unit::Time unit) { _unitSpec.time = unit; };
	void setUnits(const unit_specification_t unitSpec) { _unitSpec = unitSpec; };
	inline Unit::Distance getDistanceUnit() { return _unitSpec.distance; };
	inline Unit::Time getTimeUnit() { return _unitSpec.time; };
	inline unit_specification_t getUnits() { return _unitSpec; };

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

	// Status Flags
	bool _configuredFlag = false;
	bool _initializedFlag = false;
	volatile bool _runningFlag = false;
	volatile bool _selectedFlag = false;

	// Hardware Configuration (Pins) //todo should be made all const
	const int _chipSelectPin;
	const size_t _readoutSize = adns_readout_max_size;
	int _motionSensePin;

	// Current Sensor Settings
	String _firmwareRevision;
	uint16_t _resolutionCountsPerInch;				// Counts-Per-Inch
	uint16_t _maxSamplePeriodUs;					// Microseconds
	uint16_t _minSamplePeriodUs;					// Microseconds
	adns_time_t _acquisitionStartMicrosCountOffset; // Microseconds

	// Unit Conversions
	float _resolutionInchPerCount;
	unit_specification_t _unitSpec = DEFAULT_UNIT;

	// Readout Data
	adns_capture_t _currentCapture;
	adns_displacement_t _currentDisplacement;
	adns_position_t _currentPosition;
	adns_sample_t _lastSample; //todo use CircularBuffer, push, implement available()

// Static Settings
#ifdef ADNS_READMODE_BURST
	static const uint8_t _motionLatchRegAddr = (uint8_t)RegisterAddress::Motion_Burst;
	static const bool _burstModeFlag = ADNS_READMODE_BURST;
#else
	static const uint8_t _motionLatchRegAddr = (uint8_t)RegisterAddress::Motion;
	static const bool _burstModeFlag = false;
#endif
#ifdef ADNS_POSITIONUPDATE_AUTOMATIC
	static const bool _autoUpdateFlag = ADNS_POSITIONUPDATE_AUTOMATIC;
#else
	static const bool _autoUpdateFlag = false;
#endif
};

// =============================================================================
// Inline Convenience Functions
// =============================================================================
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

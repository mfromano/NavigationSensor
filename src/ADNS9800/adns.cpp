/*
  ADNS.cpp - Library for communicating with ADNS-9800 laser mouse sensor.
  Created by Mark Bucklin, May 21, 2014.
  Adapted from mrjohnk: https://github.com/mrjohnk/ADNS-9800
  Updated 7/1/2017
*/

#include "./adns.h"
#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>

// #include <TeensyDelay.h>

//==================================================================================
//   INTERFACE
//==================================================================================
ADNS::ADNS(int csPin)
{
}

bool ADNS::begin()
{
	initialize();
	return true;
}

bool ADNS::begin(const uint16_t cpi, const uint16_t hz)
{
	_resolution = cpi;
	_minSampleRate = hz;
	initialize();
	// setResolution(cpi);
	// setMinSampleRate(hz);
	return true;
}

void ADNS::triggerAcquisitionStart()
{
	if (!_initializedFlag)
	{
		begin();
	}
	// todo check if _runningFlag -> stop()

	// Flush Sample Registers -> Write 0 to Motion Register
	select();
	SPI.transfer(RegisterAddress::Motion | 0x80);
	SPI.transfer(0x00);
	// Reset Start-Time (Time-Origin)
	setAcquisitionStartTime();
	// Standard Delays
	delayMicroseconds(ADNS_DELAYMICROS_NCSINACTIVE_POST_WRITE);
	deselect();
	delayMicroseconds(ADNS_DELAYMICROS_POST_WRITE);
}

//==================================================================================
// Motion Check & Readout
//==================================================================================
//todo reading
void ADNS::triggerSampleCapture()
{
	// Timestamps
	adns_time_t sampleStartTime;
	adns_duration_t sampleDuration;

	// Fetch/Prepare Sample Readout Buffer
	adns_sample_t &sample = _currentSample;
	adns_raw_readout_t &raw = sample.readout;

	// Copy Sample-Start-Time from Prior Sample
	sampleStartTime = sample.startTime + sample.duration;

	// Run Read-Register Sequence with Timestamp Inserted Directly after Address
	select();
	SPI.transfer(RegisterAddress::Motion);
	uint32_t triggerTime = micros();
	// todo : used elapsedMicros
	sampleDuration = (sampleStartTime + triggerTime) - (_acquisitionStartTime + _startTimeMicrosOffset);

	delayMicroseconds(ADNS_DELAYMICROS_READ_ADDR_DATA);
	uint8_t data = SPI.transfer(0);
	delayMicroseconds(ADNS_DELAYMICROS_NCSINACTIVE_POST_READ);
	deselect();
	delayMicroseconds(ADNS_DELAYMICROS_POST_READ);

	raw.motion = data & 0x80;
	_sampleCapturedFlag = true;
}

void ADNS::triggerSampleReadout()
{
	if (!_sampleCapturedFlag)
	{
		triggerSampleCapture();
	}
	adns_raw_readout_t &raw = _currentReadout;
	raw.dxL = readRegister(RegisterAddress::Delta_X_L);
	raw.dxH = readRegister(RegisterAddress::Delta_X_H);
	raw.dyL = readRegister(RegisterAddress::Delta_Y_L);
	raw.dyH = readRegister(RegisterAddress::Delta_Y_H);
	// dx = (int16_t)(readRegister(RegisterAddress::Delta_X_L)) | ((int16_t)(readRegister(RegisterAddress::Delta_X_H)) << 8);
	// dy = (int16_t)(readRegister(RegisterAddress::Delta_Y_L)) | ((int16_t)(readRegister(RegisterAddress::Delta_Y_H)) << 8);
	_sampleCapturedFlag = false;
}

//==================================================================================
// Burst Motion Check & Readout
//==================================================================================
void ADNS::motionBurstRead(adns_raw_readout_t &mbRaw)
{
	select();
	latchMotionDataForBurstRead();
	//todo: in block timestamp
	byte *buf = &mbRaw.data;
	numBytes = _numBytes;
	bool motionWasRead = readLatchedMotionBurstData(buf, numBytes);

	block.dx = (int16_t)(mbRaw.dxL) | ((int16_t)(mbRaw.dxH) << 8);
	block.dy = (int16_t)(mbRaw.dyL) | ((int16_t)(mbRaw.dyH) << 8);
	//todo: status, numFeatures, pixelstats
	return true;
}

void ADNS::latchMotionDataForBurstRead()
{
	select();
	SPI.transfer(RegisterAddress::Motion_Burst);
	delayMicroseconds(ADNS_DELAYMICROS_READ_ADDR_DATA);
	_sampleCapturedFlag = true;
}

bool ADNS::readLatchedMotionBurstData(byte *buf, size_t numBytes) //todo: change type to adns_raw_readout_t
{
	if (!(_sampleCapturedFlag))
		latchMotionDataForBurstRead();

	// byte *motPtr = buf;
	byte motData;
	bool motionDetected = false;
	motData = SPI.transfer(0x00);
	if (bit_is_set(motData, 7))
	{
		*buf = motData;
		SPI.transfer(++buf, --numBytes);
		motionDetected = true;
	}

	// if (_sampleCapturedFlag == false)
	_sampleCapturedFlag = false;
	return motionDetected;
}

//==================================================================================
// Timing & Conversion
//==================================================================================
void ADNS::setAcquisitionStartTime(const adns_time_t timeOrigin)
{
	_startTimeMicrosOffset = micros();
	adns_time_t offsetOrigin = timeOrigin + (adns_time_t)_startTimeMicrosOffset;
	_acquisitionStartTime = timeOrigin;
	_currentSample.startTime = timeOrigin;
	_currentSample.duration = 0;
}

//==================================================================================
//   SENSOR COMMUNICATION (SPI)
//==================================================================================
void ADNS::select()
{
	if (_selectedFlag == false)
	{
		SPI.beginTransaction(
			SPISettings(
				ADNS_SPI_MAX_SPEED,
				ADNS_SPI_BIT_ORDER,
				ADNS_SPI_DATA_MODE));
		digitalWrite(_chipSelectPin, LOW);
		_selectedFlag = 1;
		delayMicroseconds(1);
	}
}

void ADNS::deselect()
{
	if (_selectedFlag == true)
	{
		// delayMicroseconds(1); // tSCLK-NCS
		digitalWrite(_chipSelectPin, HIGH);
		SPI.endTransaction();
		_selectedFlag = 0;
	}
}

uint8_t ADNS::readRegister(const RegisterAddress address)
{
	// Send 7-bit address with msb=0, clock for 1 uint8_t to receive 1 uint8_t
	select();
	// waitNextTransactionDelay(); //TODO use getMicrosElapse
	SPI.transfer(address & 0x7f);
	delayMicroseconds(ADNS_DELAYMICROS_READ_ADDR_DATA); // tSRAD
	uint8_t data = SPI.transfer(0);
	// setNextTransactionDelay(ADNS_DELAYMICROS_POST_READ); //TODO use getMicrosElapse
	delayMicroseconds(ADNS_DELAYMICROS_NCSINACTIVE_POST_READ); // tSCLK-NCS
	deselect();
	delayMicroseconds(ADNS_DELAYMICROS_POST_READ);
	return data;
}

void ADNS::writeRegister(const RegisterAddress address, const uint8_t data)
{
	// Send 7-bit address with msb=1 followed by data to write
	select();
	// waitNextTransactionDelay(); //TODO use getMicrosElapse
	SPI.transfer(address | 0x80);
	SPI.transfer(data);
	// setNextTransactionDelay(ADNS_DELAYMICROS_POST_WRITE); //TODO use getMicrosElapse
	delayMicroseconds(ADNS_DELAYMICROS_NCSINACTIVE_POST_WRITE);
	deselect();
	delayMicroseconds(ADNS_DELAYMICROS_POST_WRITE);
}

//==================================================================================
//   SENSOR SETTINGS
//==================================================================================
void ADNS::setResolution(uint16_t cpi)
{
	//Input may take any value between 50 and 8200 (will be rounded to nearest 50 cpi)
	cpi = constrain(cpi, ADNS_RESOLUTION_MIN_CPI, ADNS_RESOLUTION_MAX_CPI);
	// uint8_t data = readRegister(RegisterAddress::Configuration_I);
	// Keep current values from reserved bits (0x3f = B00111111) note: data sheet has error, mask is 0xFF
	// uint8_t mask = ADNS_RESOLUTION_REGISTER_MASK;
	// data = (data & ~mask) | (((uint8_t)(cpi / (uint16_t)ADNS_RESOLUTION_MIN_CPI)) & mask);
	uint8_t data = (uint8_t)(cpi / (uint16_t)ADNS_RESOLUTION_MIN_CPI);
	writeRegister(RegisterAddress::Configuration_I, data);
	// Read back resolution from same register to confirm and store in cached property
	getResolution(); // todo: check resolution matches assigned resolution -> report
}

uint16_t ADNS::getResolution()
{
	uint8_t mask = ADNS_RESOLUTION_REGISTER_MASK;
	uint8_t data = readRegister(RegisterAddress::Configuration_I);
	data = data & mask;
	uint16_t cpi = (uint16_t)data * (uint16_t)ADNS_RESOLUTION_MIN_CPI;
	_resolution = cpi;
	return cpi;
}

void ADNS::setMaxSamplePeriod(uint16_t us)
{
	/* Configures sensor hardware -> sets the maximum frame period (minimum frame rate) 
	that can be selected by the automatic frame rate control, OR the actual frame rate if 
	the sensor is placed in manual frame rate control mode
	*/
	uint8_t dataL, dataH;
	uint16_t delayNumCyles = us * ADNS_CHIP_FREQ_MHZ;
	dataL = lowByte(delayNumCyles);
	dataH = highByte(delayNumCyles);
	writeRegister(RegisterAddress::Frame_Period_Max_Bound_Lower, dataL);
	writeRegister(RegisterAddress::Frame_Period_Max_Bound_Upper, dataH);
	getMaxSamplePeriod();
}

uint16_t ADNS::getMaxSamplePeriod()
{
	uint8_t dataL, dataH;
	dataH = readRegister(RegisterAddress::Frame_Period_Max_Bound_Upper);
	dataL = readRegister(RegisterAddress::Frame_Period_Max_Bound_Lower);
	uint16_t us = makeWord(dataH, dataL) / ADNS_CHIP_FREQ_MHZ;
	_maxSamplePeriod = us;
	return us;
}

void ADNS::setMinSampleRate(uint16_t hz)
{
	// Convenience function to call inverse function and pass to setMaxSamplePeriod()
	uint16_t us = (uint16_t)(1000000UL / (uint32_t)hz);
	setMaxSamplePeriod(us);
	getMinSampleRate();
}

uint16_t ADNS::getMinSampleRate()
{
	uint16_t us = getMaxSamplePeriod();
	uint16_t hz = (uint16_t)(1000000UL / (uint32_t)us);
	_minSampleRate = hz;
	return hz;
}

uint16_t ADNS::getSamplePeriod()
{
	uint8_t dataL, dataH;
	dataH = readRegister(RegisterAddress::Frame_Period_Upper);
	dataL = readRegister(RegisterAddress::Frame_Period_Lower);
	uint16_t us = makeWord(dataH, dataL) / ADNS_CHIP_FREQ_MHZ;
	return us;
}

uint16_t ADNS::getSampleRate()
{
	uint16_t us = getSamplePeriod();
	return (uint16_t)(1000000UL / (uint32_t)us);
}
void ADNS::setSampleRateRange()
{
	//todo minSamplePeriod + Shutter
}

void ADNS::setMotionSensePinInterruptMode(int pin)
{
	_motionSensePin = pin;
	// pinMode(pin, INPUTPULLUP);
	// attachInterrupt(digitalPinToInterrupt(pin), (*)(update), LOW)
}

//==================================================================================
//   CONFIGURATION
//==================================================================================
void ADNS::initialize()
{
	if (!_initializedFlag)
	{
		// Set up Serial Peripheral Interface (SPI) & specified chip-select pin
		pinMode(_chipSelectPin, OUTPUT);
		SPI.begin();
		delay(100);
	}
	if (!_configuredFlag)
	{
		// Power-Up Sensor & Set/Confirm Settings on Sensor Device
		powerUpSensor();
		setResolution(_resolution);
		setMinSampleRate(_minSampleRate);
		delaySleepTimeout();
		setMaxLiftDetectionThreshold();
		_configuredFlag = true;
	}
	_currentPosition.x = 0;
	_currentPosition.y = 0;
	_initializedFlag = true;
}

void ADNS::powerUpSensor()
{
	deselect();
	select();
	deselect();
	writeRegister(RegisterAddress::Power_Up_Reset, 0x5a);
	delay(50);
	readRegister(RegisterAddress::Motion);
	readRegister(RegisterAddress::Delta_X_L);
	readRegister(RegisterAddress::Delta_X_H);
	readRegister(RegisterAddress::Delta_Y_L);
	readRegister(RegisterAddress::Delta_Y_H);
	uploadFirmware();
	delay(10);
	enableLaser();
	delay(1);
}

void ADNS::shutDownSensor()
{
	// todo: test
	writeRegister(RegisterAddress::Shutdown, 0xB6);
	_configuredFlag = false;
}

void ADNS::resetSensor()
{
	// todo: test
	deselect();
	select();
	deselect();
	writeRegister(RegisterAddress::Power_Up_Reset, 0x5a);
	delay(50);
	writeRegister(RegisterAddress::Observation, 0x00);
	delayMicroseconds(max(2000, _maxSamplePeriod));
	// uint8_t obs = readRegister(RegisterAddress::Observation);
	// should then check bits 0:5 are set
	readRegister(RegisterAddress::Motion);
	readRegister(RegisterAddress::Delta_X_L);
	readRegister(RegisterAddress::Delta_X_H);
	readRegister(RegisterAddress::Delta_Y_L);
	readRegister(RegisterAddress::Delta_Y_H);
	delay(10);
	enableLaser();
	delay(1);
}

void ADNS::uploadFirmware()
{
	// Firmware supplied by chip manufacturer (Pixart) as a hex file must be uploaded each time the sensor is powered on. Hex-file is defined in array & stored in flash ram in simple header-file ("firmware/adns9800_srom_A6.h") and can easily be interchangeable with updated firmware.
	// Put sensor in firmware upload mode
	writeRegister(RegisterAddress::Configuration_IV, 0x02);
	writeRegister(RegisterAddress::SROM_Enable, 0x1d);
	delay(10);
	writeRegister(RegisterAddress::SROM_Enable, 0x18);
	// SROM Load Burst Sequence
	select();
	SPI.transfer(RegisterAddress::SROM_Load_Burst | 0x80);
	// Write firmware from latest version
	uint8_t c;
	for (uint16_t i = 0; i < firmware_length; i++)
	{
		c = (uint8_t)pgm_read_byte(firmware_data + i);
		delayMicroseconds(15);
		SPI.transfer(c);
		// Store firmware-revision from 2nd uint8_t in hex file array
		if (i == 1)
			_firmwareRevision = String(c, HEX);
	}
	delayMicroseconds(10);
	deselect();
	delayMicroseconds(160);
}

void ADNS::enableLaser()
{
	// Set Force_Disabled bit (bit0) to 0 in LAS ER_CTRL0 register
	uint8_t data = readRegister(RegisterAddress::LASER_CTRL0);
	data = (data & ADNS_LASER_CTRL0_REGISTER_MASK); //TODO NOT RIGHT SCREW IT
	writeRegister(RegisterAddress::LASER_CTRL0, data);
	delay(1);
}

void ADNS::delaySleepTimeout()
{
	writeRegister(RegisterAddress::Run_Downshift, 0xFF);
	writeRegister(RegisterAddress::Rest1_Downshift, 0xFF);
	writeRegister(RegisterAddress::Rest2_Downshift, 0xFF);
}

void ADNS::disableSleepTimeout()
{
	// write 0 to bit 5 and also write zeros in bits 0 and 1;
	uint8_t data = readRegister(RegisterAddress::Configuration_II);
	data &= (~bit(5) & ~0x03);
	writeRegister(RegisterAddress::Configuration_II, data);
}

void ADNS::setMaxLiftDetectionThreshold()
{
	uint8_t data = readRegister(RegisterAddress::Lift_Detection_Thr);
	data = (data & ~ADNS_LIFT_DETECTION_REGISTER_MASK) | (0xFF & ADNS_LIFT_DETECTION_REGISTER_MASK);
	writeRegister(RegisterAddress::Lift_Detection_Thr, data);
}

//==================================================================================
// Inline Convenience Functions
//==================================================================================
int16_t reg2Int16(uint8_t bL, uint8_t bH)
{
	int16_t iw16 = (int16_t)makeWord(bH, bL);
	return convertTwosComplement(iw16);
};
int16_t convertTwosComplement(int16_t b)
{
	//Convert from 16-BIT 2's complement
	if (b & 0x8000)
	{
		b = -1 * ((b ^ 0xffff) + 1);
	}
	return b;
};
uint32_t getMicrosElapse(uint32_t t1, uint32_t t2)
{
	uint32_t dt = t1 > t2 ? 1 + t1 + ~t2 : t2 - t1;
	return dt;
}

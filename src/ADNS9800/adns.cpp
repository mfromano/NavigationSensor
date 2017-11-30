/*
  ADNS.cpp - Library for communicating with ADNS-9800 laser mouse sensor.
  Created by Mark Bucklin, May 21, 2014.
  Adapted from mrjohnk: https://github.com/mrjohnk/ADNS-9800
  Updated 7/1/2017
*/

#include "adns.h"
#include <digitalWriteFast.h>

#define DELAY_450NS asm volatile("nop")
#define DELAY_1uS \
	DELAY_450NS;  \
	DELAY_450NS;

// =============================================================================
//   Setup
// =============================================================================
bool ADNS::begin(const uint16_t cpi, const uint16_t hz)
{
	_resolutionCountsPerInch = cpi;
	_maxSamplePeriodUs = (uint16_t)(1000000UL / (uint32_t)hz);
	initialize();
	return true;
}

// =============================================================================
// Trigger Start, Capture, & Readout
// =============================================================================

void ADNS::triggerAcquisitionStart()
{
	// Check Initialized & Running State
	if (!_initializedFlag)
		initialize();
	if (_runningFlag)
		triggerAcquisitionStop();

	// Flush Sample Registers -> Write 0 to Motion Register
	select();
	SPI.transfer((uint8_t)RegisterAddress::Motion | 0x80);
	SPI.transfer(0x00);

	// Reset Current Position
	_currentPosition.x = 0;
	_currentPosition.y = 0;
	_currentPosition.t = 0;

	// Set Start-Time Microsecond Offset
	adns_time_t us = micros();
	adns_capture_t &capture = _currentCapture;
	capture.startTime = 0;
	capture.readout.motion = 0x00;
	_acquisitionStartMicrosCountOffset = us;

	// Standard Delays
	delayMicroseconds(ADNS_DELAYMICROS_NCSINACTIVE_POST_WRITE);
	deselect();
	delayMicroseconds(ADNS_DELAYMICROS_POST_WRITE);

	// Set Flag to Indicate Running State
	_runningFlag = true;
}

// Read from ADNS9800 Sensor and Update _currentCapture & _currentSample
void ADNS::triggerSampleCapture()
{
	// Trigger Start if Not Running
	if (!_runningFlag)
		triggerAcquisitionStart();

	// Timestamps
	adns_time_t usStart, usFinish;
	adns_duration_t dt;

	// Get Local References to Current Data Structures
	adns_capture_t &capture = _currentCapture;
	adns_readout_t &readout = capture.readout;
	adns_sample_t &sample = _currentSample;
	adns_displacement_t &displacement = sample.displacement;

	// Copy Start-Time from Prior Sample Finish-Time
	capture.startTime = capture.endTime;
	usStart = capture.startTime + _acquisitionStartMicrosCountOffset;

	// Run Read-Register Sequence with Timestamp Inserted Directly after Address
	select();

// Read from Motion or Motion_Burst Address to Latch Data
#if (ADNS_READMODE_BURST)
	static const byte MOTION_LATCH_ADDR = (byte)RegisterAddress::Motion_Burst;
#else
	static const byte MOTION_LATCH_ADDR = (byte)RegisterAddress::Motion_Burst;
#endif
	SPI.transfer(MOTION_LATCH_ADDR & 0x7f);

	// Record Sample Finish-Time
	usFinish = micros(); // todo replace with elapsedMicros or sec,nsec
	dt = getMicrosElapse(usStart, usFinish);
	capture.endTime = capture.startTime + dt;

// Read Latched Data from Sensor Registers
#if (ADNS_READMODE_BURST)

	while (getMicrosElapse(capture.endTime, micros()) < _minSamplePeriodUs)
	{
	};
	SPI.transfer(readout.data, adns_readout_max_size);

	// Release SPI Bus
	deselect();
#else
	// Read Motion-Register
	delayMicroseconds(ADNS_DELAYMICROS_READ_ADDR_DATA);
	uint8_t motion = SPI.transfer(0);
	delayMicroseconds(ADNS_DELAYMICROS_NCSINACTIVE_POST_READ);
	deselect();
	delayMicroseconds(ADNS_DELAYMICROS_POST_READ);

	// Update Current Sample Displacement Register Values
	readout.motion = motion;
	if (bit_is_set(motion, 7))
	{
		readout.dxL = readRegister(RegisterAddress::Delta_X_L);
		readout.dxH = readRegister(RegisterAddress::Delta_X_H);
		readout.dyL = readRegister(RegisterAddress::Delta_Y_L);
		readout.dyH = readRegister(RegisterAddress::Delta_Y_H);
	}
	else
	{
		readout.dxL = 0x00;
		readout.dxH = 0x00;
		readout.dyL = 0x00;
		readout.dyH = 0x00;
	}
#endif

	// Update Displacement
	displacement.dx += ((int16_t)(readout.dxL) | ((int16_t)(readout.dxH) << 8));
	displacement.dy += ((int16_t)(readout.dyL) | ((int16_t)(readout.dyH) << 8));
	displacement.dt += dt; //((capture.endTime - capture.startTime));

#if (ADNS_POSITIONUPDATE_AUTOMATIC)
	triggerPositionUpdate();
#endif
}

// Update _currentPosition and Reset _currentSample
void ADNS::triggerPositionUpdate()
{
	// Local References
	adns_sample_t &sample = _lastSample;
	adns_position_t &position = _currentPosition;
	adns_displacement_t &displacement = _currentSample.displacement;

	// Set Timestamp with prior Position Time
	noInterrupts();
	sample.timestamp = position.t;

	// Update Current Position Data
	position.x += displacement.dx;
	position.y += displacement.dy;
	position.t += displacement.dt;

	// Copy & Reset Displacement
	sample.displacement = displacement;
	displacement = {0, 0, 0};
	interrupts();
}

void ADNS::triggerAcquisitionStop() { _runningFlag = false; }

// =============================================================================
// Data-Sample Conversion & Access
// =============================================================================
//todo: homogenize and combine these functions
displacement_t ADNS::readDisplacement(const unit_specification_t unit) const
{
	// Retrieve last displacement sample
	const adns_displacement_t &displacement = _lastSample.displacement;

	// Initialize sample return structure
	displacement_t u;

	// Pre-Compute Conversion Coefficients for Efficiency
	const float distancePerCount = Unit::perInch(unit.distance) * _resolutionInchPerCount;
	const float timePerCount = Unit::perMicrosecond(unit.time);

	// Apply Conversion Coefficient
	u.dx = (float)displacement.dx * distancePerCount;
	u.dy = (float)displacement.dy * distancePerCount;
	u.dt = (float)displacement.dt * timePerCount;
	return u;
}

position_t ADNS::readPosition(const unit_specification_t unit) const
{
	// Retrieve last displacement sample
	const adns_position_t &position = _currentPosition;

	// Initialize sample return structure
	position_t p;

	// Pre-Compute Conversion Coefficients for Efficiency
	const float distancePerCount = Unit::perInch(unit.distance) * _resolutionInchPerCount;
	const float timePerCount = Unit::perMicrosecond(unit.time);

	// Apply Conversion Coefficient
	p.x = (float)position.x * distancePerCount;
	p.y = (float)position.y * distancePerCount;
	p.t = position.t * timePerCount;
	return p;
}

velocity_t ADNS::readVelocity(const unit_specification_t unit) const
{
	// Retrieve last displacement sample
	const adns_displacement_t &displacement = _lastSample.displacement;

	// Initialize sample return structure
	velocity_t v;

	// Pre-Compute Conversion Coefficients for Efficiency
	const float distancePerCount = Unit::perInch(unit.distance) * _resolutionInchPerCount;
	const float timePerCount = Unit::perMicrosecond(unit.time);
	const float distancePerTimeInterval = distancePerCount * 1 / (timePerCount * (float)displacement.dt);

	// Apply Conversion Coefficient
	v.x = (float)displacement.dx * distancePerTimeInterval;
	v.y = (float)displacement.dy * distancePerTimeInterval;
	return v;
}

void ADNS::printLast()
{
	// Trigger Capture (and position update)
	triggerSampleCapture();
#if (!ADNS_POSITIONUPDATE_AUTOMATIC)
	triggerPositionUpdate();
#endif
	// Ensure Serial Stream has Started
	if (!Serial)
	{
		Serial.begin(115200);
		delay(50);
	}
	// Print Timestamp of Last Sample
	Serial.print("\ntimestamp [us]:\t");
	Serial.println(_lastSample.timestamp);

	// Initialize Unit-Specification and Description Variables
	unit_specification_t unitType;
	String xyUnit;
	String tUnit;

	// Print Displacement
	unitType = {Unit::Distance::MICROMETER, Unit::Time::MICROSECOND};
	xyUnit = Unit::getAbbreviation(unitType.distance);
	tUnit = Unit::getAbbreviation(unitType.time);
	displacement_t u = readDisplacement(unitType);
	Serial.print("<dx,dy,dt> [" + xyUnit + "," + xyUnit + "," + tUnit + "]\t<");
	Serial.print(u.dx, 3);
	Serial.print(",");
	Serial.print(u.dy, 3);
	Serial.print(",");
	Serial.print(u.dt);
	Serial.println(">\t");

	// Print Position
	unitType = {Unit::Distance::MILLIMETER, Unit::Time::MILLISECOND};
	xyUnit = Unit::getAbbreviation(unitType.distance);
	tUnit = Unit::getAbbreviation(unitType.time);
	position_t p = readPosition(unitType);
	Serial.print("<x,y,t> [" + xyUnit + "," + xyUnit + "," + tUnit + "]\t<");
	Serial.print(p.x, 3);
	Serial.print(",");
	Serial.print(p.y, 3);
	Serial.print(",");
	Serial.print(p.t);
	Serial.println(">\t");

	// Print Velocity
	unitType = {Unit::Distance::METER, Unit::Time::SECOND};
	xyUnit = Unit::getAbbreviation(unitType.distance);
	tUnit = Unit::getAbbreviation(unitType.time);
	velocity_t v = readVelocity(unitType);
	Serial.print("<Vx,Vy> [" + xyUnit + "/" + tUnit + "," + xyUnit + "/" + tUnit + "]\t<");
	Serial.print(v.x, 3);
	Serial.print(",");
	Serial.print(v.y, 3);
	Serial.println(">\t");
}

// =============================================================================
// Sensor Settings
// =============================================================================
void ADNS::setResolutionCountsPerInch(const uint16_t cpi)
{
	// Input may take any value between 50 and 8200 (will be rounded to nearest 50
	// cpi)
	uint16_t cpiValid =
		constrain(cpi, ADNS_RESOLUTION_MIN_CPI, ADNS_RESOLUTION_MAX_CPI);
	// uint8_t data = readRegister(RegisterAddress::Configuration_I);
	// Keep current values from reserved bits (0x3f = B00111111) note: data sheet
	// has error, mask is 0xFF uint8_t mask = ADNS_RESOLUTION_REGISTER_MASK; data
	// = (data & ~mask) | (((uint8_t)(cpi / (uint16_t)ADNS_RESOLUTION_MIN_CPI)) &
	// mask);
	uint8_t data = (uint8_t)(cpiValid / (uint16_t)ADNS_RESOLUTION_MIN_CPI);
	writeRegister(RegisterAddress::Configuration_I, data);
	// Read back resolution from same register to confirm and store in cached
	// property
	getResolutionCountsPerInch(); // todo: check resolution matches assigned resolution ->
								  // report
}

uint16_t ADNS::getResolutionCountsPerInch()
{
	uint8_t mask = ADNS_RESOLUTION_REGISTER_MASK;
	uint8_t data = readRegister(RegisterAddress::Configuration_I);
	data = data & mask;
	uint16_t cpi = (uint16_t)data * (uint16_t)ADNS_RESOLUTION_MIN_CPI;
	_resolutionCountsPerInch = cpi;
	_resolutionInchPerCount = 1.0f / (float)cpi;
	return cpi;
}

void ADNS::setMaxSamplePeriodUs(const uint16_t us)
{
	/* Configures sensor hardware -> sets the maximum frame period (minimum frame
  rate) that can be selected by the automatic frame rate control, OR the actual
  frame rate if the sensor is placed in manual frame rate control mode
  */
	uint8_t dataL, dataH;
	uint16_t delayNumCyles = us * ADNS_CHIP_FREQ_MHZ;
	dataL = lowByte(delayNumCyles);
	dataH = highByte(delayNumCyles);
	writeRegister(RegisterAddress::Frame_Period_Max_Bound_Lower, dataL);
	writeRegister(RegisterAddress::Frame_Period_Max_Bound_Upper, dataH);
	getMaxSamplePeriodUs();
}

uint16_t ADNS::getMaxSamplePeriodUs()
{
	uint8_t dataL, dataH;
	dataH = readRegister(RegisterAddress::Frame_Period_Max_Bound_Upper);
	dataL = readRegister(RegisterAddress::Frame_Period_Max_Bound_Lower);
	uint16_t us = makeWord(dataH, dataL) / ADNS_CHIP_FREQ_MHZ;
	_maxSamplePeriodUs = us;
	return us;
}

void ADNS::setMinSamplePeriodUs(const uint16_t us)
{
	// todo ensure frameperiod_maxbound >= frameperiod_minbound + shutter_maxbound
	uint8_t dataL, dataH;
	uint16_t delayNumCyles = us * ADNS_CHIP_FREQ_MHZ;
	dataL = lowByte(delayNumCyles);
	dataH = highByte(delayNumCyles);
	writeRegister(RegisterAddress::Frame_Period_Min_Bound_Lower, dataL);
	writeRegister(RegisterAddress::Frame_Period_Min_Bound_Upper, dataH);
	getMinSamplePeriodUs();
}

uint16_t ADNS::getMinSamplePeriodUs()
{
	uint8_t dataL, dataH;
	dataH = readRegister(RegisterAddress::Frame_Period_Min_Bound_Upper);
	dataL = readRegister(RegisterAddress::Frame_Period_Min_Bound_Lower);
	uint16_t us = makeWord(dataH, dataL) / ADNS_CHIP_FREQ_MHZ;
	_minSamplePeriodUs = us;
	return us;
}

// =============================================================================
// Sensor Status
// =============================================================================
uint16_t ADNS::getSamplePeriodUs()
{
	uint8_t dataL, dataH;
	dataH = readRegister(RegisterAddress::Frame_Period_Upper);
	dataL = readRegister(RegisterAddress::Frame_Period_Lower);
	uint16_t us = makeWord(dataH, dataL) / ADNS_CHIP_FREQ_MHZ;
	return us;
}

uint16_t ADNS::getSampleRateHz()
{
	uint16_t us = getSamplePeriodUs();
	return (uint16_t)(1000000UL / (uint32_t)us);
}

// =============================================================================
// Sensor Communication (SPI)
// =============================================================================
void ADNS::select()
{
	if (_selectedFlag == false)
	{
		SPI.beginTransaction(SPISettings(ADNS_SPI_MAX_SPEED, ADNS_SPI_BIT_ORDER,
										 ADNS_SPI_DATA_MODE));
		digitalWriteFast(_chipSelectPin, LOW);
		_selectedFlag = 1;
		delayMicroseconds(1);
	}
}

void ADNS::deselect()
{
	if (_selectedFlag == true)
	{
		// delayMicroseconds(1); // tSCLK-NCS
		digitalWriteFast(_chipSelectPin, HIGH);
		SPI.endTransaction();
		_selectedFlag = 0;
	}
}

uint8_t ADNS::readRegister(const RegisterAddress address)
{
	// Send 7-bit address with msb=0, clock for 1 uint8_t to receive 1 uint8_t
	select();
	// waitNextTransactionDelay(); //TODO use getMicrosElapse
	SPI.transfer((uint8_t)address & 0x7f);
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
	SPI.transfer((uint8_t)address | 0x80);
	SPI.transfer(data);
	// setNextTransactionDelay(ADNS_DELAYMICROS_POST_WRITE); //TODO getMicrosElapse
	delayMicroseconds(ADNS_DELAYMICROS_NCSINACTIVE_POST_WRITE);
	deselect();
	delayMicroseconds(ADNS_DELAYMICROS_POST_WRITE);
}

// =============================================================================
// Mode
// =============================================================================
void ADNS::setMotionSensePinInterruptMode(const int pin)
{
	_motionSensePin = pin;
	// todo: set flag and use timer to poll if using this mode??
	// pinModeFast(pin, INPUT_PULLUP);
	// attachInterrupt(digitalPinToInterrupt(pin), triggerSampleCapture, LOW);
	// todo: interrupt requires a static member function
	// SPI.usingInterrupt(digitalPinToInterrupt(pin));
}

// =============================================================================
// Configuration
// =============================================================================
void ADNS::initialize()
{
	if (!_initializedFlag)
	{
		// Set up Serial Peripheral Interface (SPI) & specified chip-select pin
		pinModeFast(_chipSelectPin, OUTPUT);
		SPI.begin();
		delay(100);
	}
	if (!_configuredFlag)
	{
		// Power-Up Sensor & Set/Confirm Settings on Sensor Device
		powerUpSensor();
		setResolutionCountsPerInch(_resolutionCountsPerInch);
		setMaxSamplePeriodUs(_maxSamplePeriodUs);
		getMinSamplePeriodUs(); // todo update period and resolution in single fcn
		delaySleepTimeout();
		setMaxLiftDetectionThreshold();
		_configuredFlag = true;
	}
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
	delayMicroseconds(max(2000, _maxSamplePeriodUs));
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
	// Firmware supplied by chip manufacturer (Pixart) as a hex file must be
	// uploaded each time the sensor is powered on. Hex-file is defined in array &
	// stored in flash ram in simple header-file ("firmware/adns9800_srom_A6.h")
	// and can easily be interchangeable with updated firmware. Put sensor in
	// firmware upload mode
	writeRegister(RegisterAddress::Configuration_IV, 0x02);
	writeRegister(RegisterAddress::SROM_Enable, 0x1d);
	delay(10);
	writeRegister(RegisterAddress::SROM_Enable, 0x18);
	// SROM Load Burst Sequence
	select();
	SPI.transfer((uint8_t)RegisterAddress::SROM_Load_Burst | 0x80);
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
	data = (data & ADNS_LASER_CTRL0_REGISTER_MASK); // TODO NOT RIGHT SCREW IT
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
	data = (data & ~ADNS_LIFT_DETECTION_REGISTER_MASK) |
		   (0xFF & ADNS_LIFT_DETECTION_REGISTER_MASK);
	writeRegister(RegisterAddress::Lift_Detection_Thr, data);
}

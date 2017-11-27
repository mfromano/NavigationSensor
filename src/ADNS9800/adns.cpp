/*
  ADNS.cpp - Library for communicating with ADNS-9800 laser mouse sensor.
  Created by Mark Bucklin, May 21, 2014.
  Adapted from mrjohnk: https://github.com/mrjohnk/ADNS-9800
  Updated 7/1/2017
*/

#include "adns.h"
#include <elapsedMillis.h>
#include <digitalWriteFast.h>

// =============================================================================
//   Setup
// =============================================================================
bool ADNS::begin(const uint16_t cpi, const uint16_t hz)
{
	_resolutionCountsPerInch = cpi;
	_minSampleRate = hz;
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

	// Set Start-Time Microsecond Offset
	adns_time_t us = micros();
	adns_capture_t &sample = _currentCapture;
	sample.startTime = 0;
	sample.endTime = 0;
	sample.readout.motion = 0x00;
	_acquisitionStartMicrosCountOffset = us;
	// // _lastSampleMicrosCount = us;

	// Standard Delays
	delayMicroseconds(ADNS_DELAYMICROS_NCSINACTIVE_POST_WRITE);
	deselect();
	delayMicroseconds(ADNS_DELAYMICROS_POST_WRITE);

	// Set Flag to Indicate Running State
	_runningFlag = true;
}

// Read from ADNS9800 Sensor and Update _currentCapture & _currentDisplacement
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
	adns_displacement_t &displacement = _currentDisplacement;
	adns_readout_t &readout = capture.readout;

	// todo noInterrupts();

	// Copy Start-Time from Prior Sample Finish-Time
	capture.startTime = capture.endTime;
	usStart = capture.startTime + _acquisitionStartMicrosCountOffset;

	// Run Read-Register Sequence with Timestamp Inserted Directly after Address
	select();

	// Read from Motion or Motion_Burst Address to Latch Data
	SPI.transfer(ADNS::_motionLatchRegAddr & 0x7f);

	// Record Sample Finish-Time
	usFinish = micros();
	dt = getMicrosElapse(usStart, usFinish);
	capture.endTime = capture.startTime + dt;

	// Read Latched Data from Sensor Registers
	if (ADNS::_burstModeFlag)
	{
		while (getMicrosElapse(capture.endTime, micros()) < _minSamplePeriod)
		{
		};
		for (uint8_t k = 0; k < adns_readout_max_size; k++)
		{
			readout.data[k] = SPI.transfer(0x00);
		}
	}
	else
	{
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
	}

	// Release SPI Bus
	deselect();

	// Update Displacement
	displacement.dx += ((int16_t)(readout.dxL) | ((int16_t)(readout.dxH) << 8));
	displacement.dy += ((int16_t)(readout.dyL) | ((int16_t)(readout.dyH) << 8));
	displacement.dt += ((capture.endTime - capture.startTime));

	if (_autoUpdateFlag)
		triggerPositionUpdate();

	// todo interrupts();
}

// Update _currentPosition and Reset _currentDisplacement
void ADNS::triggerPositionUpdate()
{
	// Local References
	adns_sample_t &sample = _lastSample;
	adns_position_t &position = _currentPosition;
	adns_displacement_t &displacement = _currentDisplacement;

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
displacement_t ADNS::readDisplacement()
{
	// Retrieve last displacement sample
	const adns_displacement_t &displacement = _lastSample.displacement;

	// Initialize sample return structure
	displacement_t u;

	// Convert Displacement Since Last Read to Velocity Sample
	const float &ipc = _resolutionInchPerCount; // inches-per-count
	float umPerCnt = 25400.0 * ipc;				// micrometers-per-count //todo constexpr
	u.dx = (float)displacement.dx * umPerCnt;   // micron
	u.dy = (float)displacement.dy * umPerCnt;   // micron
	u.dt = (float)displacement.dt;				// microseconds // todo: standard units
	return u;
}

position_t ADNS::readPosition()
{
	// Retrieve last displacement sample
	const adns_position_t &position = _currentPosition;

	// Initialize sample return structure
	position_t p;

	// Convert Displacement Since Last Read to Velocity Sample
	const float &ipc = _resolutionInchPerCount; // inches-per-count
	float mmPerCnt = 25.4 * ipc;				// millimeters-per-count //todo constant conversion when resolution set
	p.x = (float)position.x * mmPerCnt;			// millimeters
	p.y = (float)position.y * mmPerCnt;			// millimeters
	p.t = position.t;							// microseconds // todo: standard units
	return p;
}

velocity_t ADNS::readVelocity()
{
	// Retrieve last displacement sample
	const adns_displacement_t &displacement = _lastSample.displacement;

	// Initialize sample return structure
	velocity_t v;

	// Convert Displacement Since Last Read to Velocity Sample
	const float &ipc = _resolutionInchPerCount; // inches-per-count

	float cmPerCntSec = (2.54 * ipc * 1000000.0f) / ((float)displacement.dt);
	v.x = (float)displacement.dx * cmPerCntSec; // centimeters
	v.y = (float)displacement.dy * cmPerCntSec; // centimeters

	return v;
}

void ADNS::printLast()
{
	// // struct
	// // {
	// // 	uint32_t capture;
	// // 	uint32_t update;
	// // 	uint32_t velocity;
	// // } t;
	// // elapsedMicros elap(0);

	// // triggerSampleCapture();
	// // t.capture = elap;
	// // elap = 0;
	// // if (!_autoUpdateFlag)
	// // {
	// // 	triggerPositionUpdate();
	// // 	t.update = elap;
	// // 	elap = 0;
	// // }
	// // velocity_t v = readVelocity();
	// // t.velocity = elap;

	triggerSampleCapture();
	if (!_autoUpdateFlag)
	{
		triggerPositionUpdate();
	}
	displacement_t u = readDisplacement();
	position_t p = readPosition();
	velocity_t v = readVelocity();

	if (!Serial)
	{
		Serial.begin(115200);
		delay(50);
	}
	// Print Timestamp of Last Sample
	Serial.println(_lastSample.timestamp);

	// Print Displacement
	Serial.print("<dx,dy,dt> [um,um,us]:\t<");
	Serial.print(u.dx);
	Serial.print(",");
	Serial.print(u.dy);
	Serial.print(",");
	Serial.print(u.dt);
	Serial.println(">\t");

	// Print Position
	Serial.print("<x,y,t> [mm,mm,us]:\t<");
	Serial.print(p.x);
	Serial.print(",");
	Serial.print(p.y);
	Serial.print(",");
	Serial.print(p.t);
	Serial.println(">\t");

	// Print Velocity
	Serial.print("<Vx,Vy> [cm/s]:\t<");
	Serial.print(v.x);
	Serial.print(",");
	Serial.print(v.y);
	Serial.println(">\t");

	// // // Print Elapsed Time //todo remove
	// // Serial.print("\tcapture: ");
	// // Serial.print(t.capture);
	// // Serial.print("\tupdate: ");
	// // Serial.print(t.update);
	// // Serial.print("\tvelocity: ");
	// // Serial.println(t.velocity);
}

// =============================================================================
// Output Unit
// =============================================================================
void ADNS::setDisplacementUnits(Unit::Distance distance, Unit::Time time)
{
	ADNS::dist_time_unit_t &unit = _unit.displacement;
	unit.distance = distance;
	unit.time = time;
}
void ADNS::setPositionUnits(Unit::Distance distance, Unit::Time time)
{
	ADNS::dist_time_unit_t &unit = _unit.position;
	unit.distance = distance;
	unit.time = time;
}
void ADNS::setVelocityUnits(Unit::Distance distance, Unit::Time time)
{
	ADNS::dist_time_unit_t &unit = _unit.velocity;
	unit.distance = distance;
	unit.time = time;
}

// // void ADNS::setDistanceUnit(Unit::Distance unit)
// // {
// // 	_unit.distance = unit;
// // }

// // void ADNS::setTimeUnit(Unit::Time unit)
// // {
// // 	_unit.time = unit;
// // }

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
// Sensor Settings
// =============================================================================
void ADNS::setResolution(const uint16_t cpi)
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
	getResolution(); // todo: check resolution matches assigned resolution ->
					 // report
}

uint16_t ADNS::getResolution()
{
	uint8_t mask = ADNS_RESOLUTION_REGISTER_MASK;
	uint8_t data = readRegister(RegisterAddress::Configuration_I);
	data = data & mask;
	uint16_t cpi = (uint16_t)data * (uint16_t)ADNS_RESOLUTION_MIN_CPI;
	_resolutionCountsPerInch = cpi;
	_resolutionInchPerCount = 1.0f / (float)cpi;
	return cpi;
}

void ADNS::setMaxSamplePeriod(const uint16_t us)
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

void ADNS::setMinSamplePeriod(const uint16_t us)
{
	// todo ensure frameperiod_maxbound >= frameperiod_minbound + shutter_maxbound
	uint8_t dataL, dataH;
	uint16_t delayNumCyles = us * ADNS_CHIP_FREQ_MHZ;
	dataL = lowByte(delayNumCyles);
	dataH = highByte(delayNumCyles);
	writeRegister(RegisterAddress::Frame_Period_Min_Bound_Lower, dataL);
	writeRegister(RegisterAddress::Frame_Period_Min_Bound_Upper, dataH);
	getMinSamplePeriod();
}

uint16_t ADNS::getMinSamplePeriod()
{
	uint8_t dataL, dataH;
	dataH = readRegister(RegisterAddress::Frame_Period_Min_Bound_Upper);
	dataL = readRegister(RegisterAddress::Frame_Period_Min_Bound_Lower);
	uint16_t us = makeWord(dataH, dataL) / ADNS_CHIP_FREQ_MHZ;
	_minSamplePeriod = us;
	return us;
}

void ADNS::setMinSampleRate(const uint16_t hz)
{
	// Convenience function to call inverse function and pass to
	// setMaxSamplePeriod()
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

// =============================================================================
// Mode
// =============================================================================
void ADNS::setMotionSensePinInterruptMode(const int pin)
{
	_motionSensePin = pin;
	// todo: attach interrupt to motion-sense pin
	// pinMode(pin, INPUTPULLUP);
	// attachInterrupt(digitalPinToInterrupt(pin), (*)(update), LOW)
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
		setResolution(_resolutionCountsPerInch);
		setMinSampleRate(_minSampleRate);
		getMinSamplePeriod(); // todo update period and resolution in single fcn
		delaySleepTimeout();
		setMaxLiftDetectionThreshold();
		_configuredFlag = true;
	}
	_currentPosition.x = 0;
	_currentPosition.y = 0;
	_currentPosition.t = 0;
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

// =============================================================================
// Inline Convenience Functions
// =============================================================================
int16_t reg2Int16(uint8_t bL, uint8_t bH)
{
	int16_t iw16 = (int16_t)makeWord(bH, bL);
	return convertTwosComplement(iw16);
};
int16_t convertTwosComplement(int16_t b)
{
	// Convert from 16-BIT 2's complement
	if (b & 0x8000)
	{
		b = -1 * ((b ^ 0xffff) + 1);
	}
	return b;
};
adns_duration_t getMicrosElapse(adns_time_t t1, adns_time_t t2)
{
	adns_duration_t dt = t1 > t2 ? 1 + t1 + ~t2 : t2 - t1;
	return dt;
}

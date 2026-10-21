#pragma once

// VEML7700 and VEML6030 I2C Light Sensor 0..140000 Lux
// Copyright (C) https://muman.ch + https://github.com/mumanchu, 2025.11.01
// If you reuse this software, please include the above copyright notice
/*
For details see
https://muman.ch/muman/index.htm?muman-veml7700-light-sensor.htm

VEML7700 I2C adds = 0x10, one  fixed address
VEML6030 I2C adds = 0x48 (ADDR=VDD) or 0x10 (ADDR=GND), two fixed addresses

DATA SHEET
https://www.vishay.com/docs/84286/veml7700.pdf

APPLICATION NOTES
https://www.vishay.com/docs/84323/designingveml7700.pdf
*/

// For debug output
#define VEML7700_LOG_ERROR(s) Serial.println(s)
//#define VEML7700_LOG_ERROR(s)

typedef unsigned int uint;		// 16 or 32 bits
typedef unsigned long ulong;	// always 32 bits


class VEML7700LightSensor
{
protected:
	TwoWire* wire;
	int i2cAdds;

	// Cached Configuration Register ALS_CONF_0 value
	// saves reading it every time
	uint r0;

	// Pre-calculated values for lux calculation, see configure()
	float luxPerCount;
	bool needsCorrection;
	uint readDelayMs;

	// Time of last read, used by readyToRead(), uses readDelayMs
	ulong lastReadMs;

public:
	// Gain Selection
	enum ALS_GAIN
	{
		X1 = 0,		// x1
		X2 = 1,		// x2
		X1_8 = 2,	// x 1/8
		X1_4 = 3	// x 1/4
	};

	// ALS Integration Time
	enum ALS_IT
	{
		MS25 = 0x0C,	// 25ms
		MS50 = 0x08,	// 50ms
		MS100 = 0x00,	// 100ms
		MS200 = 0x01,	// 200ms
		MS400 = 0x02,	// 400ms
		MS800 = 0x03	// 800ms
	};

	// ALS Persistence
	enum ALS_PERS { P1 = 0, P2 = 1, P4 = 2, P8 = 3 };

	// Power Saving Modes
	enum PSM { MODE1 = 0, MODE2 = 1, MODE3 = 2, MODE4 = 3 };

	bool begin(TwoWire* twoWire, int i2cAddress);
	bool readIdRegister(uint* addressOptionCode, uint* deviceId);
	bool shutDown(bool shutDown);
	bool configure(ALS_GAIN gain, ALS_IT integrationTime);
	bool setHighThreshold(uint highThreshold);
	bool setLowThreshold(uint lowThreshold);
	bool setPersistence(ALS_PERS persistence);
	bool readInterruptStatus(bool* low, bool* high);
	bool setPowerSavingMode(PSM mode, bool enable);
	bool readAmbientLightSensor(uint* alsCount);
	bool readWhiteLightSensor(uint* wlsCount);
	bool readyToRead();
	float computeLux(uint alsCount);
	float correctLux(float lux);
	bool autoReadLux(bool startReading, bool* readingReady, float* lux);

	bool dumpRegisters();

protected:
	bool readRegister(uint reg, uint* value);
	bool writeRegister(uint reg, uint value);
};


// Call 'Wire.begin(400000); Wire.setTimeout(100);' first
// (there can be more than one I2C device sharing Wire)
bool VEML7700LightSensor::begin(TwoWire* twoWire, int i2cAddress)
{
	wire = twoWire;
	i2cAdds = i2cAddress;
	lastReadMs = 0;

	// shut down and set default configuration (r0=0)
	r0 = 0;
	if (!shutDown(true))
		return false;

	// ensure power saving mode is off
	if (!setPowerSavingMode(PSM::MODE1, false))
		return false;

	// power up and restart
	if (!shutDown(false))
		return false;
	delay(3);			// 2.5ms delay after power up

	return true;
}

// Shut Down (stops current reading) or Power Up (starts next reading)
// bool shutDown : true = shut down, false = power up
// needs 2.5ms delay after power up before reading starts
bool VEML7700LightSensor::shutDown(bool shutDown)
{
	// ALS_SD bit 0 : 1 = ALS shut down, 0 = ALS power on
	r0 = (r0 & 0xFFFE) | (shutDown ? 1 : 0);
	return writeRegister(0, r0);
}

// addressOptionCode : 0xC4=I2C address 0x10, 0xD4=I2C address 0x48
// deviceId = 0x81 for both devices (not checked)
bool VEML7700LightSensor::readIdRegister(uint* addressOptionCode, uint* deviceId)
{
	uint value;
	if (!readRegister(0x07, &value))
		return false;
	*addressOptionCode = value >> 8;
	*deviceId = (byte)value;
	return true;
}

/* Write ALS_COF_0 register and set up constants for lux calculation and read delay
   NOTE: The previous reading must complete before the new configuration is used

   bit 54321098 76543210
       000gg0tt ttpp00is
   gg   = ALS_GAIN
   tttt = ALS_IT
   pp   = ALS_PERS
   i    = ALS_INT
   s    = ALS_SD
*/
bool VEML7700LightSensor::configure(ALS_GAIN gain, ALS_IT integrationTime)
{
	// pre-compute values for lux calculation
	float resolution;
	switch (integrationTime) {
	case MS25:
		readDelayMs = 25;
		resolution = 0.1344f;
		break;
	case MS50:
		readDelayMs = 50;
		resolution = 0.0672f;
		break;
	case MS100:
		readDelayMs = 100;
		resolution = 0.0336f;
		break;
	case MS200:
		readDelayMs = 200;
		resolution = 0.0168f;
		break;
	case MS400:
		readDelayMs = 400;
		resolution = 0.0084f;
		break;
	case MS800:
		readDelayMs = 800;
		resolution = 0.0042f;
		break;
	default:
		VEML7700_LOG_ERROR("invalid int time");
		return false;
	}
	// increase by 30% to be safe (timing is +-30%)
	readDelayMs = (readDelayMs * 130) / 100;

	needsCorrection = false;
	float multiplier;
	switch (gain) {
	case X1:
		multiplier = 1.0f;
		break;
	case X2:
		multiplier = 2.0f;
		break;
	case X1_4:
		needsCorrection = true;
		multiplier = 8.0f;
		break;
	case X1_8:
		needsCorrection = true;
		multiplier = 16.0f;
		break;
	default:
		VEML7700_LOG_ERROR("invalid gain");
		return false;
	}
	luxPerCount = resolution * multiplier;

	lastReadMs = millis();

	r0 &= 0x3f;		// preserve bits 5..0 (ALS_PERS ALS_INT ALS_SD)
	r0 += (gain << 11) + (integrationTime << 6);
	return writeRegister(0, r0);
}

// Interrupt thresholds and persistence
bool VEML7700LightSensor::setHighThreshold(uint highThreshold)
{
	return writeRegister(1, highThreshold);
}

bool VEML7700LightSensor::setLowThreshold(uint lowThreshold)
{
	return writeRegister(2, lowThreshold);
}

// Persistence defines the length of time that the light level 
// must remain before the interrupt is raised. It is the number 
// of readings above/below the threshold until the interrupt is
// raised: 1, 2, 4, 8
bool VEML7700LightSensor::setPersistence(ALS_PERS persistence)
{
	r0 = (r0 & 0xFFCF) | ((uint)persistence << 4);
	return writeRegister(0, r0);
}

// Reading below or above the threshold setting
bool VEML7700LightSensor::readInterruptStatus(bool* low, bool* high)
{
	uint r6;
	if (!readRegister(6, &r6)) {
		*low = false;
		*high = false;
		return false;
	}
	*low  = r6 & 0x8000 ? true : false;
	*high = r6 & 0x4000 ? true : false;
	return true;
}

// Power saving mode
// use this for continuous measurements at a slower rate 
// see App Note p15
bool VEML7700LightSensor::setPowerSavingMode(PSM mode, bool enable)
{
	uint r3 = (mode & 3) << 1;
	if (enable)
		r3 |= 1;
	return writeRegister(3, r3);
}

// Raw sensor readings
bool VEML7700LightSensor::readAmbientLightSensor(uint* alsCount)
{
	lastReadMs = millis();
	return readRegister(4, alsCount);
}

bool VEML7700LightSensor::readWhiteLightSensor(uint* wlsCount)
{
	//lastReadMs = millis();  //TODO?
	return readRegister(5, wlsCount);
}

/* Returns true after readDelayMs milliseconds since last read or config
   Use this in loop() to prevent blocking:
      if (sensor.readyToRead()) {
          uint alsCount;
          sensor.readAmbientLightSensor(&alsCount);
          ...
      }
*/
bool VEML7700LightSensor::readyToRead()
{
	return (millis() - lastReadMs) >= readDelayMs;
}

// Compute lux, based on gain and integration time
float VEML7700LightSensor::computeLux(uint alsCount)
{
	float lux = alsCount * luxPerCount;

	// when using gain 1/4 and 1/8 the correction formula should be used
	return needsCorrection ? correctLux(lux) : lux;
}

// Non-linear correction formula, see App Note p5
// https://www.vishay.com/docs/84323/designingveml7700.pdf#page=5
// lux = 6.0135e-13 * pow(lux, 4) - 9.3924e-9 * pow(lux, 3) + 
//       8.1488e-5 * pow(lux, 2) + 1.0023 * lux;
float VEML7700LightSensor::correctLux(float lux)
{
	/* formula using powers
	float plux = lux;
	float alux = 1.0023f * plux;
	plux *= lux;		// lux ^ 2
	alux += 8.1488e-5f * plux;
	plux *= lux;		// lux ^ 3
	alux += -9.3924e-9f * plux;
	plux *= lux;		// lux ^ 4
	alux += 6.0135E-13f * plux;
	*/

	// this does the same thing but faster (half the number of multiplications)
	return (((6.0135e-13f * lux - 9.3924e-9f) * lux + 8.1488e-5f) * lux + 1.0023f) * lux;
}


/* Read the ALS sensor with automatic gain and integration time adjustment 
   according to the light level. This is a non-blocking implementation of 
   the algorithm in App Note p21
   https://www.vishay.com/docs/84323/designingveml7700.pdf#page=21
   For the white sensor, call this first, then call readWhiteLightSensor() 
   as soon as 'readingReady' is returned, so it uses the same settings.

   This is a NON_BLOCKING function. It must be called from loop() in a 
   special way. With low light levels, the App Note version blocks for over
   2 seconds! This version allows the program to keep running, taking less 
   than 5 microseconds per call while waiting for the reading, or 650us if
   an I2C message is sent.
   
   Usage:
   
    bool startReading = true;    // begin reading
    void loop() 
    {
        float lux;
        bool readingReady;
        sensor.autoReadLux(startReading, &readingReady, &lux);
        startReading = false;    // reading now in progress
        if (readingReady) {      // reading complete
            Serial.printf("%lu lux=%.2f\n\r", millis(), lux);
            Serial.flush();
            startReading = true; // start next reading
        }
        // do other stuff while waiting
        ...
    }
*/
bool VEML7700LightSensor::autoReadLux(bool startReading, bool* readingReady, float* lux)
{
	static const byte time[6] = {
		ALS_IT::MS25, ALS_IT::MS50, ALS_IT::MS100, ALS_IT::MS200, ALS_IT::MS400, ALS_IT::MS800
	};
	static const byte gain[4] = {
		ALS_GAIN::X1_8, ALS_GAIN::X1_4, ALS_GAIN::X1, ALS_GAIN::X2
	};
	static int ixTime;
	static int ixGain;
	
	*readingReady = false;
	*lux = 0.0f;
	uint alsCount = 0;

	// start with 100ms integration time and lowest gain 1/8
	if (startReading) {
		ixTime = 2;
		ixGain = 0;
	}
	else
		goto read;		// naughty, but very simple

	while (true) {
		/* set the new configuration
		   NOTE: The flowchart on p21 shows ALS_SD=1 (shutDown(true)) and ALS_SD=0 
		   (shutDown(false)). I think shutDown() is used because the new gain and 
		   integration time are used immediately instead of after the end of the 
		   current reading. The Adafruit library uses double-length delays instead, 
		   but you don't need those if shutDown() is used. shutDown() is faster.
		*/
		shutDown(true);		// stop current reading
		if (!configure((ALS_GAIN)gain[ixGain], (ALS_IT)time[ixTime]))
			return false;
		shutDown(false);	// start new reading
		return true;		// non-blocking, call again with startReading=false to read

	read:
		if (!readyToRead())
			return true;	// not ready yet, return

		if (!readAmbientLightSensor(&alsCount))
			return false;

		// if count <= 100, increase gain to max, then increase integration time
		if (alsCount <= 100) {
			if (ixGain < 3)
				++ixGain;
			else if (ixTime < 5)
				++ixTime;
			else
				break;
		}

		// if count > 100, decrease integration time until count <= 10000 
		else {
			if (alsCount <= 10000 || ixTime == 0)
				break;
			--ixTime;
		}
	}
	*lux = computeLux(alsCount);
	*readingReady = true;
	return true;
}

#if 0
// This is the original blocking version - it hangs the program for up to 2 seconds!
bool VEML7700LightSensor::autoReadLux(float* lux)
{
	static const byte time[6] = {
		ALS_IT::MS25, ALS_IT::MS50, ALS_IT::MS100, ALS_IT::MS200, ALS_IT::MS400, ALS_IT::MS800
	};
	static const byte gain[4] = {
		ALS_GAIN::X1_8, ALS_GAIN::X1_4, ALS_GAIN::X1, ALS_GAIN::X2
	};
	*lux = 0.0f;
	uint alsCount = 0;

	// start with 100ms integration time and lowest gain 1/8
	int ixTime = 2;
	int ixGain = 0;

	while (true) {
		shutDown(true);		// stop current reading
		if (!configure((ALS_GAIN)gain[ixGain], (ALS_IT)time[ixTime]))
			return false;
		shutDown(false);	// start new reading

		// read count after the delay (blocking)
		delay(readDelayMs);
		if (!readAmbientLightSensor(&alsCount))
			return false;

		// if count <= 100, increase gain to max, then increase integration time
		if (alsCount <= 100) {
			if (ixGain < 3)
				++ixGain;
			else if (ixTime < 5)
				++ixTime;
			else
				break;
		}

		// if count > 100, decrease integration time until count <= 10000 
		else {
			if (alsCount <= 10000 || ixTime == 0)
				break;
			--ixTime;
		}
	}
	*lux = computeLux(alsCount);
	return true;
}
#endif

//#ifdef DEBUG
bool VEML7700LightSensor::dumpRegisters()
{
	char s[100];
	for (uint reg = 0; reg < 8; ++reg) {
		uint value;
		if (!readRegister(reg, &value))
			return false;
		sprintf(s, "%02X = 0x%04X", reg, value);
		Serial.println(s);
	}
	Serial.println("");
	return true;
}
//#endif


// Internal methods
bool VEML7700LightSensor::readRegister(uint reg, uint* value)
{
	*value = 0;
	wire->beginTransmission(i2cAdds);
	wire->write((byte)reg);
	if (wire->endTransmission(false) != 0) {
		VEML7700_LOG_ERROR("endtx failed");
		return false;
	}
	if (wire->requestFrom(i2cAdds, 2) != 2) {
		VEML7700_LOG_ERROR("requestFrom failed");
		return false;
	}
	uint v = wire->read();			// lsb first
	v += wire->read() << 8;
	*value = v;
	return true;
}

bool VEML7700LightSensor::writeRegister(uint reg, uint value)
{
	wire->beginTransmission(i2cAdds);
	wire->write((byte)reg);
	wire->write((byte)value);		// lsb first
	wire->write((byte)(value >> 8));
	if (Wire.endTransmission() != 0) {
		VEML7700_LOG_ERROR("endtx failed");
		return false;
	}
	return true;
}

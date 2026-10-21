// VEML7700 Library Example
// Copyright (C) muman.ch, 2025.11.01
// email: info@muman.ch
// For details see
// https://muman.ch/muman/index.htm?muman-veml7700-light-sensor.htm

#include <Wire.h>

#include "VEML7700LightSensor.h"
VEML7700LightSensor sensor;

bool startReading;

void setup() 
{
	Serial.begin(115200);
	delay(500);
	Serial.println("\n\n\rStarted\n\r");

	Wire.begin();
	Wire.setClock(400000);
	Wire.setTimeout(100);		// or Wire.setWireTimeout(100);

	if (!sensor.begin(&Wire, 0x10)) {	// 0x10 or 0x48
		Serial.println("Sensor not responding, halted");
		Serial.flush();
		while (1);
	}
	sensor.dumpRegisters();

	// start first reading
	startReading = true;
}

void loop() 
{
	char s[100];

	float lux;
	bool readingReady;
	sensor.autoReadLux(startReading, &readingReady, &lux);
	startReading = false;		// reading now in progress
	if (readingReady) {			// reading ready
		sprintf(s, "%lu lux=%.2f", millis(), lux);
		//use this if you don't have %f float support
		//sprintf(s, "%lu lux=%lu", millis(), (ulong)lux);
		Serial.println(s);
		Serial.flush();
		startReading = true;	// start next reading
	}

	// do other stuff while waiting

}

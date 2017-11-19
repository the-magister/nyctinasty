#ifndef Skein_Messages_h
#define Skein_Messages_h

#include <Arduino.h>

#include <EEPROM.h>

// common sensor structure
#define N_SENSOR 8
typedef struct {
	// out-of-range
	uint16_t min,max;
	
	// noise level in readings
	uint16_t noise; 
	
	// distance
	uint16_t dist[N_SENSOR];
} SensorReading;

// common command structure; commited to EEPROM and recalled at startup.
typedef struct {
	// send fps, frame = SensorReading
	uint8_t fps;
} Command;

// with a default 
const Command defaultCommand = { 20 };

// save/load to/from EEPROM
void saveCommand(Command command);
void loadCommand(Command command);

#endif
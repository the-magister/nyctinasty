#ifndef Skein_Messages_h
#define Skein_Messages_h

#include <Arduino.h>

#include <EEPROM.h>

// common sensor structure
#define N_SENSOR 8
typedef struct {
	// out-of-range
	word min,max;
	
	// noise level in readings
	word noise; 
	
	// distance
	word dist[N_SENSOR];
} SensorReading;

// common command structure; commited to EEPROM and recalled at startup.
typedef struct {
	// send fps, frame = SensorReading
	byte fps;
} Command;

// save/load to/from EEPROM
void saveCommand(Command command);
void loadCommand(Command command);


#endif
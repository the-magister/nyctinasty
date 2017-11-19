#include "Skein_Messages.h"

#define EEPROM_LOCATION 157

// save/load to/from EEPROM
void saveCommand(Command command) {
	EEPROM.put(EEPROM_LOCATION, command);

//	Serial.print("Saving EEPROM value for fps=");
//	Serial.println(command.fps);
	
}

Command loadCommand() {
	Command command;
	EEPROM.get(EEPROM_LOCATION, command);

//	Serial.print("Reading EEPROM value for fps=");
//	Serial.println(command.fps);
	
	// unitialized?
	if( command.fps == 255 | command.fps == 0 ) {
		Serial.println("Unintialized");
		command.fps = defaultFPS;
	}
	
	return( command );
}


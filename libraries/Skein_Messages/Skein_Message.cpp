#include "Skein_Messages.h"

#define EEPROM_LOCATION 157

// save/load to/from EEPROM
void saveCommand(Command command) {
#if defined(ESP8266) 
	EEPROM.begin(128);
#endif

	EEPROM.put(EEPROM_LOCATION, command);

//	Serial.print("Saving EEPROM value for fps=");
//	Serial.println(command.fps);

#if defined(ESP8266) 
	EEPROM.end();
#endif

}

Command loadCommand() {
#if defined(ESP8266) 
	EEPROM.begin(128);
#endif

	Command command;
	EEPROM.get(EEPROM_LOCATION, command);

#if defined(ESP8266) 
	EEPROM.end();
#endif
	
//	Serial.print("Reading EEPROM value for fps=");
//	Serial.println(command.fps);
	
	// unitialized?
	if( command.fps == 255 | command.fps == 0 ) {
		Serial.println("Unintialized");
		command.fps = defaultFPS;
	}
	
	return( command );
}


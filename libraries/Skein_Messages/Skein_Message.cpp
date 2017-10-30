#include "Skein_Messages.h"

#define EEPROM_LOCATION 157

// save/load to/from EEPROM
void saveCommand(Command command) {
	EEPROM.put(EEPROM_LOCATION, command);
}

void loadCommand(Command command) {
	EEPROM.get(EEPROM_LOCATION, command);
}


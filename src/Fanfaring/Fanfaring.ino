// IDE Settings:
// Tools->Board : "WeMos D1 R2 & mini"
// Tools->Flash Size : "4M (3M SPIFFS)"
// Tools->CPU Frequency : "160 MHz"
// Tools->Upload Speed : "921600"

#include <Streaming.h>
#include <Metro.h>
#include <SoftwareSerial.h>
#include "Nyctinasty_Messages.h"
#include "Nyctinasty_Comms.h"

// My role and arch number
NyctRole myRole = Fanfaring; // see Nyctinasty_Comms.h; set N_ROLES to pull from EEPROM

// comms
NyctComms comms;

// publish system state
SystemCommand sC;
void setup() {
  // for local output
  delay(500);
  Serial.begin(115200);

  Serial << endl << endl << endl << F("Startup begins.") << endl;

  // configure comms
  comms.begin(myRole);
  myRole = comms.getRole();

  Serial << "Publishing..." << endl;
  
  while( ! comms.isConnected() ) {
    delay(500);
    comms.update();
  }
  sC.state = WINNING;
  comms.publish(&sC);
  
  Serial << F("Startup complete.") << endl;
}

void loop() {
  // comms handling
  comms.update();

  static Metro publishInterval(2000UL);
  if( publishInterval.check() ) {
    sC.state = FANFARE;
    comms.publish(&sC);
  }
}


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

struct cT_t {
  boolean hasUpdate = false;
  CannonTrigger cannon;
} cT;

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
  
  comms.subscribe(&cT.cannon, &cT.hasUpdate);

  while( ! comms.isConnected() ) {
    delay(500);
    comms.update();
  }
  sC.state = LONELY;
  comms.publish(&sC);

  Serial << F("Startup complete.") << endl;
}

void loop() {
  // comms handling
  comms.update();

  static Metro timeout(60UL * 1000UL);
  if( timeout.check() ) {
    sC.state = LONELY;
    comms.publish(&sC);
    Serial << "Not triggered.  LONELY." << endl;
  }

  // check for settings update
  if ( cT.hasUpdate && ( cT.cannon.left == TRIGGER_ON || cT.cannon.right == TRIGGER_ON )) {
    Serial << "Cannon. left=" << cT.cannon.left << " right=" << cT.cannon.right << endl;
    cT.hasUpdate = false;
  
    if( sC.state != FANFARE ) {
      Serial << "Triggered.  FANFARE." << endl;
      sC.state = OHAI;  
      comms.publish(&sC);
      delay(1000);
      sC.state = GOODNUF;  
      comms.publish(&sC);
      delay(1000);
      sC.state = GOODJOB;  
      comms.publish(&sC);
      delay(1000);
      sC.state = WINNING;  
      comms.publish(&sC);
      delay(1000);
      sC.state = FANFARE;  
      comms.publish(&sC);
    }

    timeout.reset();
  }

  

}


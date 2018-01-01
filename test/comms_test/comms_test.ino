// IDE Settings:
// Tools->Board : "WeMos D1 R2 & mini"
// Tools->Flash Size : "4M (3M SPIFFS)"
// Tools->CPU Frequency : "160 MHz"
// Tools->Upload Speed : "921600"

#include <Metro.h>
#include <Streaming.h>
#include "Nyctinasty_Messages.h"
#include "Nyctinasty_Comms.h"

// my role
NyctRole role = Frequency;

// incoming message storage
struct sC_t {
  boolean hasUpdate;
  SystemCommand settings;
} sC;

// our frequency updates send as this structure
SepalArchFrequency freq;

// comms
NyctComms comms;

void setup() {
  Serial.begin(115200);
  
  comms.begin(role);
  comms.subscribe(&sC.settings, &sC.hasUpdate);
}

void loop() {
  comms.update();

  if( sC.hasUpdate ) {
    Serial << F("Settings update.") << endl;
    sC.hasUpdate = false;
  }

  static Metro sendInterval(1000UL);
  if( sendInterval.check() ) {
    sendInterval.reset();
    comms.publish(&freq);
  }
}

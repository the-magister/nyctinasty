// IDE Settings:
// Tools->Board : "WeMos D1 R2 & mini"
// Tools->Flash Size : "4M (3M SPIFFS)"
// Tools->CPU Frequency : "160 MHz"
// Tools->Upload Speed : "921600"

#define SHOW_SERIAL_DEBUG true

#include <Streaming.h>
#include <Metro.h>
#include <SoftwareSerial.h>
#include <SoftEasyTransfer.h>
#include "Nyctinasty_Messages.h"

// wire it up
// D5..D8 used for LEDS
#define RX D1
#define TX D2

// LED handling

// our distance updates send as this structure as this topic
SepalArchDistance dist;

// talk to the ADC device
SoftwareSerial mySerial(RX, TX, false, 1024);
SoftEasyTransfer ETin;

void setup() {
  // for local output
  Serial.begin(115200);

  Serial << endl << endl << endl << F("Startup begins.") << endl;

  // for remote output
  mySerial.begin(115200);

  // messages
  ETin.begin(details(dist), &mySerial);

  // after set up the input pin
  pinMode(TX, OUTPUT); // trigger to send

  Serial << F("Startup complete.") << endl;
}

void loop() {
  // check to see if we need to pull new distance data.
  static Metro distanceUpdate(10UL);
  if ( distanceUpdate.check() ) {
    distanceUpdate.reset();

    // ask for data
    digitalWrite(TX, HIGH);
  }

  // get data from ADCs
  if ( ETin.receiveData() ) {
    // stop asking for data
    digitalWrite(TX, LOW);
  }

  if ( SHOW_SERIAL_DEBUG ) {
    const char sep[] = ",";

    for ( byte i = 0; i < N_SENSOR; i++ ) Serial << dist.prox[i] << sep;

    Serial << dist.min << sep;
    Serial << dist.noise << sep;
    Serial << dist.max << sep;
    //    Serial << smoothing << sep;
    Serial << endl;
  }

}


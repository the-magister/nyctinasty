/*
   This module is responsible for polling the LIDAR sensors and publishing that data.
   We can have multiple sensors, and this is the i-th sensor.

   Publishes: skein/range/i/[0-7]: distance information, mm
              skein/range/i/oor: distance corresponding to out-of-range, mm
*/

// You'll need to add http://arduino.esp8266.com/stable/package_esp8266com_index.json to the Additional Board Managers URL entry in Preferences.
// Compile for NodeMCU 1.0 (ESP-12E Module), 80 Mhz, 921600 Upload Speed, 4M (3M SPIFFS).
#include <Streaming.h>
#include <Metro.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include "Skein_Comms.h"

// really need to save this to EEPROM
// should we ever need to extend this to more uCs, this will generate an offset.
byte subsetIndex = 1;

const byte Nsensor = 8;
word range[Nsensor];
word outOfRange;

// connect to the MQTT network with this id
String id = "skeinSensor" + String(subsetIndex, 10);

// subscribe and process these topics
String control = "skein/control/#";

SoftwareSerial mySerial(D5, D1); // D2, D1 / RX, TX; cross to other pair.
String message;

void setup(void)  {
  Serial.begin(115200);
  delay(10);
  Serial << endl << endl << "Startup." << endl;

  // for remote output
  mySerial.begin(57600);

  commsBegin(id);
  commsSubscribe(control);

  message.reserve(256);
}

void loop(void) {
  
  // comms handling
  commsUpdate();

  // bail out if no connection
  if ( ! commsConnected() ) return;

  // sensor handling
  static word updateCount = 0;
  if( mySerial.available()>0 ) {
    readPublishRanges();
    updateCount++;
  }

  // send oor information once every 30 seconds
  static Metro oorPubInterval(30UL * 1000UL);
  if ( commsConnected() && oorPubInterval.check() ) {
    String topic = "skein/range/" + String(subsetIndex, 10) + "/oor";
    commsPublish(topic, String(outOfRange, 10));
    oorPubInterval.reset();
  }

  const unsigned long updateInterval = 10000UL;
  static Metro updateTimer(updateInterval);
  if( updateTimer.check() ) {
    Serial << "Update count: " << updateCount << endl;
    Serial << "Updates per second: ";
    Serial.print( (float)updateCount / (float)(updateInterval/1000) );
    Serial << endl;
    updateCount = 0;
  }
  
}

void readPublishRanges() {
  const char delim = ',';
  const char term = '\0';

  // get readings
  for( byte i=0; i<Nsensor; i++ ) {
    message = mySerial.readStringUntil(delim);
    range[i] = message.toInt();
    publishRange(i);
  }

  // get oor
  message = mySerial.readStringUntil(term);
  outOfRange = message.toInt();

}

void publishRange(byte index) {

  String topic = "skein/range/" + String(subsetIndex, 10) + "/" + String(index, 10);
  String message = String(range[index], 10);

  commsPublish(topic, message);
  yield();
}

void commsProcess(String topic, String message) {
  Serial << "<- " << topic << " " << message << "\t=> ";

  Serial << F("WARNING. unknown topic. continuing.");

  Serial << endl;
}



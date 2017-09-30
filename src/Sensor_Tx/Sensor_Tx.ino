/*
   This module is responsible for polling the LIDAR sensors and publishing that data.
   We can have multiple sensors, and this is the i-th sensor.

   Publishes: skein/range/i/[0-7]: distance information, mm
              skein/range/oor: distance corresponding to out-of-range, mm
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
word value[Nsensor];
word range[Nsensor];

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
  static word updateCount = 0;
  
  // comms handling
  commsUpdate();

  // bail out if no connection
  if ( ! commsConnected() ) return;

  // sensor handling
  if( mySerial.available()>0 ) {
    message = mySerial.readStringUntil(',');
    value[0] = message.toInt();
    calculateRange(0);
    publishRange(0);
//    Serial << range[0] << ",";
    
    message = mySerial.readStringUntil(',');
    value[1] = message.toInt();
    calculateRange(1);
    publishRange(1);
//    Serial << range[1] << ",";

    message = mySerial.readStringUntil(',');
    value[2] = message.toInt();
    calculateRange(2);
    publishRange(2);
//    Serial << range[2] << ",";

    message = mySerial.readStringUntil('\n');
    value[3] = message.toInt();
    calculateRange(3);
    publishRange(3);
//    Serial << range[3] << endl;
    
    updateCount++;
  }

  // send oor information once every 30 seconds
  static Metro oorPubInterval(30UL * 1000UL);
  if ( commsConnected() && oorPubInterval.check() ) {
//    commsPublish("skein/range/oor", String(outOfRange, 10));
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

void publishRange(byte index) {

  String topic = "skein/range/" + String(subsetIndex, 10) + "/" + String(index, 10);
  String message = String(range[index], 10);

  commsPublish(topic, message);
  yield();
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}

void calculateRange(byte index) {
  float invValue = (float)constrain(value[index], 10, 1023);
  invValue = 1.0/invValue;

  const float minV = 1.0/1023.0;
  const float maxV = 1.0/10.0;
  float r = mapFloat(invValue, minV, maxV, 0.0, 2040.0)*10.0;
  range[index] = r;
//  Serial.print(range[index]);
//  Serial << endl;
}

void commsProcess(String topic, String message) {
  Serial << "<- " << topic << " " << message << "\t=> ";

  Serial << F("WARNING. unknown topic. continuing.");

  Serial << endl;
}



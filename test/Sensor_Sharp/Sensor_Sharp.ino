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
#include <WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include "Skein_MQTT.h"

// store readings
SensorReading reading;

// store settings
Command settings;

// really need to save this to EEPROM
// should we ever need to extend this to more uCs, this will generate an offset.
byte subsetIndex = 1;

// connect to the MQTT network with this id
String id = "skeinSensor" + String(subsetIndex, 10);

// subscribe to these topics
String settingsTopic = "skein/control/" + String(subsetIndex, 10);
boolean settingsUpdate = false;

// publish to these topics
String rangeTopic = "skein/range/" + String(subsetIndex, 10);

SoftwareSerial mySerial(D5, D1, false, 256); // D2, D1 / RX, TX; cross to other pair.

void setup(void)  {
  Serial.begin(115200);
  delay(10);
  Serial << endl << endl << "Startup." << endl;

  // for remote output
  mySerial.begin(57600);

  // set and send fps to Sensor_ADC
  settings.fps = 20;
  saveCommand(settings);
//  loadCommand(settings);
//  ETout.sendData();

  commsBegin(id, 16);
  commsSubscribe(settingsTopic, &settings, &settingsUpdate);

}

void loop(void) {
  // comms handling
  commsUpdate();

  // bail out if no connection
  if ( ! commsConnected() ) return;

  // sensor handling
  static word updateCount = 0;
  if ( mySerial.available() >= sizeof(reading)+3 ) {
    if( !mySerial.find("MSG") ) return;
    
    byte buffer[sizeof(reading)];
    byte length = mySerial.readBytes(buffer, sizeof(reading));
 //   Serial << "message len:" << length << " reading size:" << sizeof(reading) << endl;
    if( length == sizeof(reading) ) {
      memcpy( (void *)&reading, (void *)&buffer, sizeof(reading) );
 //     Serial << "dist0:" << reading.dist[0] << endl;      
    }

    publishRanges();
    updateCount++;
  }

  // settings handling
  if ( settingsUpdate ) {
    settingsUpdate = false;
    Serial << F("Settings. fps=") << settings.fps << endl;
    saveCommand(settings);
 //   ETout.sendData();
  }

  const unsigned long updateInterval = 10000UL;
  static Metro updateTimer(updateInterval);
  if ( updateTimer.check() ) {
    Serial << endl;
    Serial << "Update count: " << updateCount << endl;
    Serial << "Updates per second: ";
    Serial.print( (float)updateCount / (float)(updateInterval / 1000) );
    Serial << endl;
    updateCount = 0;
  }

}

void publishRanges() {
  // bail out if no connection
  if ( ! commsConnected() ) return;

  commsPublish( rangeTopic, &reading );
  yield();
}




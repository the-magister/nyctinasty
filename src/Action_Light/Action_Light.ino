/*
   This module is responsible for running lighting based on the Lidar sensor

   Subscribes: skein/range/i/#
               skein/range/oor
*/

// You'll need to add http://arduino.esp8266.com/stable/package_esp8266com_index.json to the Additional Board Managers URL entry in Preferences.
// Compile for NodeMCU 1.0 (ESP-12E Module), 80 Mhz, 921600 Upload Speed, 4M (3M SPIFFS).
#include <Streaming.h>
#include <Metro.h>
#define FASTLED_ESP8266_RAW_PIN_ORDER
#include <FastLED.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "Skein_MQTT.h"

// lighting
// How many leds are in the strip?
const byte NUM_LEDS = 1 + N_SENSOR;
// Data pin that led data will be written out over
#define DATA_PIN 12 // GPIO12/D6.
CRGBArray<NUM_LEDS> leds;
const unsigned long targetFPS = 20;

// connect to the MQTT network with this id
byte subsetIndex = 0;
String id = "skeinLight" + subsetIndex;

// subscribe to these topics
String settingsTopic = "skein/control/" + String(subsetIndex, 10);
Command settings;
boolean settingsUpdate = false;

String lidarTopic = "skein/range/0";
SensorReading lidar;
boolean lidarUpdate = false;

String sharpTopic = "skein/range/1";
SensorReading sharp;
boolean sharpUpdate = false;

void setup(void)  {
  Serial.begin(115200);
  delay(20);
  Serial << endl << endl << "Startup." << endl;

  FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, NUM_LEDS);

  commsBegin(id, 16);
  commsSubscribe(settingsTopic, &settings, &settingsUpdate);
  commsSubscribe(lidarTopic, &lidar, &lidarUpdate);
  commsSubscribe(sharpTopic, &sharp, &sharpUpdate);

}

void loop(void) {
  // comms handling
  commsUpdate();

  // bail out if no connection
  if ( ! commsConnected() ) return;

  // settings handling
  if ( settingsUpdate ) {
    settingsUpdate = false;
    Serial << F("Settings. fps=") << settings.fps << endl;
    saveCommand(settings);
    // noting that we're not doing anything with this (currently)
  }

  // lidar handling
  if ( lidarUpdate ) {
    lidarUpdate = false;
    Serial << F("Lidar") << endl;
    float R = (float)(lidar.max) / log2(255.0 + 1.0);
    for (byte i = 0; i < N_SENSOR; i++) {
      byte value = round( pow(2.0, (float)(lidar.max - lidar.dist[i]) / R) - 1.0 );
      leds[i+1] = blend(leds[i+1], CHSV(HUE_BLUE, 255, value), (fract8)128);
    }
  }

  // lidar handling
  if ( sharpUpdate ) {
    sharpUpdate = false;
    Serial << F("Sharp") << endl;
    float R = (float)(sharp.max) / log2(255.0 + 1.0);
    for (byte i = 0; i < N_SENSOR; i++) {
      byte value = round( pow(2.0, (float)(sharp.max - sharp.dist[i]) / R) - 1.0 );
      leds[i+1] = blend(leds[i+1], CHSV(HUE_GREEN, 255, value), (fract8)128);
    }
  }

  // DON'T hammer the LEDs.  the clockless protocol interferes with
  // WiFi interrupt handling.
  static Metro ledUpdate(1000UL / targetFPS);
  if ( ledUpdate.check() ) {
    FastLED.show();
    ledUpdate.reset();
  }
}

/*
void commsProcess(String topic, String message) {

  //  Serial << "<- " << topic << " " << message << "\t=> ";

  if ( topic.equals(oor) ) {

    outOfRange = message.toInt();
    R = (float)(outOfRange) / log2(255.0 + 1.0);
    Serial << "oor=" << outOfRange << "\tR=" << R;
  } else if ( topic.startsWith("skein/range/0") ) {  // lidar
    // take the last character of the topic as the range index
    topic.remove(0, topic.length() - 1);
    byte i = topic.toInt();
    word m = message.toInt();

    // cap range
    lidarRange[i] = m < outOfRange ? m : outOfRange;
    // see: https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms/
    lidarValue[i] = round( pow(2.0, (float)(outOfRange - lidarRange[i]) / R) - 1.0 );
    // average
    const byte lidarSmoothing = 3;
    lidarAvgValue[i] = (lidarAvgValue[i] * (lidarSmoothing - 1) + lidarValue[i]) / lidarSmoothing;
    // set LED brightness by value
    //leds[1+i] = CHSV(HUE_BLUE, 255, 2*lidarAvgValue[i]);
    leds[1 + i] = blend(leds[1 + i], CHSV(HUE_BLUE, 255, lidarAvgValue[i]), (fract8)128);
    /*
      if (i==3) Serial << "Got lidar data: " << m << " oor: " << outOfRange << " val: " << lidarValue[i] << endl;
    */
    //    if( i==0 ) Serial << "range[" << i << "]=" << range[i] << "\tvalue[" << i << "]=" << value[i];
/*
  } else if ( topic.startsWith("skein/range/1") ) {  // sharp

    // take the last character of the topic as the range index
    topic.remove(0, topic.length() - 1);
    byte i = topic.toInt();
    word m = message.toInt();

    // cap range
    sharpRange[i] = m < outOfRange ? m : outOfRange;
    // see: https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms/
    sharpValue[i] = round( pow(2.0, (float)(outOfRange - sharpRange[i]) / R) - 1.0 );
    //if (i==3) Serial << "Got sharp data: " << m << " oor: " << outOfRange << " val: " << sharpValue[i] << endl;

    // average
    const byte sharpSmoothing = 3;
    sharpAvgValue[i] = (sharpAvgValue[i] * (sharpSmoothing - 1) + sharpValue[i]) / sharpSmoothing;
    // set LED brightness by value
    //leds[1+i] = CHSV(HUE_GREEN, 255, 2*sharpAvgValue[i]);
    leds[1 + i] = blend(leds[1 + i], CHSV(HUE_GREEN, 255, sharpAvgValue[i]), (fract8)128);

    //    if( i==0 ) Serial << "range[" << i << "]=" << range[i] << "\tvalue[" << i << "]=" << value[i];
  }
  else {
    Serial << F("WARNING. unknown topic. continuing.");
  }

  //Serial << endl;
}

*/

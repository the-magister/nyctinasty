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
#include "Skein_Comms.h"

// really need to save this to EEPROM
// should we ever need to extend this to more uCs, this will generate an offset.
const byte subsetIndex = 0;

const byte Nsensor = 8;
word range[Nsensor];
word outOfRange = (1 << 11) - 1; // coresponds to a reading that's out-of-range

// lighting
// How many leds are in the strip?
const byte NUM_LEDS = 1 + Nsensor;
// Data pin that led data will be written out over
#define DATA_PIN 12 // GPIO12/D6.
CRGBArray<NUM_LEDS> leds;
const unsigned long targetFPS = 20;

byte value[Nsensor];
unsigned long avgValue[Nsensor];
// linearize perception to value
float R = (float)(outOfRange) / log2(255.0 + 1.0);

// connect to the MQTT network with this id
String id = "skeinLight" + subsetIndex;

// subscribe and process these topics
String ranges = "skein/range/" + String(subsetIndex, 10) + "/#";
String oor = "skein/range/oor";

void setup(void)  {
  Serial.begin(115200);
  delay(20);
  Serial << endl << endl << "Startup." << endl;

  FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, NUM_LEDS);

  commsBegin(id);
  commsSubscribe(ranges);
  commsSubscribe(oor);
}

void loop(void) {
  // comms handling
  commsUpdate();

  // lights handling
  if ( commsConnected() ) {
    for ( byte i = 0; i < Nsensor; i++ ) {
      Serial << avgValue[i] << ",";
    }
    Serial << 255 << endl;
    // do stuff with FastLED to map range[] to lights
  }

  // DON'T hammer the LEDs.  the clockless protocol interferes with
  // WiFi interrupt handling.
  static Metro ledUpdate(1000UL / targetFPS);
  if ( ledUpdate.check() ) {
    FastLED.show();
    ledUpdate.reset();
  }
}

void commsProcess(String topic, String message) {

  //  Serial << "<- " << topic << " " << message << "\t=> ";

  if ( topic.equals(oor) ) {

    outOfRange = message.toInt();
    R = (float)(outOfRange) / log2(255.0 + 1.0);
    Serial << "oor=" << outOfRange << "\tR=" << R;
  } else if ( topic.startsWith("skein/range") ) {
    // take the last character of the topic as the range index
    topic.remove(0, topic.length() - 1);
    byte i = topic.toInt();
    word m = message.toInt();

    // cap range
    range[i] = m < outOfRange ? m : outOfRange;
    // see: https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms/
    value[i] = round( pow(2.0, (float)(outOfRange - range[i]) / R) - 1.0 );
    // average
    const byte smoothing = 3;
    avgValue[i] = (avgValue[i] * (smoothing - 1) + value[i]) / smoothing;
    // set LED brightness by value
    leds[1+i] = CHSV(HUE_BLUE, 0, avgValue[i]);

    //    if( i==0 ) Serial << "range[" << i << "]=" << range[i] << "\tvalue[" << i << "]=" << value[i];
  } else {
    Serial << F("WARNING. unknown topic. continuing.");
  }

  Serial << endl;
}



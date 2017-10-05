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
word lidarRange[Nsensor];
word sharpRange[Nsensor];
word outOfRange = (1 << 11) - 1; // coresponds to a reading that's out-of-range

// lighting
// How many leds are in the strip?
const byte NUM_LEDS = 1 + Nsensor;
// Data pin that led data will be written out over
#define DATA_PIN 12 // GPIO12/D6.
CRGBArray<NUM_LEDS> leds;
const unsigned long targetFPS = 20;

byte lidarValue[Nsensor];
unsigned long lidarAvgValue[Nsensor];
byte sharpValue[Nsensor];
unsigned long sharpAvgValue[Nsensor];
// linearize perception to value
float R = (float)(outOfRange) / log2(255.0 + 1.0);

// connect to the MQTT network with this id
String id = "skeinLight" + subsetIndex;

// subscribe and process these topics
//String ranges = "skein/range/" + String(subsetIndex, 10) + "/#";
String ranges = "skein/range/#";   //    skein/range/0/0-7   lidar   skein/range/1/0-3 sharp
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

    for ( byte i = 0; i < Nsensor; i++ ) {
      lidarValue[i] = 0;
      lidarAvgValue[i] = 0;
      sharpValue[i] = 0;
      sharpAvgValue[i] = 0;
    }

  // lights handling
  if ( commsConnected() ) {
    /*
    for ( byte i = 0; i < Nsensor; i++ ) {
      Serial << lidarAvgValue[i] << ",";
    }
    Serial << 255 << endl;
    */
    /*
    for ( byte i = 0; i < Nsensor/2; i++ ) {
      Serial << sharpAvgValue[i] << ",";
    }
    Serial << 255 << endl;
    */
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
    leds[1+i] = blend(leds[1+i],CHSV(HUE_BLUE, 255, lidarAvgValue[i]),(fract8)128);
    /*
    if (i==3) Serial << "Got lidar data: " << m << " oor: " << outOfRange << " val: " << lidarValue[i] << endl;
*/
    //    if( i==0 ) Serial << "range[" << i << "]=" << range[i] << "\tvalue[" << i << "]=" << value[i];
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
    leds[1+i] = blend(leds[1+i],CHSV(HUE_GREEN, 255, sharpAvgValue[i]),(fract8)128);

    //    if( i==0 ) Serial << "range[" << i << "]=" << range[i] << "\tvalue[" << i << "]=" << value[i];
  }  
  else {
    Serial << F("WARNING. unknown topic. continuing.");
  }

  //Serial << endl;
}



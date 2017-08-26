/*
   This module is responsible for running lighting based on the Lidar sensor

   Subscribes: skein/range/i/#
               skein/range/oor
*/

// You'll need to add http://arduino.esp8266.com/stable/package_esp8266com_index.json to the Additional Board Managers URL entry in Preferences.
// Compile for NodeMCU 1.0 (ESP-12E Module), 80 Mhz, 921600 Upload Speed, 4M (3M SPIFFS).
#include <Streaming.h>
#include <Metro.h>
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
byte value[Nsensor];
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

  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED pin as an output

  commsBegin(id);
  commsSubscribe(ranges);
  commsSubscribe(oor);
}

void loop(void) {
  // comms handling
  commsUpdate();

  // lights handling

  // do stuff with FastLED to map range[] to lights
}

void commsProcess(String topic, String message) {

  Serial << "<- " << topic << " " << message << "\t=> ";

  if ( topic.equals(oor) ) {

    outOfRange = message.toInt();
    R = (float)(outOfRange) / log2(255.0 + 1.0);
    Serial << "oor=" << outOfRange << "\tR=" << R;
  } else if ( topic.startsWith("skein/range") ) {
    // take the last character of the topic as the range index
    topic.remove(0, topic.length()-1);
    byte i = topic.toInt();
    word m = message.toInt();

    range[i] = m;
    // see: https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms/
    value[i] = round( pow(2.0, (float)range[i] / R) - 1.0 );

    Serial << "range[" << i << "]=" << range[i] << "\tvalue[" << i << "]=" << value[i];
  } else {
    Serial << F("WARNING. unknown topic. continuing.");
  }
  
  Serial << endl;
}



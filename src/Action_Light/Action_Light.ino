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
String id = "skeinLight" + String(subsetIndex, 10);

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
//    Serial << F("Lidar") << endl;
//    float R = (float)(lidar.max) / log2(255.0 + 1.0);
    for (byte i = 0; i < N_SENSOR; i++) {
//      byte value = round( pow(2.0, (float)(lidar.max - lidar.dist[i]) / R) - 1.0 );
      byte value = distanceToBrightness( lidar.dist[i], lidar.min, lidar.max );
//      leds[i + 1] = blend(leds[i + 1], CHSV(HUE_BLUE, 255, value), (fract8)128);
      leds[i + 1] = CRGB(leds[i + 1].red, leds[i + 1].green, value);
    }
  }

  // sharp handling
  if ( sharpUpdate ) {
    sharpUpdate = false;
   // Serial << F("Sharp") << endl;
//    float R = (float)(sharp.max) / log2(255.0 + 1.0);
    for (byte i = 0; i < N_SENSOR; i++) {
//      byte value = round( pow(2.0, (float)(sharp.max - sharp.dist[i]) / R) - 1.0 );
      byte value = distanceToBrightness( sharp.dist[i], sharp.min, sharp.max);
      if( i == 3 ) {
        Serial << sharp.dist[i] << "," << sharp.max << "," << sharp.noise << "," << sharp.min << "," << value << endl;
      }
//      leds[i + 1] = blend(leds[i + 1], CHSV(HUE_GREEN, 255, value), (fract8)128);
      leds[i + 1] = CRGB(leds[i + 1].red, value, leds[i + 1].blue);
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


// gamma correction table; a linear distance index into this array will correct for nonlinear perception
uint8_t const exp_gamma[256] =
{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3,
  4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 12, 12, 12, 13, 13, 14, 14, 14, 15, 15,
  16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 23, 23, 24, 24, 25, 26, 26, 27, 28, 28, 29, 30, 30, 31, 32, 32, 33,
  34, 35, 35, 36, 37, 38, 39, 39, 40, 41, 42, 43, 44, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
  61, 62, 63, 64, 65, 66, 67, 68, 70, 71, 72, 73, 74, 75, 77, 78, 79, 80, 82, 83, 84, 85, 87, 89, 91, 92, 93, 95, 96, 98,
  99, 100, 101, 102, 105, 106, 108, 109, 111, 112, 114, 115, 117, 118, 120, 121, 123, 125, 126, 128, 130, 131, 133,
  135, 136, 138, 140, 142, 143, 145, 147, 149, 151, 152, 154, 156, 158, 160, 162, 164, 165, 167, 169, 171, 173, 175,
  177, 179, 181, 183, 185, 187, 190, 192, 194, 196, 198, 200, 202, 204, 207, 209, 211, 213, 216, 218, 220, 222, 225,
  227, 229, 232, 234, 236, 239, 241, 244, 246, 249, 251, 253, 254, 255
};

uint8_t distanceToBrightness( uint16_t distance, uint16_t distanceMin, uint16_t distanceMax ) {

  // enforce constraint
  distance = constrain( distance, distanceMin, distanceMax );

  // map, noting we map distanceMin to brightMax, etc.
  uint16_t distByte = map( distance, distanceMin, distanceMax, (uint16_t)255, (uint16_t)0 );

  // linearize distance to perception via table lookup
  return( exp_gamma[(uint8_t)distByte] );
}

// was: byte value = round( pow(2.0, (float)(lidar.max - lidar.dist[i]) / R) - 1.0 );





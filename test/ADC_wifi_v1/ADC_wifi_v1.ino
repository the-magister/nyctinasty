/*
   This module is responsible for polling the LIDAR sensors and publishing that data.
   We can have multiple sensors, and this is the i-th sensor.

   Publishes: skein/range/i/[0-7]: distance information, mm
              skein/range/i/oor: distance corresponding to out-of-range, mm
*/

// You'll need to add http://arduino.esp8266.com/stable/package_esp8266com_index.json to the Additional Board Managers URL entry in Preferences.
// Compile for Adafruit HUZZAH Feather
#include <Streaming.h>
#include <Metro.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include "Skein_Messages.h"
#include <SoftEasyTransfer.h>
#define FASTLED_ESP8266_RAW_PIN_ORDER
#include <FastLED.h>

// pin definitions
#define RED_LED 0
#define BLU_LED 2
#define RX 12
#define TX 14
#define L1 4
#define L2 5

SoftwareSerial mySerial(RX, TX, false, 256);
SoftEasyTransfer ETin, ETout;

// store readings
SensorReading readings;

// store settings
Command settings;

const byte NUM_LEDS = 12;
CRGBArray<NUM_LEDS> light0;
CRGBArray<NUM_LEDS> light1;

void setup(void)  {
  Serial.begin(115200);
  delay(10);
  Serial << endl << endl << "Startup." << endl;

  pinMode(BLU_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  
  // for remote output
  mySerial.begin(57600);
  while( !mySerial ) delay(5);
  
  // messages
  ETin.begin(details(readings), &mySerial);
  ETout.begin(details(settings), &mySerial);

  // load settings
//  EEPROM.begin(512); // required for ESPers
//  settings = loadCommand();
  settings.fps = defaultFPS;
//  saveCommand(settings);
//  EEPROM.commit();
  
  Serial << "Startup. fps=" << settings.fps << endl;
//  ETout.sendData(); // send target FPS

  delay(1000);
  
  FastLED.addLeds<WS2811, L1, RGB>(light0, NUM_LEDS);
  FastLED.addLeds<WS2811, L2, RGB>(light1, NUM_LEDS);
  FastLED.setMaxRefreshRate(settings.fps);

  light0(0, NUM_LEDS - 1) = CRGB::White;
  light1(0, NUM_LEDS - 1) = CRGB::White;
  FastLED.show();
  delay(500);
  light0(0, NUM_LEDS - 1) = CRGB::Black;
  light1(0, NUM_LEDS - 1) = CRGB::Black;
  FastLED.show();

  Serial << "Startup complete." << endl;
}

void loop(void) {
  // can't use FastLED's .show() routine, as that has a blocking loop/wait
  static Metro showInterval(1000UL / settings.fps);
  if ( showInterval.check() ) {
    FastLED.show();
    showInterval.reset();
  }


  // check for data
  static boolean LEDred = false;
  if ( ETin.receiveData() ) {
    LEDred = !LEDred;
    digitalWrite(RED_LED, LEDred);

    setLEDs();
    FastLED.show();
    showInterval.reset();
  }

}

void setLEDs() {
  // shift the last frame down the structure
  light0 >>= 4;
  light1 >>= 4;
  
  // set color, maximal
  light0(0, 3) = CRGB::Green;
  light1(0, 3) = CRGB::Blue;

  // scale readings
  uint16_t scaled[N_SENSOR] = {0};
  for(byte i=0; i<N_SENSOR; i++ ) {
    scaled[i] = map(readings.dist[0], readings.min, readings.max, (uint16_t)255, (uint16_t)0);
  }

  // dim/fade lighting by scaled distance
  light0[0] %= scaled[3];
  light0[1] %= scaled[2];
  light0[2] %= scaled[1];
  light0[3] %= scaled[0];
  light1[0] %= scaled[4];
  light1[1] %= scaled[5];
  light1[2] %= scaled[6];
  light1[3] %= scaled[7];
}


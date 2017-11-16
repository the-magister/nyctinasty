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

const byte NUM_LEDS = 12;
CRGBArray<NUM_LEDS> light0;
CRGBArray<NUM_LEDS> light1;

void setup(void)  {
  Serial.begin(115200);
  delay(10);
  Serial << endl << endl << "Startup." << endl;

  // for remote output
  mySerial.begin(57600);

  FastLED.addLeds<WS2811, L1, RGB>(light0, NUM_LEDS);
  FastLED.addLeds<WS2811, L2, RGB>(light1, NUM_LEDS);

  light0(0,NUM_LEDS-1) = CRGB(255,0,0);
  FastLED.show();
}

void loop(void) {
  static boolean LEDstate = false;
  if( mySerial.available() ) {
    int value = mySerial.parseInt();
    int scaled = map(value, 0, 1023, 1, 255);
    Serial << value << "," << scaled << endl;
    LEDstate = !LEDstate;
    digitalWrite(BLU_LED, LEDstate);
    
    light0(0,NUM_LEDS-1) = CHSV(HUE_GREEN, 128, scaled);
    light1(0,NUM_LEDS-1) = CHSV(HUE_BLUE, 128, scaled);
    FastLED.show();
  }
}


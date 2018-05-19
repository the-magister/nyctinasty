// IDE Settings:
// Tools->Board : "WeMos D1 R2 & mini"
// Tools->Flash Size : "4M (3M SPIFFS)"
// Tools->CPU Frequency : "160 MHz"
// Tools->Upload Speed : "921600"

#define SHOW_SERIAL_DEBUG true

#include <Streaming.h>
#include <Metro.h>
#include <SoftwareSerial.h>
#include <SoftEasyTransfer.h>
#include "Nyctinasty_Messages.h"
#include "Nyctinasty_Comms.h"

#define FASTLED_INTERRUPT_RETRY_COUNT 1
#include <FastLED.h>

// wire it up
// RX/TX for softserial comms
#define RX D1 // GPIO5
#define TX D2 // GPIO4

// LEDs are connected:
// D5, GPIO14
// D6, GPIO12
// D7, GPIO13
// D8, GPIO15

// also used
// D4, GPIO2, BUILTIN_LED

// our internal storage, mapped to the hardware.
// pay no attention to the man behind the curtain.
#define COLOR_ORDER RGB
#define COLOR_CORRECTION TypicalLEDStrip
#define NUM_PINS 4
#define LEDS_BAR 4
#define LEDS_VERT 20
#define LEDS_PER_PIN LEDS_BAR+LEDS_VERT

CRGBArray<LEDS_PER_PIN> leftBack;
CRGBArray<LEDS_PER_PIN> rightBack;
CRGBArray<LEDS_PER_PIN> leftFront;
CRGBArray<LEDS_PER_PIN> rightFront;

// verticals
CRGBSet leftUp = leftFront(LEDS_BAR, LEDS_PER_PIN-1);
CRGBSet rightUp = rightFront(LEDS_BAR, LEDS_PER_PIN-1);
CRGBSet leftDown = leftBack(LEDS_BAR, LEDS_PER_PIN-1);
CRGBSet rightDown = rightBack(LEDS_BAR, LEDS_PER_PIN-1);

// color choices, based on arch and sepal information
byte archHue[N_ARCH] = {HUE_RED, HUE_GREEN, HUE_BLUE};
byte archSat[N_ARCH] = {128, 128, 128};

// our distance updates send as this structure as this topic
SepalArchDistance dist;

// talk to the ADC device
SoftwareSerial mySerial(RX, TX, false, 1024);
SoftEasyTransfer ETin;

void setup() {
  // for local output
  Serial.begin(115200);

  Serial << endl << endl << endl << F("Startup begins.") << endl;


  // for remote output
  mySerial.begin(115200);

  // messages
  ETin.begin(details(dist), &mySerial);

  // after set up the input pin
  pinMode(TX, OUTPUT); // trigger to send

  // LEDs
  Serial << F("Configure leds...");
  FastLED.addLeds<WS2811, D5, COLOR_ORDER>(leftBack, LEDS_PER_PIN).setCorrection(COLOR_CORRECTION);
  FastLED.addLeds<WS2811, D6, COLOR_ORDER>(rightBack, LEDS_PER_PIN).setCorrection(COLOR_CORRECTION);
  FastLED.addLeds<WS2811, D7, COLOR_ORDER>(leftFront, LEDS_PER_PIN).setCorrection(COLOR_CORRECTION);
  FastLED.addLeds<WS2811, D8, COLOR_ORDER>(rightFront, LEDS_PER_PIN).setCorrection(COLOR_CORRECTION);
  FastLED.setBrightness(255);
  runStartupPattern();
  Serial << F(" done.") << endl;

  Serial << F("Startup complete.") << endl;
}


void askForDistance() {
  // toggled TX pin
  static boolean pinState = false;
  static Metro distanceUpdate(10UL);

  if ( distanceUpdate.check() ) {
    distanceUpdate.reset();
    pinState = !pinState;
    digitalWrite(TX, pinState);
  }
}

void loop() {
  // check to see if we need to pull new distance data.
  askForDistance();

  // get data from ADCs
  static uint32_t counter = 0;
  if ( ETin.receiveData() && SHOW_SERIAL_DEBUG ) {
    counter ++;
/*
    const char sep[] = ",";

    for ( byte i = 0; i < N_SENSOR; i++ ) Serial << dist.prox[i] << sep;

    Serial << dist.min << sep;
    Serial << dist.noise << sep;
    Serial << dist.max << sep;
    //    Serial << smoothing << sep;
    Serial << endl;
*/

    idle();
  }

  EVERY_N_SECONDS( 1 ) {
    Serial << counter << endl;
    counter=0;
  }

}

void runStartupPattern() {
  // find the pins
  leftBack.fill_solid(CRGB::Purple);
  rightBack.fill_solid(CRGB::Aqua);
  leftFront.fill_solid(CRGB::Red);
  rightFront.fill_solid(CRGB::Blue);
  FastLED.show();
  delay(1000);

  leftBack.fill_solid(CRGB::Black);
  rightBack.fill_solid(CRGB::Black);
  leftFront.fill_solid(CRGB::Black);
  rightFront.fill_solid(CRGB::Black);
  FastLED.show();
  delay(333);
}

void idle() {
  static byte hue = 0;

//  EVERY_N_MILLISECONDS(50) {
    // show a throbbing rainbow background
    hue++;
    leftBack.fill_rainbow(hue, 255 / leftBack.size()); // paint
    rightBack = leftBack;
    leftFront.fill_rainbow(hue + 128, -255 / leftFront.size());
    rightFront = leftFront;
    FastLED.show();
//  }
}


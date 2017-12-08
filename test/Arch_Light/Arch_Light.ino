// Use if you want to force the software SPI subsystem to be used for some reason (generally, you don't)
// #define FASTLED_FORCE_SOFTWARE_SPI
// Use if you want to force non-accelerated pin access (hint: you really don't, it breaks lots of things)
// #define FASTLED_FORCE_SOFTWARE_SPI
// #define FASTLED_FORCE_SOFTWARE_PINS
#define FASTLED_ESP8266_RAW_PIN_ORDER
//#define FASTLED_ESP8266_D1_PIN_ORDER
// #define FASTLED_ALLOW_INTERRUPTS 0
#define FASTLED_INTERRUPT_RETRY_COUNT 1

#include "FastLED.h"

///////////////////////////////////////////////////////////////////////////////////////////
//
// Move a white dot along the strip of leds.  This program simply shows how to configure the leds,
// and then how to turn a single pixel white and then off, moving down the line of pixels.
//

// How many leds are in the strip?
#define NUM_LEDS 20
#define DATA_PIN1 D5
#define DATA_PIN2 D6
#define DATA_PIN3 D7
#define DATA_PIN4 D8

CRGB leds1[NUM_LEDS];
CRGB leds2[NUM_LEDS];
CRGB leds3[NUM_LEDS];
CRGB leds4[NUM_LEDS];

#define COLOR_ORDER RGB
#define COLOR_CORRECTION TypicalLEDStrip


// This function sets up the ledsand tells the controller about them
void setup() {
  // sanity check delay - allows reprogramming if accidently blowing power w/leds
  delay(2000);

  FastLED.addLeds<WS2811, DATA_PIN1, RGB>(leds1, NUM_LEDS).setCorrection(COLOR_CORRECTION);
  FastLED.addLeds<WS2811, DATA_PIN2, RGB>(leds2, NUM_LEDS).setCorrection(COLOR_CORRECTION);
  FastLED.addLeds<WS2811, DATA_PIN3, RGB>(leds3, NUM_LEDS).setCorrection(COLOR_CORRECTION);
  FastLED.addLeds<WS2811, DATA_PIN4, RGB>(leds4, NUM_LEDS).setCorrection(COLOR_CORRECTION);

  fill_solid(leds1, NUM_LEDS, CRGB::Red);
  fill_solid(leds2, NUM_LEDS, CRGB::Red);
  fill_solid(leds3, NUM_LEDS, CRGB::Red);
  fill_solid(leds4, NUM_LEDS, CRGB::Red);
  FastLED.show();
  delay(1000);

  fill_solid(leds1, NUM_LEDS, CRGB::Green);
  fill_solid(leds2, NUM_LEDS, CRGB::Green);
  fill_solid(leds3, NUM_LEDS, CRGB::Green);
  fill_solid(leds4, NUM_LEDS, CRGB::Green);
  FastLED.show();
  delay(1000);

  fill_solid(leds1, NUM_LEDS, CRGB::Blue);
  fill_solid(leds2, NUM_LEDS, CRGB::Blue);
  fill_solid(leds3, NUM_LEDS, CRGB::Blue);
  fill_solid(leds4, NUM_LEDS, CRGB::Blue);
  FastLED.show();
  delay(1000);

  FastLED.clear();
  FastLED.show();

  pinMode(BUILTIN_LED, OUTPUT);
}

// This function runs over and over, and is where you do the magic to light
// your leds.
void loop() {
  static boolean ledState = false;

  fadeToBlackBy( leds1, NUM_LEDS, 16);
  fadeToBlackBy( leds2, NUM_LEDS, 16);
  fadeToBlackBy( leds3, NUM_LEDS, 16);
  fadeToBlackBy( leds4, NUM_LEDS, 16);

  // Move a single white led
  static int bWhite = 0;
  bWhite++;
  if ( bWhite >= NUM_LEDS ) bWhite = 0;

  leds1[bWhite] = CRGB::White;
  leds2[bWhite] = CRGB::White;
  leds3[bWhite] = CRGB::White;
  leds4[bWhite] = CRGB::White;

  // Show the leds (only one of which is set to white, from above)
  FastLED.show();

  // Wait a little bit
  delay(100);

  ledState = !ledState;
  digitalWrite(BUILTIN_LED, ledState);

}

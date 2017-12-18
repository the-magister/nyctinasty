// Compile for Wemos D1 R2 & Mini
#include <Metro.h>
#include <Streaming.h>
// pin order
#define FASTLED_ESP8266_RAW_PIN_ORDER
// with retries on strip updates
#define FASTLED_INTERRUPT_RETRY_COUNT 1
#include <FastLED.h>
#include "Nyctinasty_Messages.h"

// our internal storage, mapped to the hardware.
// pay no attention to the man behind the curtain.
#define NUM_PINS 4
#define LEDS_PER_PIN (N_LEDS+N_SENSOR/2)
CRGB leds[NUM_PINS * LEDS_PER_PIN];
CRGBSet ledsLeftDown(&leds[0 * LEDS_PER_PIN], LEDS_PER_PIN);
CRGBSet ledsLeftUp(&leds[1 * LEDS_PER_PIN], LEDS_PER_PIN);
CRGBSet ledsRightDown(&leds[2 * LEDS_PER_PIN], LEDS_PER_PIN);
CRGBSet ledsRightUp(&leds[3 * LEDS_PER_PIN], LEDS_PER_PIN);

#define COLOR_ORDER RGB
#define COLOR_CORRECTION TypicalLEDStrip

void setup() {
  Serial.begin(115200);

  FastLED.addLeds<WS2811_PORTA, NUM_PINS, RGB>(leds, LEDS_PER_PIN).setCorrection(COLOR_CORRECTION);
  FastLED.setBrightness(255);

  runStartupPattern();

}

void loop() {
  EVERY_N_MILLISECONDS( 10 ) {
    pushToHardware();
  }
}

/*
  void applyToHardware() {
  // arch bar update
  CRGBSet bar(lights.bar, N_SENSOR);
  const uint16_t halfBar = bar.size() / 2 - 1;
  const uint16_t fullBar = bar.size() - 1;
  ledsLeftDown(halfBar, 0) = bar(0, halfBar);
  ledsRightDown(0, halfBar) = bar(halfBar + 1, fullBar);
  // mirror bar
  ledsLeftUp(0, halfBar) = ledsLeftDown(0, halfBar);
  ledsRightUp(0, halfBar) = ledsRightDown(0, halfBar);

  // down lights update
  CRGBSet leftDown(lights.leftDown, N_LEDS);
  CRGBSet rightDown(lights.rightDown, N_LEDS);
  const uint16_t startDown = halfBar + 1;
  const uint16_t endDown = ledsLeftDown.size() - 1;
  ledsLeftDown(startDown, endDown) = leftDown;
  ledsRightDown(startDown, endDown) = rightDown;

  // up lights update
  CRGBSet leftUp(lights.leftUp, N_LEDS);
  CRGBSet rightUp(lights.rightUp, N_LEDS);
  const uint16_t startUp = halfBar + 1;
  const uint16_t endUp = ledsLeftUp.size() - 1;
  ledsLeftUp(startUp, endUp) = leftUp;
  ledsRightUp(startUp, endUp) = rightUp;

  }
*/

void pushToHardware() {
  FastLED.show();

  // show our frame rate, periodically
  EVERY_N_SECONDS( 5 ) {
    float reportedFPS = FastLED.getFPS();
    Serial << F("FPS reported (Hz): ") << reportedFPS << endl;
  }
}

void runStartupPattern() {
  // find the pins
  ledsLeftDown.fill_solid(CRGB::Red);
  FastLED.show();
  delay(333);

  ledsLeftUp.fill_solid(CRGB::Blue);
  FastLED.show();
  delay(333);

  ledsRightDown.fill_solid(CRGB::Green);
  FastLED.show();
  delay(333);

  ledsRightUp.fill_solid(CRGB::Purple);
  FastLED.show();
  delay(333);

  ledsLeftDown.fill_solid(CRGB::Black);
  ledsRightDown.fill_solid(CRGB::Black);
  ledsLeftUp.fill_solid(CRGB::Black);
  ledsRightUp.fill_solid(CRGB::Black);
  FastLED.show();
  delay(333);

  // find the bar
  CRGBArray<N_SENSOR> bar;
  bar.fill_solid(CRGB::Black);
  bar[0] = CRGB::Green;
  bar[N_SENSOR-1] = CRGB::Red;
  const uint16_t halfBar = bar.size() / 2 - 1;
  const uint16_t fullBar = bar.size() - 1;
  ledsLeftDown(halfBar, 0) = bar(0, halfBar);
  ledsRightDown(0, halfBar) = bar(halfBar + 1, fullBar);
  // mirror bar
  ledsLeftUp(0, halfBar) = ledsLeftDown(0, halfBar);
  ledsRightUp(0, halfBar) = ledsRightDown(0, halfBar);
  FastLED.show();
  delay(333);

  // down lights update
  CRGBArray<N_LEDS> leftDown;
  CRGBArray<N_LEDS> rightDown;
  leftDown.fill_solid(CRGB::Black);
  rightDown.fill_solid(CRGB::Black);
  leftDown[0] = CRGB::Green;
  rightDown[0] = CRGB::Green;
  leftDown[N_LEDS-1] = CRGB::Red;
  rightDown[N_LEDS-1] = CRGB::Red;
  const uint16_t startDown = halfBar + 1;
  const uint16_t endDown = ledsLeftDown.size() - 1;
  ledsLeftDown(startDown, endDown) = leftDown;
  ledsRightDown(startDown, endDown) = rightDown;
  FastLED.show();
  delay(333);

  // up lights update
  CRGBArray<N_LEDS> leftUp;
  CRGBArray<N_LEDS> rightUp;
  leftUp.fill_solid(CRGB::Black);
  rightUp.fill_solid(CRGB::Black);
  leftUp[0] = CRGB::Green;
  rightUp[0] = CRGB::Green;
  leftUp[N_LEDS-1] = CRGB::Red;
  rightUp[N_LEDS-1] = CRGB::Red;
  const uint16_t startUp = halfBar + 1;
  const uint16_t endUp = ledsLeftUp.size() - 1;
  ledsLeftUp(startUp, endUp) = leftUp;
  ledsRightUp(startUp, endUp) = rightUp;
  FastLED.show();
  delay(333);

}




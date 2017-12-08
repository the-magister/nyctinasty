// Compile for Wemos D1 R2 & Mini
#define FASTLED_ESP8266_RAW_PIN_ORDER
#define FASTLED_INTERRUPT_RETRY_COUNT 1
#include <FastLED.h>
#include <Nyctinasty_Messages.h>
#include <Metro.h>
#include <Streaming.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Nyctinasty_Comms.h>

// who am I?
const byte seepleNumber = 0;
const byte archNumber = 0;
const String id = commsIdSeepleArchLight(seepleNumber, archNumber);

// ship settings
SystemCommand settings;
// in this topic
const String settingsTopic = commsTopicSystemCommand();
// and sets this true when an update arrives
boolean settingsUpdate = false;

// our led updates come as this structure
SeepleArchLight lights;
// in this topic
const String lightsTopic = commsTopicLight(seepleNumber, archNumber);
// and sets this true when an update arrives
boolean lightsUpdate = false;

// pins to data lines
#define DATA_PIN_LeftDown D5
#define DATA_PIN_RightDown D6
#define DATA_PIN_RightUp D7
#define DATA_PIN_LeftUp D8

// our internal storage, mapped to the hardware.  
// pay no attention to the man behind the curtain.
// could this be done with pointers to save memory and possibly improve speed?
// yes. knock yourself out.
CRGBArray < N_LEDS_DOWN + N_SENSOR / 2 > ledsLeftDown;
CRGBArray < N_LEDS_DOWN + N_SENSOR / 2 > ledsRightDown;
CRGBArray < N_LEDS_UP + N_SENSOR / 2 > ledsLeftUp;
CRGBArray < N_LEDS_UP + N_SENSOR / 2 > ledsRightUp;

#define COLOR_ORDER RGB
#define COLOR_CORRECTION TypicalLEDStrip

// This function sets up the ledsand tells the controller about them
void setup() {
  Serial.begin(115200);

  commsBegin(id);
  commsSubscribe(settingsTopic, &settings, &settingsUpdate);
  commsSubscribe(lightsTopic, &lights, &lightsUpdate);
  commsUpdate();

  FastLED.addLeds<WS2811, DATA_PIN_LeftDown, RGB>(ledsLeftDown, ledsLeftDown.size()).setCorrection(COLOR_CORRECTION);
  FastLED.addLeds<WS2811, DATA_PIN_RightDown, RGB>(ledsRightDown, ledsRightDown.size()).setCorrection(COLOR_CORRECTION);
  FastLED.addLeds<WS2811, DATA_PIN_LeftUp, RGB>(ledsLeftUp, ledsLeftUp.size()).setCorrection(COLOR_CORRECTION);
  FastLED.addLeds<WS2811, DATA_PIN_RightUp, RGB>(ledsRightUp, ledsRightUp.size()).setCorrection(COLOR_CORRECTION);

  ledsLeftDown.fill_solid(CRGB::Red);
  ledsRightDown.fill_solid(CRGB::Red);
  ledsLeftUp.fill_solid(CRGB::Red);
  ledsRightUp.fill_solid(CRGB::Red);
  FastLED.show();
  delay(333);

  ledsLeftDown.fill_solid(CRGB::Green);
  ledsRightDown.fill_solid(CRGB::Green);
  ledsLeftUp.fill_solid(CRGB::Green);
  ledsRightUp.fill_solid(CRGB::Green);
  FastLED.show();
  delay(333);

  ledsLeftDown.fill_solid(CRGB::Blue);
  ledsRightDown.fill_solid(CRGB::Blue);
  ledsLeftUp.fill_solid(CRGB::Blue);
  ledsRightUp.fill_solid(CRGB::Blue);
  FastLED.show();
  delay(333);

  FastLED.clear();
  FastLED.show();
}

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
  CRGBSet leftDown(lights.leftDown, N_LEDS_DOWN);  
  CRGBSet rightDown(lights.rightDown, N_LEDS_DOWN);  
  const uint16_t startDown = halfBar + 1;
  const uint16_t endDown = ledsLeftDown.size() - 1;
  ledsLeftDown(startDown, endDown) = leftDown;
  ledsRightDown(startDown, endDown) = rightDown;

  // up lights update
  CRGBSet leftUp(lights.leftUp, N_LEDS_UP);  
  CRGBSet rightUp(lights.rightUp, N_LEDS_UP);  
  const uint16_t startUp = halfBar + 1;
  const uint16_t endUp = ledsLeftUp.size() - 1;
  ledsLeftUp(startUp, endUp) = leftUp;
  ledsRightUp(startUp, endUp) = rightUp;

}

void pushToHardware() {
  FastLED.show();

  // show our frame rate, periodically
  EVERY_N_SECONDS( 5 ) {
    float reportedFPS = FastLED.getFPS();
    Serial << F("FPS reported (Hz): ") << reportedFPS << endl;
  }
}

void loop() {
  // comms handling
  commsUpdate();

  // bail out if not connected
  if ( ! commsConnected() ) return;

  // check for settings update
  if( settingsUpdate ) {
    settingsUpdate = false;
  }
  
  // check for an update to lights
  if ( lightsUpdate ) {    
    // push the message array to the hardware.
    applyToHardware();

    // show
    pushToHardware();

    // reset
    lightsUpdate = false;
  }
}


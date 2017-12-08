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
const String function = "Light";
const String seepleNumber = String(1, 10);
const String archNumber = String(1, 10);

// connect to the MQTT network with this id
const String preface = "nyc";
const String nameSep = "/";
const String id = preface + nameSep + function + nameSep + seepleNumber + nameSep + archNumber;

// our led updates come as this structure
SeepleArchLight lights;
// in this topic
const String sep = "/";
const String lightsTopic = preface + sep + function + sep + seepleNumber + sep + archNumber;
// and sets this true when an update arrives
boolean lightsUpdate = false;

// set true to publish a test pattern to our own topic 
#define SIMULATE_RADIO_UPDATE true

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

void show() {
  FastLED.show();

  // show our frame rate, periodically
  EVERY_N_SECONDS( 5 ) {
    float reportedFPS = FastLED.getFPS();
    Serial << F("FPS reported (Hz): ") << reportedFPS << endl;
  }
}

void applyToHardware() {
  // arch bar update
  const uint16_t halfBar = lights.bar.size() / 2 - 1;
  const uint16_t fullBar = lights.bar.size() - 1;
  ledsLeftDown(halfBar, 0) = lights.bar(0, halfBar);
  ledsRightDown(0, halfBar) = lights.bar(halfBar + 1, fullBar);
  // mirror bar
  ledsLeftUp(0, halfBar) = ledsLeftDown(0, halfBar);
  ledsRightUp(0, halfBar) = ledsRightDown(0, halfBar);

  // down lights update
  const uint16_t startDown = halfBar + 1;
  const uint16_t endDown = ledsLeftDown.size() - 1;
  ledsLeftDown(startDown, endDown) = lights.leftDown;
  ledsRightDown(startDown, endDown) = lights.rightDown;

  // up lights update
  const uint16_t startUp = halfBar + 1;
  const uint16_t endUp = ledsLeftUp.size() - 1;
  ledsLeftUp(startUp, endUp) = lights.leftUp;
  ledsRightUp(startUp, endUp) = lights.rightUp;
}

void loop() {
  // comms handling
  commsUpdate();

  // bail out if not connected
  if ( ! commsConnected() ) return;

  // check for an update to lights
  if ( lightsUpdate ) {
    // push the message array to the hardware.
    applyToHardware();

    // show
    show();

    // reset
    lightsUpdate = false;
  }

  if( SIMULATE_RADIO_UPDATE ) {
    // simulate
    simulateRadioUpdate();
  }
}

// publishing this is the job of Seeple_Coordinator
void simulateRadioUpdate() {
  static Metro updateInterval(1000UL / (uint32_t)defaultFPS);
  //  static Metro updateInterval(100UL);
  if ( ! updateInterval.check() ) return;

  // adjust the bar
  EVERY_N_MILLISECONDS( 100 ) {
    lights.bar.fadeToBlackBy(128);
    static byte counter = 0;
    lights.bar[counter++ % lights.bar.size()] = CRGB::White;
  }

  // show a throbbing rainbow on the down segments
  EVERY_N_MILLISECONDS( 100 ) {
    static byte fadeBy = 0;
    static byte legHue = 0;
    static int legHueDelta = 255 / 8;
    static byte legBrightness = 32;
    lights.leftDown.fill_rainbow(++legHue, legHueDelta); // paint
    lights.leftDown.fadeToBlackBy(++fadeBy);
    lights.rightDown.fill_rainbow(legHue + 128, -legHueDelta); // paint, noting we're using the other side of the wheel
    lights.rightDown.fadeToBlackBy(fadeBy + 128);
  }

  // trails
  EVERY_N_MILLISECONDS( 10 ) {
    // bpm (rate of dot travel) changes with time
    const byte bpm = 16; // 1/minute
    // fade everything
    lights.leftUp.fadeToBlackBy(bpm / 4);
    lights.leftDown.fadeToBlackBy(bpm / 4);
    // set the speed the pixel travels, see: lib8tion.h
    const uint16_t endUp = lights.rightUp.size() - 1;
    uint16_t posVal = beatsin16(bpm, 0, (uint16_t)(endUp + 1));
    // cycle through hues
    static byte hue = 0;
    // paint
    lights.leftUp[posVal] = CHSV(++hue, 255, 255);
    // mirrored direction and hue
    lights.rightUp[endUp - posVal] = CHSV(hue + 128, 255, 255);
  }

  // ship it, but bail out if no connection
  if ( commsConnected() ) commsPublish(lightsTopic, &lights);

  // all done
  updateInterval.reset();
}



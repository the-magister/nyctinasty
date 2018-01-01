// IDE Settings:
// Tools->Board : "WeMos D1 R2 & mini"
// Tools->Flash Size : "4M (3M SPIFFS)"
// Tools->CPU Frequency : "160 MHz"
// Tools->Upload Speed : "921600"

#include <Metro.h>
#include <Streaming.h>
// pin order
#define FASTLED_ESP8266_RAW_PIN_ORDER
// will toss packets while updating lights.
#define FASTLED_ALLOW_INTERRUPTS 0
//#define FASTLED_INTERRUPT_RETRY_COUNT 1
// with retries on strip updates
#include <FastLED.h>
#include <FiniteStateMachine.h>
#include "Nyctinasty_Messages.h"
#include "Nyctinasty_Comms.h"

// wire it up
//#define DATA_PIN_LeftDown D5
//#define DATA_PIN_RightDown D6
//#define DATA_PIN_RightUp D7
//#define DATA_PIN_LeftUp D8

// comms
NyctComms comms;

// define a state for every systemState
void idle(); State Idle = State(idle);
void normal(); State Normal = State(normal);
void central(); State Central = State(central);
void reboot() { comms.reboot(); }; State Reboot = State(reboot);
void reprogram() { comms.reprogram("Sepal_Arch_Light.ino.bin"); } State Reprogram = State(reprogram);
FSM stateMachine = FSM(Idle); //initialize state machine

// my role
NyctRole role = Light;

// incoming message storage and flag for update
struct sC_t { boolean hasUpdate=false; SystemCommand settings; } sC;

// incoming message storage and flag for update
struct sAD_t { boolean hasUpdate=false; SepalArchDistance dist; } sAD;

// incoming message storage and flag for update
struct sAF_t { boolean hasUpdate=false; SepalArchFrequency freq; } sAF;

// color choices, based on arch and sepal information
byte archHue, archSat;

// our internal storage, mapped to the hardware.
// pay no attention to the man behind the curtain.
#define NUM_PINS 4
#define N_LEDS 16
#define LEDS_PER_PIN (N_LEDS+N_SENSOR/2)
CRGB leds[NUM_PINS * LEDS_PER_PIN];
CRGBSet ledsLeftDown(&leds[0 * LEDS_PER_PIN], LEDS_PER_PIN);
CRGBSet ledsLeftUp(&leds[1 * LEDS_PER_PIN], LEDS_PER_PIN);
CRGBSet ledsRightDown(&leds[2 * LEDS_PER_PIN], LEDS_PER_PIN);
CRGBSet ledsRightUp(&leds[3 * LEDS_PER_PIN], LEDS_PER_PIN);

// ease accessing portions of the hardware
CRGBSet leftDown(&ledsLeftDown[N_SENSOR / 2], N_LEDS);
CRGBSet leftUp(&ledsLeftUp[N_SENSOR / 2], N_LEDS);
CRGBSet rightDown(&ledsRightDown[N_SENSOR / 2], N_LEDS);
CRGBSet rightUp(&ledsRightUp[N_SENSOR / 2], N_LEDS);

#define COLOR_ORDER RGB
#define COLOR_CORRECTION TypicalLEDStrip

void setup() {
  Serial.begin(115200);

  Serial << endl << endl << endl << F("Startup.") << endl;

  // start comms
  comms.begin(role);

  // lighting choice
  archHue = (256/N_ARCH) * comms.getArch();
  archSat = 128;

  // subscribe
  comms.subscribe(&sC.settings, &sC.hasUpdate); 
  comms.subscribe(&sAD.dist, &sAD.hasUpdate); 
  comms.subscribe(&sAF.freq, &sAF.hasUpdate); 

  //  FastLED.addLeds<WS2811, DATA_PIN_LeftDown, RGB>(ledsLeftDown, ledsLeftDown.size()).setCorrection(COLOR_CORRECTION);
  //  FastLED.addLeds<WS2811, DATA_PIN_RightDown, RGB>(ledsRightDown, ledsRightDown.size()).setCorrection(COLOR_CORRECTION);
  //  FastLED.addLeds<WS2811, DATA_PIN_LeftUp, RGB>(ledsLeftUp, ledsLeftUp.size()).setCorrection(COLOR_CORRECTION);
  //  FastLED.addLeds<WS2811, DATA_PIN_RightUp, RGB>(ledsRightUp, ledsRightUp.size()).setCorrection(COLOR_CORRECTION);
  FastLED.addLeds<WS2811_PORTA, NUM_PINS, RGB>(leds, LEDS_PER_PIN).setCorrection(COLOR_CORRECTION);
  FastLED.setBrightness(255);

//  runStartupPattern();

  FastLED.clear();
  FastLED.show();
}

void loop() {
  // comms handling
  comms.update();

  // check for settings update
  if ( sC.hasUpdate ) {
    switchState(sC.settings.state);
    sC.hasUpdate = false;
  }

  // do stuff
  stateMachine.update();

  // run the lights
  EVERY_N_MILLISECONDS( 20 ) {
    pushToHardware();
  }

}

void switchState(systemState state) {
  Serial << F("State.  Changing to ") << state << endl;
  switch ( state ) {
    case IDLE: stateMachine.transitionTo(Idle); break;
    case NORMAL: stateMachine.transitionTo(Normal); break;
    case CENTRAL: stateMachine.transitionTo(Central); break;
    case REBOOT: stateMachine.transitionTo(Reboot); break;
    case REPROGRAM: stateMachine.transitionTo(Reprogram); break;
    default:
      Serial << F("ERROR!  unknown state.") << endl;
  }
}

void idle() {
  static byte hue = 0;

  EVERY_N_MILLISECONDS(20) {
    // show a throbbing rainbow on the legs
    hue++;
    leftDown.fill_rainbow(hue, 255 / leftDown.size()); // paint
    rightDown.fill_rainbow(hue + 128, -255 / rightDown.size());

    // trails on the ups
    const byte bpm = 16; // bpm (rate of dot travel), 1/minute
    leftUp.fadeToBlackBy(bpm);
    rightUp.fadeToBlackBy(bpm);
    // set the speed the pixel travels, see: lib8tion.h
    const uint16_t endUp = rightUp.size() - 1;
    uint16_t posVal = beatsin16(bpm, 0, (uint16_t)endUp);

    // paint
    leftUp[posVal] = CHSV(hue, 255, 255);
    // mirrored direction and hue
    rightUp[endUp - posVal] = CHSV(hue + 128, 255, 255);
  }

}

void normal() {
  // check for an update to distance
  if ( sAD.hasUpdate ) {
    // map distance to lights
    mapDistanceToBar();

    // reset
    sAD.hasUpdate = false;
  }

  // check for an update to frequency
  if ( sAF.hasUpdate ) {
    // map frequency to legs
    mapFreqToLegs();

    // reset
    sAF.hasUpdate = false;
  }
}

void central() {
  // probably want
  // https://github.com/ppelleti/esp-opc-server/blob/master/esp-opc-server.ino
}

void mapFreqToLegs() {
  uint16_t avgPower[N_SENSOR] = {0};
  // frequency power
  uint16_t power[N_SENSOR][N_FREQ_BINS] = {{0.0}};

  // do the boneheaded thing and sum up the bins across all sensors
  uint32_t sumSensors[N_FREQ_BINS] = {0};
  uint32_t maxSum = 0;
  for ( uint16_t j = 0; j < N_FREQ_BINS ; j++ ) {
    for ( byte i = 0; i < N_SENSOR; i++ ) {
      sumSensors[j] += sAF.freq.power[i][j];
    }
    if ( sumSensors[j] > maxSum ) maxSum = sumSensors[j];
  }

  Serial << F("Freq bins: ");
  // set the LEDs proportional to bins, normalized to maximum bin
  for ( uint16_t j = 0; j < N_FREQ_BINS ; j++ ) {
    uint32_t value = map( sumSensors[j],
                          (uint32_t)0, maxSum,
                          (uint32_t)0, (uint32_t)255
                        );
    Serial << value << ",";
    leftDown[j % N_LEDS] = CHSV(archHue, archSat, brighten8_video(constrain(value, 0, 255)));
  }
  Serial << endl;
  rightDown = leftDown;
}

void mapDistanceToBar() {
  CRGBArray<N_SENSOR> bar;
  const uint16_t halfBar = bar.size() / 2 - 1;
  const uint16_t fullBar = bar.size() - 1;

  for ( byte i = 0; i < N_SENSOR; i++ ) {
    uint16_t intensity = map(
                           sAD.dist.prox[i] > sAD.dist.noise ? sAD.dist.prox[i] : 0,
                           sAD.dist.min, sAD.dist.max,
                           (uint16_t)0, (uint16_t)255
                         );
    bar[i % N_SENSOR] = CHSV(archHue, archSat, (byte)intensity);
  }

  // arch bar update
  ledsLeftDown(halfBar, 0) = bar(0, halfBar);
  ledsRightDown(0, halfBar) = bar(halfBar + 1, fullBar);
  // mirror bar
  ledsLeftUp(0, halfBar) = ledsLeftDown(0, halfBar);
  ledsRightUp(0, halfBar) = ledsRightDown(0, halfBar);

}

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
  const uint16_t halfBar = bar.size() / 2 - 1;
  const uint16_t fullBar = bar.size() - 1;
  for ( byte i = 0; i < N_SENSOR; i++ ) {
    bar.fill_solid(CRGB::Black);
    bar[i] = CRGB::Green;

    ledsLeftDown(halfBar, 0) = bar(0, halfBar);
    ledsRightDown(0, halfBar) = bar(halfBar + 1, fullBar);
    // mirror bar
    ledsLeftUp(0, halfBar) = ledsLeftDown(0, halfBar);
    ledsRightUp(0, halfBar) = ledsRightDown(0, halfBar);
    FastLED.show();
    delay(333);
  }

  // down lights update
  leftDown.fill_solid(CRGB::Black);
  leftDown[0] = CRGB::Green;
  leftDown[leftDown.size()-1] = CRGB::Red;
  rightDown = leftDown;
  FastLED.show();
  delay(333);

  // up lights update
  leftUp.fill_solid(CRGB::Black);
  leftUp[0] = CRGB::Green;
  leftUp[leftUp.size()-1] = CRGB::Red;
  rightUp = leftUp;
  FastLED.show();
  delay(333);

}




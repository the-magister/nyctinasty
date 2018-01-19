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
// D5..D8 used

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
struct sAF_t { boolean hasUpdate=false; SepalArchFrequency freq; } sAF[N_ARCH];

// color choices, based on arch and sepal information
byte archHue[N_ARCH] = {HUE_RED, HUE_GREEN, HUE_BLUE};
byte archSat[N_ARCH] = {128, 128, 128};

// our internal storage, mapped to the hardware.
// pay no attention to the man behind the curtain.
#define NUM_PINS 4
#define LEDS_PER_PIN 20
CRGB leds[NUM_PINS * LEDS_PER_PIN];
CRGBSet left(&leds[0 * LEDS_PER_PIN], LEDS_PER_PIN); // D5
CRGBSet right(&leds[1 * LEDS_PER_PIN], LEDS_PER_PIN); // D6
CRGBSet unused1(&leds[2 * LEDS_PER_PIN], LEDS_PER_PIN); // D7
CRGBSet unused2(&leds[3 * LEDS_PER_PIN], LEDS_PER_PIN); // D8

#define COLOR_ORDER RGB
#define COLOR_CORRECTION TypicalLEDStrip

void setup() {
  Serial.begin(115200);

  Serial << endl << endl << endl << F("Startup.") << endl;

  // start comms
  comms.begin(role);

  // subscribe
  comms.subscribe(&sC.settings, &sC.hasUpdate); 
  for ( byte i = 0; i < N_ARCH; i++ ) {
    comms.subscribe(&sAF[i].freq, &sAF[i].hasUpdate, comms.getSepal(), i*2);
  }

  // lights
  FastLED.addLeds<WS2811_PORTA, NUM_PINS, COLOR_ORDER>(leds, LEDS_PER_PIN).setCorrection(COLOR_CORRECTION);
  FastLED.setBrightness(255);
  runStartupPattern();
  
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
    // show a throbbing rainbow background
    hue++;
    left.fill_rainbow(hue, 255 / left.size()); // paint
    right.fill_rainbow(hue + 128, -255 / right.size());
    
    /*
    // trails
    const byte bpm = 16; // bpm (rate of dot travel), 1/minute
    le.fadeToBlackBy(bpm);
    ledsD.fadeToBlackBy(bpm);
    // set the speed the pixel travels, see: lib8tion.h
    const uint16_t endUp = ledsC.size() - 1;
    uint16_t posVal = beatsin16(bpm, 0, (uint16_t)endUp);

    // paint
    ledsC[posVal] = CHSV(hue, 255, 255);
    // mirrored direction and hue
    ledsD[endUp - posVal] = CHSV(hue + 128, 255, 255);
    */
  }

}

void normal() {
  // check for an update to frequency
  for ( byte i = 0; i < N_ARCH; i++ ) {
    if ( sAF[i].hasUpdate ) {
      // map frequency to legs
      mapFreqToLegs();

      // reset
      sAF[i].hasUpdate = false;
    }
  }
}

void central() {
  // probably want
  // https://github.com/ppelleti/esp-opc-server/blob/master/esp-opc-server.ino
}

// total GARBAGE
void mapFreqToLegs() {
  uint16_t avgPower[N_SENSOR] = {0};
  // frequency power
  uint16_t power[N_SENSOR][N_FREQ_BINS] = {{0.0}};

  // do the boneheaded thing and sum up the bins across all sensors
  uint32_t sumSensors[N_FREQ_BINS] = {0};
  uint32_t maxSum = 0;
  for ( uint16_t j = 0; j < N_FREQ_BINS ; j++ ) {
    for ( byte i = 0; i < N_SENSOR; i++ ) {
      sumSensors[j] += sAF[0].freq.power[i][j];
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
    left[j % LEDS_PER_PIN] = CHSV(archHue[0], archSat[0], brighten8_video(constrain(value, 0, 255)));
  }
  Serial << endl;
  right = left;
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
  left.fill_solid(CRGB::Purple);
  right.fill_solid(CRGB::Aqua);
  unused1.fill_solid(CRGB::Red);
  unused2.fill_solid(CRGB::Red);
  FastLED.show();
  delay(333);

  left.fill_solid(CRGB::Black);
  right.fill_solid(CRGB::Black);
  unused1.fill_solid(CRGB::Black);
  unused2.fill_solid(CRGB::Black);
  FastLED.show();
  delay(333);
}




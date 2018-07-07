// IDE Settings:
// Tools->Board : "WeMos D1 R2 & mini"
// Tools->Flash Size : "4M (3M SPIFFS)"
// Tools->CPU Frequency : "160 MHz"
// Tools->Upload Speed : "921600"

#include <Streaming.h>
#include <Metro.h>
#include <SoftwareSerial.h>
#include <SoftEasyTransfer.h>
#include <FiniteStateMachine.h>
#define FASTLED_INTERRUPT_RETRY_COUNT 1
#include <FastLED.h>
#include "Nyctinasty_Messages.h"
#include "Nyctinasty_Comms.h"

// spammy?
#define SHOW_SERIAL_DEBUG true

// my role and arch number
NyctRole myRole = N_ROLES; // see Nyctinasty_Comms.h; set N_ROLES to pull from EEPROM
// Arch0, Arch1, Arch2 for bootstrapping a new controller.
byte myArch;
byte leftArch, rightArch;
byte leftCoord, rightCoord;

// wire it up
#define RX D1 // GPIO5
#define TX D2 // GPIO4 
// voltage divider: 5V->3.3V
// TX to RX_ESP via 820 Ohm resistor
// RX_ESP to GND via 1500 Ohm resistor
// LEDs are connected:
// D5, GPIO14
// D6, GPIO12
// D7, GPIO13
// D8, GPIO15
// also used
// D4, GPIO2, BUILTIN_LED

// LED handling

// our internal storage, mapped to the hardware.
// pay no attention to the man behind the curtain.
#define COLOR_ORDER RGB
#define COLOR_CORRECTION TypicalLEDStrip
#define NUM_PINS 4
#define LEDS_BAR 3
#define LEDS_DOWN 17
#define LEDS_UP 13
#define LEDS_DECK 4

CRGBArray < LEDS_BAR + LEDS_DOWN > leftBack;
CRGBArray < LEDS_BAR + LEDS_DOWN > rightBack;
CRGBArray < LEDS_BAR + LEDS_UP + LEDS_DECK > leftFront;
CRGBArray < LEDS_BAR + LEDS_UP + LEDS_DECK > rightFront;

// bars
CRGBSet rightBar1 = rightBack(0, LEDS_BAR - 1);
CRGBSet rightBar2 = rightFront(0, LEDS_BAR - 1);
CRGBSet leftBar1 = leftBack(0, LEDS_BAR - 1);
CRGBSet leftBar2 = leftFront(0, LEDS_BAR - 1);

// verticals
CRGBSet leftUp = leftFront(LEDS_BAR, LEDS_BAR + LEDS_UP - 1);
CRGBSet rightUp = rightFront(LEDS_BAR, LEDS_BAR + LEDS_UP - 1);
CRGBSet leftDown = leftBack(LEDS_BAR, LEDS_BAR + LEDS_DOWN - 1);
CRGBSet rightDown = rightBack(LEDS_BAR, LEDS_BAR + LEDS_DOWN - 1);

// deck lights under rail
CRGBSet leftDeck = leftFront(LEDS_BAR + LEDS_UP, LEDS_BAR + LEDS_UP + LEDS_DECK - 1);
CRGBSet rightDeck = rightFront(LEDS_BAR + LEDS_UP, LEDS_BAR + LEDS_UP + LEDS_DECK - 1);

// color choices, based on arch information
const CHSV archColor[N_ARCH] = {
  CHSV(HUE_RED, 255, 255),
  CHSV(HUE_GREEN, 255, 255),
  CHSV(HUE_BLUE, 255, 255)
};

// deck lighting
CRGB deckColor = CRGB::FairyLight;

// comms
NyctComms comms;

// define a state for every systemState
void startup(); State Startup = State(startup);
void lonely(); State Lonely = State(lonely);
void ohai(); State Ohai = State(ohai);
void goodnuf(); State Goodnuf = State(goodnuf);
void goodjob(); State Goodjob = State(goodjob);
void winning(); State Winning = State(winning);
void fanfare(); State Fanfare = State(fanfare);
void reboot() {
  comms.reboot();
}; State Reboot = State(reboot);
FSM stateMachine = FSM(Startup); // initialize state machine

// incoming message storage and flag for update
struct sC_t {
  boolean hasUpdate = false;
  SystemCommand settings;
} sC;

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
  Serial << F("Configure softwareserial...");
  mySerial.begin(115200);
  // messages
  ETin.begin(details(dist), &mySerial);
  // after set up the input pin
  pinMode(TX, OUTPUT); // trigger to send
  Serial << F(" done.") << endl;

  // LEDs
  Serial << F("Configure leds...");
  FastLED.addLeds<WS2811, D5, COLOR_ORDER>(leftBack, leftBack.size()).setCorrection(COLOR_CORRECTION);
  FastLED.addLeds<WS2811, D6, COLOR_ORDER>(rightBack, rightBack.size()).setCorrection(COLOR_CORRECTION);
  FastLED.addLeds<WS2811, D7, COLOR_ORDER>(leftFront, leftFront.size()).setCorrection(COLOR_CORRECTION);
  FastLED.addLeds<WS2811, D8, COLOR_ORDER>(rightFront, rightFront.size()).setCorrection(COLOR_CORRECTION);
  FastLED.setBrightness(255);
  Serial << F(" done.") << endl;

  // configure comms
  comms.begin(myRole);
  myRole = comms.getRole();
  myArch = myRole - 1;
  switch (myArch) {
    case 0:
      // sC.isPlayer indexes
      leftArch = 1; rightArch = 2;
      // sC.areCoordinated indexes
      leftCoord = 0; rightCoord = 2;
      break;
    case 1:
      leftArch = 2; rightArch = 0;
      leftCoord = 1; rightCoord = 0;
      break;
    case 2:
      leftArch = 0; rightArch = 1;
      leftCoord = 2; rightCoord = 1;
      break;
  }
  Serial << F("Arch indexes: left=") << leftArch << F(" my=") << myArch << F(" right=") << rightArch << endl;
  Serial << F("Coordination indexes: left=") << leftCoord << F(" right=") << rightCoord << endl;

  // subscribe
  comms.subscribe(&sC.settings, &sC.hasUpdate);

  Serial << F("DISTANCE_SAMPLING_RATE, ms: ") << DISTANCE_SAMPLING_RATE << endl;
  Serial << F("DISTANCE_SAMPLING_FREQ, Hz: ") << DISTANCE_SAMPLING_FREQ << endl;

  Serial << F("Startup complete.") << endl;
}

void loop() {
  // comms handling
  comms.update();

  // check for settings update
  if ( sC.hasUpdate ) {
    updateState();
    calculateCoordPalette();
    calculatePlayerPalette();
    sC.hasUpdate = false;
  }

  // do stuff
  stateMachine.update();

  // update the sensor data
  updateSensors();
}

CRGBPalette16 coordPalette;
void calculateCoordPalette() {

  CRGBArray<16> pal;

  byte totalCoord = sC.settings.areCoordinated[0] + sC.settings.areCoordinated[1] + sC.settings.areCoordinated[2];

  switch (totalCoord) {
    case 0: coordPalette = CloudColors_p ; break;
    case 1: coordPalette = LavaColors_p; break;
    case 2: coordPalette = RainbowColors_p; break;
    case 3: coordPalette = PartyColors_p; break;
  }

}

CRGBPalette16 playerPaletteLeft, playerPaletteRight; // left, right
void calculatePlayerPalette() {

  CHSV me = CHSV(archColor[myArch].hue, archColor[myArch].sat,
                 sC.settings.isPlayer[myArch] ? 255 : 0
                );
  CHSV left = CHSV(archColor[leftArch].hue, archColor[leftArch].sat,
                   sC.settings.isPlayer[leftArch] ? 255 : 0
                  );
  CHSV right = CHSV(archColor[rightArch].hue, archColor[rightArch].sat,
                    sC.settings.isPlayer[rightArch] ? 255 : 0
                   );

  CRGBArray<16> palLeft;
  palLeft.fill_gradient(left, me, left);

  CRGBArray<16> palRight;
  palRight.fill_gradient(right, me, right);

  // copy out
  for (byte i = 0; i < 16; i++) {
    playerPaletteLeft[i] = palLeft[i];
    playerPaletteRight[i] = palRight[i];
  }
}


void updateState() {

  switch ( sC.settings.state ) {
    case STARTUP: stateMachine.transitionTo(Startup); break;
    case LONELY: stateMachine.transitionTo(Lonely); break;
    case OHAI: stateMachine.transitionTo(Ohai); break;
    case GOODNUF: stateMachine.transitionTo(Goodnuf); break;
    case GOODJOB: stateMachine.transitionTo(Goodjob); break;
    case WINNING: stateMachine.transitionTo(Winning); break;
    case FANFARE: stateMachine.transitionTo(Fanfare); break;
    case REBOOT: stateMachine.transitionTo(Reboot); break;
    default:
      Serial << F("ERROR!  unknown state.") << endl;
  }

  Serial << F("State change. to=");
  switch ( sC.settings.state ) {
    case STARTUP: Serial << "STARTUP"; break;  //  all roles start here

    case LONELY: Serial << "LONELY"; break;   // 0 players
    case OHAI: Serial << "OHAI"; break;   // 1 players
    case GOODNUF: Serial << "GOODNUF"; break;  // 2 players
    case GOODJOB: Serial << "GOODJOB"; break;  // 3 players or 2 players coordinated
    case WINNING: Serial << "WINNING"; break;  // 3 players and 2 players coordinated
    case FANFARE: Serial << "FANFARE"; break;  // 3 players and 3 players coordinated

    case REBOOT: Serial << "REBOOT"; break;   //  trigger to reboot
  }
  Serial << ". isPlayer? A0=" << sC.settings.isPlayer[0];
  Serial << " A1=" << sC.settings.isPlayer[1];
  Serial << " A2=" << sC.settings.isPlayer[2];
  Serial << ". coord? A01=" << sC.settings.areCoordinated[0];
  Serial << " A12=" << sC.settings.areCoordinated[1];
  Serial << " A20=" << sC.settings.areCoordinated[2];
  Serial << endl;

}

void startup() {

  static byte hue = archColor[myArch].hue;
  EVERY_N_MILLISECONDS(30) {
    // show a throbbing rainbow background
    hue++;
    leftUp.fill_rainbow(hue, 255 / leftUp.size()); // paint
    rightUp = leftUp;
    leftDown.fill_rainbow(hue + 128, -255 / leftDown.size());
    rightDown = leftFront;

    // color the bar with our color
    leftBar1.fill_solid(archColor[myArch]);
    rightBar2 = rightBar1 = leftBar2 = leftBar1;

    // do something useful with the deck lights
    updateDeck();

    // do it now
    pushToHardware();
  }

}

void lonely() {
  // need some kind of "walk into the arches" animation
  playerAndCoordinationUpdate();
}

void ohai() {
  playerAndCoordinationUpdate();
}

void goodnuf() {
  playerAndCoordinationUpdate();
}

void goodjob() {
  playerAndCoordinationUpdate();
}

void winning() {
  playerAndCoordinationUpdate();
}

void fanfare() {
  // need some kind of "yeah, playah!" animation
  playerAndCoordinationUpdate();
}

void playerAndCoordinationUpdate() {
  updateDeck();
  updateTopsByCoordination();
  updateLegsByPlayer();
  addSomeSparkles();
}

void addSomeSparkles() {
  // little strobes of color

  EVERY_N_MILLISECONDS( 100 ) {
    static byte which = 0;
    static byte colorIndex = 0;
    const byte brightness = 16;

    //    CRGB color = ColorFromPalette( CloudColors_p, colorIndex, brightness, LINEARBLEND );
    CRGB color = CRGB::FairyLight;
    color -= CRGB(64, 64, 64);

    switch ( which ) {
      case 0: leftUp[random8(leftUp.size())] += color; break;
      case 1: rightUp[random8(rightUp.size())] += color; break;
      case 2: leftDown[random8(leftDown.size())] += color; break;
      case 3: rightDown[random8(rightDown.size())] += color; break;

      case 4: leftDeck[random8(leftDeck.size())] += color; break;
      case 5: rightDeck[random8(rightDeck.size())] += color; break;
    }
    which++;
    if ( which >= 5 ) which = 0;
    colorIndex ++;
  }
}


void updateDeck() {
  leftDeck.fill_solid(deckColor);
  leftDeck.fadeLightBy( 64 ); // dim it

  rightDeck = leftDeck;
}

void updateTopsByCoordination() {
  static byte colorIndex = 0;

  EVERY_N_MILLISECONDS( 20 ) {
    colorIndex ++;
    byte j = 0;
    for ( int i = 0; i < leftUp.size(); i++) {
      leftUp[i] = ColorFromPalette( coordPalette, colorIndex + j++, 255, LINEARBLEND );
    }
    j = 0;
    for ( int i = 0; i < rightUp.size(); i++) {
      rightUp[i] = ColorFromPalette( coordPalette, colorIndex + j++, 255, LINEARBLEND );
    }
  }
}

void updateLegsByPlayer() {
  static byte colorIndex = 0;

  EVERY_N_MILLISECONDS( 20 ) {
    colorIndex ++;
    byte j = 0;
    for ( int i = 0; i < leftDown.size(); i++) {
      leftDown[i] = ColorFromPalette( playerPaletteLeft, colorIndex + j++, 255, LINEARBLEND );
    }
    j = 0;
    for ( int i = 0; i < rightDown.size(); i++) {
      rightDown[i] = ColorFromPalette( playerPaletteRight, colorIndex + j++, 255, LINEARBLEND );
    }
  }

}

void updateSensors() {

  /*
     Order of operation is critical here.  We have three subsystems that use ISRs:
      FastLED: turns off ISRs
      WiFi: received information may be lost w/o ISR
      SoftwareSerial: received information may be lost w/o ISR

      From this, you can see that we need to be very careful with FastLED.show(), and
      queue those up after we get SoftwareSerial data.  Can't really control when we get
      WiFi packets.

  */

  static Metro distanceUpdate(DISTANCE_SAMPLING_RATE);
  static boolean pinState = false;
  static uint32_t counter = 0;

  if ( distanceUpdate.check() ) {
    distanceUpdate.reset();
    pinState = !pinState;
    digitalWrite(TX, pinState);
  }

  // get data from ADCs
  if ( ETin.receiveData() ) {
    // just tracking actual distance update rate
    counter ++;

    // update bar lights
    updateBarByDistance();

    // publish distance
    comms.publish(&dist, myArch);

    // we need to update the LEDs now while we won't screw up SoftwareSerial and WiFi
    pushToHardware();
  }

  const uint32_t reportInterval = 10;
  EVERY_N_SECONDS( reportInterval ) {
    uint32_t actualDISTANCE_SAMPLING_RATE = (reportInterval * 1000UL) / counter;

    Serial << F("Distance sample interval, actual=") << actualDISTANCE_SAMPLING_RATE;
    Serial << F(" hypothetical=") << DISTANCE_SAMPLING_RATE;
    Serial << F(" ms.") << endl;

    counter = 0;
  }

}

// adjust lights on the bar with distance readings
void updateBarByDistance() {

  // compute using a dummy set of LEDs
  static CRGBArray<N_SENSOR> bar;
  for ( byte s = 0; s < N_SENSOR; s++ ) {
    // quash noise
    uint16_t prox = dist.prox[s] < dist.noise ? dist.min : dist.prox[s];
//    if ( dist.prox[s] < dist.noise ) dist.prox[s] = dist.min;
    uint16_t intensity = map(
//                           dist.prox[s],
                           prox,
                           dist.min, dist.max,
                           (uint16_t)0, (uint16_t)255
                         );
    bar[s] = CHSV(archColor[myArch].hue, archColor[myArch].sat, (byte)intensity);
  }

  // assign to hardware. ugly and direct, but we can see what's going on.
  leftBack[2] = leftFront[2] = bar[0];
  leftBack[1] = leftFront[1] = bar[1];
  leftBack[0] = leftFront[0] = bar[2];

  rightBack[0] = rightFront[0] = bar[3];
  rightBack[1] = rightFront[1] = bar[4];
  rightBack[2] = rightFront[2] = bar[5];
}

// show
void pushToHardware() {
  FastLED.show();

  // show our frame rate, periodically
  EVERY_N_SECONDS( 20 ) {
    uint16_t reportedFPS = FastLED.getFPS();
    Serial << F("FastLED reported FPS, Hz=") << reportedFPS << endl;
  }
}



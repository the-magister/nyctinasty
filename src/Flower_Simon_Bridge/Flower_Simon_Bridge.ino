// IDE Settings:
// Tools->Board : "WeMos D1 R2 & mini"
// Tools->Flash Size : "4M (3M SPIFFS)"
// Tools->CPU Frequency : "160 MHz"
// Tools->Upload Speed : "921600"

#include <Streaming.h>
#include <Metro.h>
#include <FiniteStateMachine.h>
#include <RFM69.h>
#include <FastLED.h>
#include "Nyctinasty_Messages.h"
#include "Simon_Common.h"
#include "Nyctinasty_Comms.h"

// comms
NyctComms comms;

// wire it up
// Mini -> RFM69 board
// D8 -> CS
// D7 -> MOSI
// D6 -> MISO
// D5 -> SCK
// D2 -> D0
// D1 -> RST
// 3V3 -> VIN
// G -> GND
#define RF69_SPI_CS   D8
#define RF69_IRQ_PIN  D2
#define RF69_IRQ_NUM  digitalPinToInterrupt(RF69_IRQ_PIN)
#define RF69_RST_PIN  D1
RFM69 radio;

// define a state for every systemState
void idle(); State Idle = State(idle);
void normal(); State Normal = State(normal);
void central(); State Central = State(central);
void reboot() { comms.reboot(); }; State Reboot = State(reboot);
void reprogram() { comms.reprogram("Flower_Simon_Bridge.ino.bin"); } State Reprogram = State(reprogram);
FSM stateMachine = FSM(Idle); // initialize state machine

// my role
NyctRole role = FxSimon;

// incoming message storage and flag for update
struct sC_t { boolean hasUpdate=false; SystemCommand settings; } sC;
struct sM_t { boolean hasUpdate=false; SimonSystemState simonMessage; } sM;

void setup() {
  // for local output
  Serial.begin(115200);

  Serial << endl << endl << endl << F("Startup.") << endl;

  // start comms
  comms.begin(role);

  // subscriptions
  comms.subscribe(&sC.settings, &sC.hasUpdate);
  comms.subscribe(&sM.simonMessage, &sM.hasUpdate);

  Serial << F("Startup. Initializing RFM69HW radio.") << endl;
  radioInitialize();

  Serial << F("Startup complete.") << endl;
}

void loop() {
  // comms handling
  comms.update();

  // bail out if not connected
  if ( ! comms.isConnected() ) return;

  // check for settings update
  if ( sC.hasUpdate ) {
    switchState(sC.settings.state);
    sC.hasUpdate = false;
  }

  // radio handling
  if (radio.receiveDone()) {
    Serial << F("Radio.  packet received. size=") << radio.DATALEN << endl;
  }

  // do stuff
  stateMachine.update();
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
  // dick around with the lighting 
  static Metro sendInterval(25UL);
  if( sendInterval.check() ) {

    // colors
    static byte hue[3] = {HUE_RED,HUE_GREEN,HUE_BLUE};
    for( byte i=0; i<3; i++ ) {
      hue[i]+=1;
      CRGB color = CHSV(hue[i], 255, 255);
 
      // set in message
      sM.simonMessage.light[i].red = color.red;
      sM.simonMessage.light[i].green = color.green;
      sM.simonMessage.light[i].blue = color.blue;
    }
    
    // flame effects
    fireInstruction newFire;
    newFire.duration = constrain(0, 0, 255);
    newFire.effect = constrain(veryLean, veryRich, veryLean);

    // set in message
    sM.simonMessage.fire[0] = newFire;
    sM.simonMessage.fire[1] = newFire;
    sM.simonMessage.fire[2] = newFire;

    // ship it
    radioSend();
    sendInterval.reset();
  }
}
void normal() {
  // look for messages and serve as a bridge

  // packets get dropped.  we resend on an interval
  static Metro sendInterval(25UL);

  static byte packetNumber = 255;
  if( sM.hasUpdate ) {
    packetNumber ++;
    sM.simonMessage.packetNumber = packetNumber;
  
    // ship it
    radioSend();
    sendInterval.reset();
    
    sM.hasUpdate = false;
  }

  if( sendInterval.check() ) {
    // ship it
    radioSend();
    sendInterval.reset();
  }
}

void central() { 
  normal(); 
}

void radioSend() {
  const systemMode sMode = EXTERN;
  sM.simonMessage.mode = (byte)sMode;

  radio.send((byte)BROADCAST, (const void*)(&sM.simonMessage), sizeof(SimonSystemState), false);  
}

void radioReset() {
  // Hard Reset the RFM module
  pinMode(RF69_RST_PIN, OUTPUT);
  digitalWrite(RF69_RST_PIN, HIGH);
  delay(100);
  digitalWrite(RF69_RST_PIN, LOW);
  delay(100);  
}

void radioInitialize() {
  radio = RFM69(RF69_SPI_CS, RF69_IRQ_PIN, true, RF69_IRQ_NUM);

  radioReset();
  
  radio.initialize(RF69_915MHZ, CONSOLE, D_GROUP_ID);
  radio.setHighPower(); //uncomment only for RFM69H(C)W!
  radio.promiscuous(true);
}


// compile for WeMos D1 R2 & mini
#include <Streaming.h>
#include <Metro.h>
#include <FiniteStateMachine.h>
//#include <ESP8266httpUpdate.h>
#include "Nyctinasty_Messages.h"
#include "Nyctinasty_Comms.h"
#include "Simon_Common.h"
#include <RFM69.h>
#include <SPI.h>

// define a state for every systemState
void idleUpdate(); State Idle = State(idleUpdate);
void normalUpdate(); State Normal = State(normalUpdate);
State Reboot = State(reboot);
void reprogram() {
  reprogram("Flower_Simon_Bridge.ino.bin");
}; State Reprogram = State(reprogram);
FSM stateMachine = FSM(Idle); //initialize state machine

// who am I?
const String id = commsIdFlowerSimonBridge();

// ship settings
SystemCommand settings;
// in this topic
const String settingsTopic = commsTopicSystemCommand();
// and sets this true when an update arrives
boolean settingsUpdate = false;

// for comms
#define RF69_IRQ_PIN  D2
#define RF69_IRQ_NUM  digitalPinToInterrupt(RF69_IRQ_PIN)
#define RF69_SPI_CS   D8
#define RF69_RST_PIN  D1
RFM69 radio;
simonSystemState simonMessage;

void setup() {
  // for local output
  Serial.begin(115200);

  Serial << endl << endl << F("Startup") << endl;

  commsBegin(id);
  commsSubscribe(settingsTopic, &settings, &settingsUpdate, 1); // QoS 1

  Serial << F("Startup.  Initializing WiFi radio.") << endl;
  connectWiFi();
  delay(10000);
  connectMQTT("192.168.3.1");
  delay(3000);
  
  Serial << F("Startup.  Initializing RFM69HW radio.") << endl;
  radioInitialize();

  Serial << F("Startup.  sizeof simonSystemState=") << sizeof(simonSystemState) << endl;

  Serial << F("Startup complete.") << endl;
}

void loop() {
  // comms handling
  commsUpdate();

  // bail out if not connected
  if ( ! commsConnected() ) return;

  // check for settings update
  if ( settingsUpdate ) {
    switchState(settings.state);
    settingsUpdate = false;
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
    case STARTUP: stateMachine.transitionTo(Idle); break;
    case NORMAL: stateMachine.transitionTo(Normal); break;
    case CENTRAL: stateMachine.transitionTo(Idle); break;
    case REBOOT: stateMachine.transitionTo(Reboot); break;
    case REPROGRAM: stateMachine.transitionTo(Reprogram); break;
    default:
      Serial << F("ERROR!  unknown state.") << endl;
  }
}

void idleUpdate() {
  static Metro sendInterval(20UL);
  if( sendInterval.check() ) {

    // colors
    static byte hue[3] = {HUE_RED,HUE_GREEN,HUE_BLUE};
    for( byte i=0; i<3; i++ ) {
      hue[i]+=1;
      CRGB color = CHSV(hue[i], 255, 255);
      colorInstruction newColor;
      newColor.red = color.red;
      newColor.green = color.green;
      newColor.blue = color.blue;

      // set in message
      simonMessage.light[i] = newColor;
    }
    
    // flame effects
    fireInstruction newFire;
    newFire.duration = constrain(0, 0, 255);;
    newFire.effect = constrain(veryLean, veryRich, veryLean);

    // set in message
    simonMessage.fire[0] = newFire;
    simonMessage.fire[1] = newFire;
    simonMessage.fire[2] = newFire;
    simonMessage.fire[3] = newFire; 

    // ship it
    radioSend();
    sendInterval.reset();
  }
}
void normalUpdate() {
}

void radioSend() {
  const systemMode sMode = EXTERN;
  simonMessage.mode = (byte)sMode;

  simonMessage.packetNumber++;

  radio.send((byte)BROADCAST, (const void*)(&simonMessage), sizeof(simonSystemState), false);  
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


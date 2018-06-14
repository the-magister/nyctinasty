// IDE Settings:
// Tools->Board : "WeMos D1 R2 & mini"
// Tools->Flash Size : "4M (3M SPIFFS)"
// Tools->CPU Frequency : "160 MHz"
// Tools->Upload Speed : "921600"

#include <Streaming.h>
#include <Metro.h>
#include <FiniteStateMachine.h>
#include "Nyctinasty_Messages.h"
#include "Nyctinasty_Comms.h"

// my role and arch number
NyctRole myRole = N_ROLES; // see Nyctinasty_Comms.h; set N_ROLES to pull from EEPROM
//NyctRole myRole = WaterRoute; 
//NyctRole myRole = WaterPumps1; 
//NyctRole myRole = WaterPumps2; 

// wire it up
// devices with the relay shield only have access to D1
#define PIN_CANNON D1
// devices with the light shield have access to D5-D8
#define PIN_PUMP1 D5
#define PIN_PUMP2 D6
#define PIN_PUMP3 D7
#define PIN_PUMP4 D8
// also used
// D4, GPIO2, BUILTIN_LED

// comms
NyctComms comms;

// define a state for every systemState
void startup(); State Startup = State(startup);
void offline(); State Offline = State(offline);
void online(); State Online = State(online);
void slaved(); State Slaved = State(slaved);
void reboot() {
  comms.reboot();
}; State Reboot = State(reboot);
void reprogram() {
  comms.reprogram("Pumps.ino.bin");
}; State Reprogram = State(reprogram);
FSM stateMachine = FSM(Startup); // initialize state machine

// incoming message storage and flag for update
struct sC_t {
  boolean hasUpdate = false;
  SystemCommand settings;
} sC;
struct pC_t {
  boolean hasUpdate = false;
  WaterWorks water;
} pC;

void applyToHardware() {
  switch (myRole) {
    case WaterRoute:
      digitalWrite(PIN_CANNON, pC.water.route);
      Serial << "Route: ";
      Serial << (pC.water.route == FOUNTAIN ? "Fountain" : "Cannon") << endl;

      break;

    case WaterPumps1:
      digitalWrite(PIN_PUMP1, pC.water.pump[0]);
      Serial << "Pump 1: ";
      Serial << (pC.water.pump[0] == ON ? "ON" : "off") << endl;

      digitalWrite(PIN_PUMP2, pC.water.pump[1]);
      Serial << "Pump 2: ";
      Serial << (pC.water.pump[1] == ON ? "ON" : "off") << endl;

      break;

    case WaterPumps2:
      digitalWrite(PIN_PUMP3, pC.water.pump[2]);
      Serial << "Pump 3: ";
      Serial << (pC.water.pump[2] == ON ? "ON" : "off") << endl;

      digitalWrite(PIN_PUMP4, pC.water.pump[3]);
      Serial << "Pump 4: ";
      Serial << (pC.water.pump[3] == ON ? "ON" : "off") << endl;

      break;
  }
}

void setup() {
  // set them off, then enable pin.
  digitalWrite(PIN_CANNON, LOW); pinMode(PIN_CANNON, OUTPUT);
  digitalWrite(PIN_PUMP1, LOW); pinMode(PIN_PUMP1, OUTPUT);
  digitalWrite(PIN_PUMP2, LOW); pinMode(PIN_PUMP2, OUTPUT);
  digitalWrite(PIN_PUMP3, LOW); pinMode(PIN_PUMP3, OUTPUT);
  digitalWrite(PIN_PUMP4, LOW); pinMode(PIN_PUMP4, OUTPUT);

  // for local output
  Serial.begin(115200);
  Serial << endl << endl << endl << F("Startup begins.") << endl;

  // configure comms
  comms.begin(myRole);
  myRole = comms.getRole();

  // subscribe
  comms.subscribe(&sC.settings, &sC.hasUpdate);
  comms.subscribe(&pC.water, &pC.hasUpdate);

  Serial << F("Startup complete.") << endl;
  Serial << F("'1','2','3' or '4' to toggle pumps.  'r' to toggle route.") << endl;
}

void loop() {
  // comms handling
  comms.update();

  // check for settings update
  if ( sC.hasUpdate ) {
    switchState(sC.settings.state);
    sC.hasUpdate = false;
  }

  // check for serial input
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'r': pC.water.route = (pC.water.route == CANNON) ? FOUNTAIN : CANNON; pC.hasUpdate = true; break;
      case '1': pC.water.pump[0] = (pC.water.pump[0] == ON) ? OFF : ON; pC.hasUpdate = true; break;
      case '2': pC.water.pump[1] = (pC.water.pump[1] == ON) ? OFF : ON; pC.hasUpdate = true; break;
      case '3': pC.water.pump[2] = (pC.water.pump[2] == ON) ? OFF : ON; pC.hasUpdate = true; break;
      case '4': pC.water.pump[3] = (pC.water.pump[3] == ON) ? OFF : ON; pC.hasUpdate = true; break;
      case '\n': break;
      case '\r': break;
      default:
        Serial << F("'1','2','3' or '4' to toggle pumps.  'r' to toggle route.") << endl;
        break;
    }
  }

  // do stuff
  stateMachine.update();
}

void switchState(systemState state) {
  Serial << F("State.  Changing to ") << state << endl;
  switch ( state ) {
    case STARTUP: stateMachine.transitionTo(Startup); break;
    case OFFLINE: stateMachine.transitionTo(Offline); break;
    case ONLINE: stateMachine.transitionTo(Online); break;
    case SLAVED: stateMachine.transitionTo(Slaved); break;
    case REBOOT: stateMachine.transitionTo(Reboot); break;
    case REPROGRAM: stateMachine.transitionTo(Reprogram); break;
    default:
      Serial << F("ERROR!  unknown state.") << endl;
  }
}

void startup() {
  // update with defaults
  applyToHardware();

  // transition to Offline, but we could easily get directed to Online before that.
  stateMachine.transitionTo(Offline);

}

void offline() {
  if ( comms.isConnected() ) {
    Serial << F("GOOD.  online!") << endl;
    stateMachine.transitionTo(Online);
  } else {
    normal(false);
  }
}

void online() {
  if ( comms.isConnected() ) {
    normal(true);
  } else {
    Serial << F("WARNING.  offline!") << endl;
    // reset back to defaults
    WaterWorks tmp;
    pC.water = tmp;
    pC.hasUpdate = true;

    stateMachine.transitionTo(Offline);
  }
  
}

void normal(boolean isOnline) {
  // check for update
  if ( pC.hasUpdate ) {
    // and do it.
    applyToHardware();
    pC.hasUpdate = false;
  }
}

void slaved() {
  // NOP, currently.
}



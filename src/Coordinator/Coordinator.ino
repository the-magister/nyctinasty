// IDE Settings:
// Tools->Board : "WeMos D1 R2 & mini"
// Tools->Flash Size : "4M (3M SPIFFS)"
// Tools->CPU Frequency : "160 MHz"
// Tools->Upload Speed : "921600"

#include <Streaming.h>
#include <Metro.h>
#include <SoftwareSerial.h>
#include <FiniteStateMachine.h>
#include "Nyctinasty_Messages.h"
#include "Nyctinasty_Comms.h"

// spammy?
#define SHOW_SERIAL_DEBUG true

// my role and arch number
NyctRole myRole = Coordinator; // see Nyctinasty_Comms.h; set N_ROLES to pull from EEPROM

// wire it up
// also used
// D4, GPIO2, BUILTIN_LED

// LED handling

// comms
NyctComms comms;

// define a state for every systemState
void startup(); State Startup = State(startup);
void offline(); State Offline = State(offline);
void online(); State Online = State(online);
void slaved(); State Slaved = State(slaved);
void reboot() { comms.reboot(); }; State Reboot = State(reboot);
void reprogram() { comms.reprogram("Sepal_Arch.ino.bin"); }; State Reprogram = State(reprogram);
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

// don't allow pump state switching too rapidly.
const uint32_t pumpChangeInterval = 1000UL;
byte targetPumpLevel = 0;

void setup() {
  // for local output
  Serial.begin(115200);

  Serial << endl << endl << endl << F("Startup begins.") << endl;

  // configure comms
  comms.begin(myRole);
  myRole = comms.getRole();
  
  // subscribe
  comms.subscribe(&sC.settings, &sC.hasUpdate);

  // for random numbers
  randomSeed(analogRead(0));

  Serial << F("Startup complete.") << endl;
  Serial << F("[0-4] to set pump level.  'r' to toggle route.") << endl;
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
      case '0': targetPumpLevel = 0; break;
      case '1': targetPumpLevel = 1; break;
      case '2': targetPumpLevel = 2; break;
      case '3': targetPumpLevel = 3; break;
      case '4': targetPumpLevel = 4; break;
      case '\n': break;
      case '\r': break;
      default:
          Serial << F("[0-4] to set pump level.  'r' to toggle route.") << endl;

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
   // after 5 seconds, transition to Offline, but we could easily get directed to Online before that.
  static Metro startupTimeout(5000UL);
  if( startupTimeout.check() ) stateMachine.transitionTo(Offline);
}
 
void offline() {
  if( comms.isConnected() ) {
    Serial << F("GOOD.  online!") << endl;
    stateMachine.transitionTo(Online);
  } else {
    normal(false);
  }
}

void online() {
  if( ! comms.isConnected() ) {
    Serial << F("WARNING.  offline!") << endl;
    stateMachine.transitionTo(Offline);
  } else {
    normal(true);
  }

}

void normal(boolean isOnline) {

  // queue up changes for online
  if( ! isOnline ) return;
  
  static Metro pumpChange(pumpChangeInterval);
  if( getPumpLevel() != targetPumpLevel ) {
    if( pumpChange.check() ) {
      
      changePumpLevel(); // update
      comms.publish(&pC.water); // push it
      
      pumpChange.reset();
    }
  }
}

byte getPumpLevel() {
  byte ret = 0;
  for( byte p=0; p<2; p++ ) ret += (pC.water.primePump[p] == ON) + (pC.water.boostPump[p] == ON);
  return( ret );
}

void changePumpLevel() {
  byte currentPumpLevel = getPumpLevel();
  
  Serial << "Pump level: " << currentPumpLevel << " -> " << targetPumpLevel << endl;
  
  if( currentPumpLevel < targetPumpLevel ) {
    switch( currentPumpLevel ) {
      case 0: addPrimePump(); break;
      case 1: addBoostPump(); break;
      case 2: addPrimePump(); break;
      case 3: addBoostPump(); break;
    }
  } else {
    switch( currentPumpLevel ) {
      case 1: subPrimePump(); break;
      case 2: subBoostPump(); break;
      case 3: subPrimePump(); break;
      case 4: subBoostPump(); break;
    }
  }

  Serial << "\tRoute: ";
  Serial << (pC.water.route == FOUNTAIN ? "Fountain" : "Cannon") << endl;
  Serial << "\tPrime 1: ";
  Serial << (pC.water.primePump[0] == ON ? "ON" : "off") << endl;
  Serial << "\tPrime 2: ";
  Serial << (pC.water.primePump[1] == ON ? "ON" : "off") << endl;
  Serial << "\tBoost 1: ";
  Serial << (pC.water.boostPump[0] == ON ? "ON" : "off") << endl;
  Serial << "\tBoost 2: ";
  Serial << (pC.water.boostPump[1] == ON ? "ON" : "off") << endl;

}

// start with random value
static byte nextBoostPump = random(2); 
// start with random value
static byte nextPrimePump = random(2); 

void addBoostPump() {
  // odd case but
  if( pC.water.boostPump[0] == ON && pC.water.boostPump[1] == ON ) return;

  // set next in line on
  pC.water.boostPump[nextBoostPump] = ON;

  // roll to next
  nextBoostPump = !nextBoostPump;
}
void addPrimePump() {
  // odd case but
  if( pC.water.primePump[0] == ON && pC.water.primePump[1] == ON ) return;

  // set next in line on
  pC.water.primePump[nextPrimePump] = ON;

  // roll to next
  nextPrimePump = !nextPrimePump;
}

void subBoostPump() {
  // odd case but
  if( pC.water.boostPump[0] == OFF && pC.water.boostPump[1] == OFF ) return;

  // might need to toggle the same one
  if( (pC.water.boostPump[0] == ON) + (pC.water.boostPump[1] == ON) == 1 ) nextBoostPump != nextBoostPump;
  
  // set next in line on
  pC.water.boostPump[nextBoostPump] = OFF;

  // roll to next
  nextBoostPump = !nextBoostPump;
}
void subPrimePump() {
  // odd case but
  if( pC.water.primePump[0] == OFF && pC.water.primePump[1] == OFF ) return;

  // might need to toggle the same one
  if( (pC.water.primePump[0] == ON) + (pC.water.primePump[1] == ON) == 1 ) nextPrimePump != nextPrimePump;
  
  // set next in line on
  pC.water.primePump[nextPrimePump] = OFF;

  // roll to next
  nextPrimePump = !nextPrimePump;
}

void slaved() {
  // NOP, currently.  Will look for lighting directions, either through UDP (direct) or MQTT (procedural).
}


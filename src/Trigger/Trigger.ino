// IDE Settings:
// Tools->Board : "Adafruit Feather HUZZAH ESP8266"
// Tools->Flash Size : "4M (3M SPIFFS)"
// Tools->CPU Frequency : "160 MHz"
// Tools->Upload Speed : "921600"

#include <Streaming.h>
#include <Metro.h>
#include <FiniteStateMachine.h>
#include <Bounce2.h>
#include "Nyctinasty_Messages.h"
#include "Nyctinasty_Comms.h"

// my role and arch number
NyctRole myRole = Trigger; // see Nyctinasty_Comms.h; set N_ROLES to pull from EEPROM

// wire it up
// devices with the light shield have access to D5-D8
#define PIN_LEFT 14 // wire GPIO14 through a N.O. switch to GND
#define PIN_RIGHT 12 // wire GPIO12 through a N.O. switch to GND
// also used
// D4, GPIO2, BUILTIN_LED
Bounce left = Bounce(); 
Bounce right = Bounce(); 

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
void reboot() { comms.reboot(); }; State Reboot = State(reboot);
FSM stateMachine = FSM(Startup); // initialize state machine

// incoming message storage and flag for update
struct sC_t {
  boolean hasUpdate = false;
  SystemCommand settings;
} sC;

CannonTrigger cannon;

void setup() {
  // set them off, then enable pin.
  pinMode(PIN_LEFT, INPUT_PULLUP);
  left.attach(PIN_LEFT);
  left.interval(5); // interval in ms
  
  pinMode(PIN_RIGHT, INPUT_PULLUP);
  right.attach(PIN_RIGHT);
  right.interval(5); // interval in ms

  pinMode(BUILTIN_LED, OUTPUT);

  // for local output
  Serial.begin(115200);
  Serial << endl << endl << endl << F("Startup begins.") << endl;

  // configure comms
  comms.begin(myRole);
  myRole = comms.getRole();

  // subscribe
  comms.subscribe(&sC.settings, &sC.hasUpdate);

  Serial << F("Startup complete.") << endl;
}

void loop() {
  // comms handling
  comms.update();

  // check for settings update
  if ( sC.hasUpdate ) {
    switchState(sC.settings.state);
    sC.hasUpdate = false;
    // and push the trigger state
    comms.publish(&cannon);
    // show cannon state
    digitalWrite(BUILTIN_LED, !( cannon.right==TRIGGER_ON || cannon.left==TRIGGER_ON) );
  }

  // update 
  left.update();
  right.update();

  // Get the updated value :
  triggerState trigLeft = left.read()==LOW ? TRIGGER_ON : TRIGGER_OFF;
  triggerState trigRight = right.read()==LOW ? TRIGGER_ON : TRIGGER_OFF;
  
  if( trigLeft != cannon.left || trigRight != cannon.right ) {
    cannon.left = trigLeft;
    cannon.right = trigRight;
    Serial << "Cannon. left=" << cannon.left << " right=" << cannon.right << endl;
    comms.publish(&cannon);
    
    digitalWrite(BUILTIN_LED, !( cannon.right==TRIGGER_ON || cannon.left==TRIGGER_ON) );
  }

  // do stuff
  stateMachine.update();
}

void switchState(systemState state) {
  switch ( state ) {
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
  switch( state ) {
    case STARTUP: Serial << "STARTUP"; break;  //  all roles start here
  
    case LONELY: Serial << "LONELY"; break;   // 0 players
    case OHAI: Serial << "OHAI"; break;   // 1 players
    case GOODNUF: Serial << "GOODNUF"; break;  // 2 players
    case GOODJOB: Serial << "GOODJOB"; break;  // 3 players or 2 players coordinated
    case WINNING: Serial << "WINNING"; break;  // 3 players and 2 players coordinated
    case FANFARE: Serial << "FANFARE"; break;  // 3 players and 3 players coordinated 
  
    case REBOOT: Serial << "REBOOT"; break;   //  trigger to reboot 
  }
}

void startup() {
}
void lonely() {
}
void ohai() {
}
void goodnuf() {
}
void goodjob() {
}
void winning() {
}
void fanfare() {
}


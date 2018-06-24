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
//NyctRole myRole = WaterPrime; 
//NyctRole myRole = WaterBoost; 

// wire it up
// devices with the light shield have access to D5-D8
const byte pin[4] = {D5, D6, D7, D8};
// also used
// D4, GPIO2, BUILTIN_LED

// current settings
#define OFF LOW
#define ON HIGH
boolean pinState[4] = {OFF, OFF, OFF, OFF};

// comms
NyctComms comms;

// define a state for every systemState
void startup(); State Startup = State(startup);
void lonely(); State Lonely = State(lonely);
void ohai(); State Ohai = State(ohai);
void goodnuf(); State Goodnuf = State(goodnuf);
void goodjob(); State Goodjob = State(goodjob);
void winning(); State Winning = State(winning);
void reboot() { comms.reboot(); }; State Reboot = State(reboot);
FSM stateMachine = FSM(Startup); // initialize state machine

// incoming message storage and flag for update
struct sC_t {
  boolean hasUpdate = false;
  SystemCommand settings;
} sC;
struct cT_t {
  boolean hasUpdate = false;
  CannonTrigger cannon;
} cT;

void applyToHardware() {
  static boolean lastState[4] = {true};
  boolean same = true;
  for( byte i=0; i<4; i++ ) {
    if( lastState[i] != pinState[i] ) same = false; 
  }
  if( same ) return;
  
  Serial << "Settings. Pin";
  for( byte i=0; i<4; i++ ) {
    digitalWrite(pin[i], pinState[i]);
    Serial << " " << i << "=" << pinState[i];
    lastState[i] = pinState[i];
  }
  Serial << endl; 
}

void setup() {
  // set them off, then enable pin.
  applyToHardware();
  for( byte i=0; i<4; i++ ) pinMode(pin[i], OUTPUT);

  // for local output
  Serial.begin(115200);
  Serial << endl << endl << endl << F("Startup begins.") << endl;

  // configure comms
  comms.begin(myRole);
  myRole = comms.getRole();

  // subscribe
  comms.subscribe(&sC.settings, &sC.hasUpdate);
  comms.subscribe(&cT.cannon, &cT.hasUpdate);

  Serial << F("Startup complete.") << endl;
}

void loop() {
  // comms handling
  comms.update();

  // check for settings update
  if ( sC.hasUpdate ) {
    switchState(sC.settings.state);
    sC.hasUpdate = false;
  }

  // check for settings update
  if ( cT.hasUpdate ) {
    Serial << "Cannon. state=" << cT.cannon.state << endl;
    cT.hasUpdate = false;
  }

  // check for serial input
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 's': sC.settings.state = STARTUP; sC.hasUpdate = true; break;
      case 'l': sC.settings.state = LONELY; sC.hasUpdate = true; break;
      case 'o': sC.settings.state = OHAI; sC.hasUpdate = true; break;
      case 'n': sC.settings.state = GOODNUF; sC.hasUpdate = true; break;
      case 'g': sC.settings.state = GOODJOB; sC.hasUpdate = true; break;
      case 'w': sC.settings.state = WINNING; sC.hasUpdate = true; break;
      case 'r': sC.settings.state = REBOOT; sC.hasUpdate = true; break;
      case 't': cT.cannon.state = cT.cannon.state == TRIGGER_ON ? TRIGGER_OFF : TRIGGER_ON; cT.hasUpdate = true; break;
      case '\n': break;
      case '\r': break;
      default:
        Serial << F("(s)tartup, (l)onely, (o)hai, good(n)uf, (g)oodjob, (w)inning, (r)eboot, (t)rigger.") << endl;
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
    case LONELY: stateMachine.transitionTo(Lonely); break;
    case OHAI: stateMachine.transitionTo(Ohai); break;
    case GOODNUF: stateMachine.transitionTo(Goodnuf); break;
    case GOODJOB: stateMachine.transitionTo(Goodjob); break;
    case WINNING: stateMachine.transitionTo(Winning); break;
    case REBOOT: stateMachine.transitionTo(Reboot); break;
    default:
      Serial << F("ERROR!  unknown state.") << endl;
  }
}

void startup() {
  // update with defaults
  applyToHardware();

  // transition to Offline, but we could easily get directed to Online before that.
  stateMachine.transitionTo(Lonely);

}
/*
State      PrimePumps  BoostPumps   Route
Lonely     0           0
Ohai       1           0
Goodnuf    1           1
Goodjob    2           1
Winning    2           2
*/

void lonely() {
  switch (myRole) {
    case WaterRoute:  pinState[0] = duty(0,1);  pinState[1] = ON;   break;
    case WaterPrime:  pinState[0] = OFF;  pinState[1] = OFF;  break;
    case WaterBoost:  pinState[0] = OFF;  pinState[1] = OFF;  break;
  }
  applyToHardware();
}
void ohai() {
  switch (myRole) {
    case WaterRoute:  pinState[0] = duty(0,1);  pinState[1] = ON;   break;
    case WaterPrime:  pinState[0] = ON;   pinState[1] = OFF;  break;
    case WaterBoost:  pinState[0] = OFF;  pinState[1] = OFF;  break;
  }
  applyToHardware();
}
void goodnuf() {
  switch (myRole) {
    case WaterRoute:  pinState[0] = duty(3,7);  pinState[1] = OFF;  break;
    case WaterPrime:  pinState[0] = ON;   pinState[1] = OFF;  break;
    case WaterBoost:  pinState[0] = ON;   pinState[1] = OFF;  break;
  }
  applyToHardware();
}
void goodjob() {
  switch (myRole) {
    case WaterRoute:  pinState[0] = duty(3,2);  pinState[1] = OFF;  break;
    case WaterPrime:  pinState[0] = ON;   pinState[1] = ON;  break;
    case WaterBoost:  pinState[0] = ON;   pinState[1] = OFF;  break;
  }
  applyToHardware();
}
void winning() {
  switch (myRole) {
    case WaterRoute:  pinState[0] = duty(10,0);  pinState[1] = OFF;  break;
    case WaterPrime:  pinState[0] = ON;   pinState[1] = ON;  break;
    case WaterBoost:  pinState[0] = ON;   pinState[1] = ON;  break;
  }
  applyToHardware();
}

boolean duty(uint32_t secOn, uint32_t secOff) {
  static boolean state = false;
  static Metro onTime(1000UL*secOn);
  static Metro offTime(1000UL*secOff);

  // is the trigger squeezed?
  boolean trigger = cT.cannon.state == TRIGGER_ON;
  
  if( !trigger || secOn==0 ) {
    state = false;
    return( state ); // that was easy
  }
  if( trigger && secOff==0 ) {
    state = true;
    return( state ); // that was easy
  }
  
  byte lastOn, lastOff;
  if( secOn != lastOn ) {
    onTime.interval(secOn*1000UL);  
    lastOn = secOn;
  }
  if( secOff != lastOff ) {
    offTime.interval(secOff*1000UL);  
    lastOff = secOff;
  }

  if( state && onTime.check() ) {
    state = false;
    offTime.reset();
    Serial << millis() << " off" << endl;
  } else if( !state && offTime.check() ){
    state = true;
    onTime.reset();
    Serial << millis() << " on" << endl;
  }

  return( state );
}


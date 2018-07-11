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
void fanfare(); State Fanfare = State(fanfare);
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

// try to alternate which pump is mostly used
byte mainPump, subPump;

// track which solenoid is which
const byte cannonSol = 0;
const byte fountainSol = 1;

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

  // for random numbers
  randomSeed(analogRead(0));
  // pick a main pump on startup.  Ideally, we'll swap every startup
  mainPump = constrain(random(2), 0, 1); // 0 or 1
  subPump = 1-mainPump;
  Serial << F("Main pump=") << mainPump << F(" second pump=") << subPump << endl;

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
    Serial << "Cannon. left=" << cT.cannon.left << " right=" << cT.cannon.right << endl;
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
      case 't': 
        cT.cannon.left = cT.cannon.left == TRIGGER_ON ? TRIGGER_OFF : TRIGGER_ON; 
        cT.cannon.right = cT.cannon.right == TRIGGER_ON ? TRIGGER_OFF : TRIGGER_ON; 
        cT.hasUpdate = true; break;
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
 Serial << endl;
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
//    case WaterRoute:  pinState[0] = duty(0,1);  pinState[1] = OFF;   break;
    case WaterRoute:  pinState[cannonSol] = OFF;  pinState[fountainSol] = OFF;  break;

    case WaterPrime:  pinState[mainPump] = OFF;  pinState[subPump] = OFF;  break;
    case WaterBoost:  pinState[mainPump] = OFF;  pinState[subPump] = OFF;  break;
  }
  applyToHardware();
}
void ohai() {
  switch (myRole) {
//    case WaterRoute:  pinState[0] = duty(0,1);  pinState[1] = OFF;   break;
    case WaterRoute:  pinState[cannonSol] = OFF;  pinState[fountainSol] = OFF;  break;
    
    case WaterPrime:  pinState[mainPump] = ON;   pinState[subPump] = OFF;  break;
    case WaterBoost:  pinState[mainPump] = OFF;  pinState[subPump] = OFF;  break;
  }
  applyToHardware();
}
void goodnuf() {
  switch (myRole) {
//    case WaterRoute:  pinState[0] = duty(2,3);  pinState[1] = OFF;  break;
    case WaterRoute:  pinState[cannonSol] = triggerRight();  pinState[fountainSol] = !triggerLeft();  break;
    
    case WaterPrime:  pinState[mainPump] = ON;   pinState[subPump] = OFF;  break;
    case WaterBoost:  pinState[mainPump] = ON;   pinState[subPump] = OFF;  break;
  }
  applyToHardware();
}
void goodjob() {
  switch (myRole) {
//    case WaterRoute:  pinState[0] = duty(2,3);  pinState[1] = ON;  break;
    case WaterRoute:  pinState[cannonSol] = triggerRight();  pinState[fountainSol] = !triggerLeft();  break;
    
    case WaterPrime:  pinState[mainPump] = ON;   pinState[subPump] = ON;  break;
    case WaterBoost:  pinState[mainPump] = ON;   pinState[subPump] = OFF;  break;
  }
  applyToHardware();
}
void winning() {
  switch (myRole) {
//    case WaterRoute:  pinState[0] = duty(10,0);  pinState[1] = ON;  break;
    case WaterRoute:  pinState[cannonSol] = triggerRight();  pinState[fountainSol] = !triggerLeft();  break;
    
    case WaterPrime:  pinState[mainPump] = ON;   pinState[subPump] = ON;  break;
    case WaterBoost:  pinState[mainPump] = ON;   pinState[subPump] = ON;  break;
  }
  applyToHardware();
}

void fanfare() {
  winning();
}

boolean triggerRight() {
  return( cT.cannon.right == TRIGGER_ON );
}
boolean triggerLeft() {
  return( cT.cannon.left == TRIGGER_ON );
}

boolean duty(uint32_t timeOn, uint32_t secOff) {
  const uint32_t mult = 1000UL;
  static boolean state = false;
  static Metro onTime(mult*timeOn);
  static Metro offTime(mult*secOff);

  // is the trigger squeezed?
  boolean trigger = 
    cT.cannon.left == TRIGGER_ON ||
    cT.cannon.right == TRIGGER_ON
  ;
  
  if( !trigger || timeOn==0 ) {
    state = false;
    return( state ); // that was easy
  }
  if( trigger && secOff==0 ) {
    state = true;
    return( state ); // that was easy
  }
  
  byte lastOn, lastOff;
  if( timeOn != lastOn ) {
    onTime.interval(timeOn*mult);  
    lastOn = timeOn;
  }
  if( secOff != lastOff ) {
    offTime.interval(secOff*mult);  
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


/*
 
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

*/

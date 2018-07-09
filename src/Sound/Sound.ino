// IDE Settings:
// Tools->Board : "WeMos D1 R2 & mini"
// Tools->Flash Size : "4M (3M SPIFFS)"
// Tools->CPU Frequency : "160 MHz"
// Tools->Upload Speed : "921600"

#include <Streaming.h>
#include <Metro.h>
#include <SoftwareSerial.h>
#include <FiniteStateMachine.h>
#include <Tsunami.h>
#include "Nyctinasty_Messages.h"
#include "Nyctinasty_Comms.h"

// spammy?
#define SHOW_SERIAL_DEBUG true

// my role and arch number
NyctRole myRole = Sound; // see Nyctinasty_Comms.h; set N_ROLES to pull from EEPROM

// wire it up
// RX/TX for softserial comms
#define RX D1 // GPIO5.  wire to Tsunami TX0
#define TX D2 // GPIO4.  wire to Tsunami RX1
// talk to the Sound device device
SoftwareSerial mySerial(RX, TX, false, 1024);

// also used
// D4, GPIO2, BUILTIN_LED
#define LED BUILTIN_LED // our LED

// comms
NyctComms comms;

// define a state for every systemState
void startup(); State Startup = State(startup);

// Gameplay states
void lonelyEnter(); void lonely(); void lonelyExit(); State Lonely = State(((byte)LONELY),lonelyEnter,lonely,lonelyExit);
void ohaiEnter(); void ohai(); void ohaiExit(); State Ohai = State(((byte)OHAI),ohaiEnter,ohai,ohaiExit);
void goodnufEnter();  void goodnuf();  void goodnufExit(); State Goodnuf = State((byte)GOODNUF,goodnufEnter,goodnuf,goodnufExit);
void goodjobEnter(); void goodjob(); void goodjobExit();  State Goodjob = State((byte)GOODJOB,goodjobEnter,goodjob,goodjobExit);
void winningEnter(); void winning(); void winningExit(); State Winning = State((byte)WINNING,winningEnter,winning,winningExit);
void fanfareEnter(); void fanfare(); void fanfareExit(); State Fanfare = State((byte)FANFARE,fanfareEnter,fanfare,fanfareExit);

// Sounds
#define LONELY_START 100
#define OHAI_START 200
#define GOODNUF_START 400
#define GOODJOB_START 500
#define WINNING_START 600
#define FANFARE_START 300
#define BEAT_START 700
#define POS_CHANGE_START 800
#define NEG_CHANGE_START 900
#define GUNFIRE_START 950
#define PALETTE_START 1000  // Palettes are 6 track blocks of related sounds(ohai,goodnuf,goodjob,winning,.pos change,neg change)

#define LONELY_NUM 10
#define OHAI_NUM 14
#define FANFARE_NUM 69
#define GOODNUF_NUM 7  // 1
#define GOODJOB_NUM 4  // 2
#define WINNING_NUM 3  // 1
#define BEAT_NUM 30
#define POS_CHANGE_NUM 18
#define NEG_CHANGE_NUM 22
#define GUNFIRE_NUM 11
#define PALETTE_NUM 0

int COOR_START[(byte)N_STATES] = {0,0,OHAI_START,GOODNUF_START,GOODJOB_START,WINNING_START};
int COOR_NUM[(byte)N_STATES] = {0,0,OHAI_NUM,GOODNUF_NUM,GOODJOB_NUM,WINNING_NUM};

State & lastState = Startup;
int beatTrack;
int corrTrack[N_STATES] = {-1,-1,-1,-1,-1,-1};
int lonelyTrack = -1;
int fanefareTrack = -1;
int corrChangedTrack = -1;
int cannonTrack = -1;

#define LONELY_PLAY_SPACING 180000l
#define BEAT_CHANGE_SPACING 60000l

Metro lonelyTimer(1UL);
Metro ohaiLockoutTimer(1UL);
Metro beatChangeTimer(1UL);
Metro coorTimer(2000UL);
Metro gunfireLockout(200UL);


#define SINGLE_TRACK_DB -5
#define TWO_TRACKS_DB -10

void reboot() {
  comms.reboot();
}; State Reboot = State(reboot);
void reprogram() {
  comms.reprogram("Sepal_Sound.ino.bin");
}; State Reprogram = State(reprogram);
FSM stateMachine = FSM(Startup); // initialize state machine

// incoming message storage and flag for update
struct sC_t {
  boolean hasUpdate = false;
  SystemCommand settings;
} sC;

// distance updates receive as this structure as this topic
typedef struct {
  boolean hasUpdate = false;
  SepalArchDistance dist;
} sAD_t;
sAD_t sAD[N_ARCH];

// frequency updates receive as this structure as this topic
typedef struct {
  boolean hasUpdate = false;
  SepalArchFrequency freq;
} sAF_t;
sAF_t sAF[N_ARCH];

struct cT_t {
  boolean hasUpdate = false;
  CannonTrigger cannon;
} cT;

// Our Tsunami object
Tsunami tsunami;

// some tracks to mess with
long lastActivity = 0;  // Last time we detected movement



void setup() {
  // for local output
  Serial.begin(115200);
  Serial << endl << endl << endl << F("Startup begins.") << endl;

  delay(1000);

  // for remote output
  Serial << F("Configure Tsunami...");
  // Tsunami startup at 57600
  tsunami.start();
  delay(10);

  // Send a stop-all command and reset the sample-rate offset, in case we have
  //  reset while the Tsunami was already playing.
  tsunami.stopAllTracks();
  tsunami.samplerateOffset(0, 0);

  // Enable track reporting from the Tsunami
  tsunami.setReporting(true);

  Serial << F(" done.") << endl;

  delay(1000);

  // configure comms
  comms.begin(myRole);
  myRole = comms.getRole();

  // subscribe
  comms.subscribe(&sC.settings, &sC.hasUpdate);
  comms.subscribe(&cT.cannon, &cT.hasUpdate);

  for ( byte i = 0; i < N_ARCH; i++ ) {
    Serial.printf("Subscribe to arch: %d\n",i);
    // subscribe to frequency and distance messages
    comms.subscribe(&sAF[i].freq, &sAF[i].hasUpdate, i);
    comms.subscribe(&sAD[i].dist, &sAD[i].hasUpdate, i);
  }

  Serial << F("Setup complete. Wait for subscriptions") << endl;

  Serial << "Done with Setup" << endl;
  // Allow time for subscriptions to land
  //delay(5000);  // Not sure if this is really necessary but we dont always connect 

  corrTrack[0] = -1;
  corrTrack[1] = -1;
  corrTrack[2] = -1;
  corrTrack[4] = -1;
  corrTrack[5] = -1;
  corrTrack[6] = -1;
}

void loop() {
  // comms handling
  comms.update();
  tsunami.update();


  // check for settings update
  if ( sC.hasUpdate ) {
    // TODO: Turned off for local testing
    switchState(sC.settings.state);
    sC.hasUpdate = false;
  }

  // do stuff
  stateMachine.update();

  //mimicController();  // Uncomment to mimic the controller
  //testStates();  // Uncomment to run through all the states. Comment out switchState call above

  updateBeat();  // Periodically change the beat while playing

  updateCannon();
  // TODO: Add small delay here to allow interrupts to fire?
  delay(1);
}

void switchState(systemState state) {
  Serial << F("State.  Changing to ");

  lastState = stateMachine.getCurrentState();

  switch ( state ) {
    case STARTUP: Serial << "Startup" << endl; stateMachine.transitionTo(Startup); break;
    //case SLAVED: stateMachine.transitionTo(Slaved); break;
    case REBOOT: stateMachine.transitionTo(Reboot); break;
    //case REPROGRAM: stateMachine.transitionTo(Reprogram); break;
    
    case LONELY: Serial << "Lonely" << endl; stateMachine.transitionTo(Lonely); break;
    case OHAI: Serial << "Ohai" << endl; stateMachine.transitionTo(Ohai); break;
    case GOODNUF: Serial << "Goodnuf" << endl; stateMachine.transitionTo(Goodnuf); break;
    case GOODJOB: Serial << "Goodjob" << endl; stateMachine.transitionTo(Goodjob); break;
    case WINNING: Serial << "Winning" << endl; stateMachine.transitionTo(Winning); break;
    case FANFARE: Serial << "Fanfare" << endl; stateMachine.transitionTo(Fanfare); break;
    default:
      Serial << F("ERROR!  unknown state.") << endl;
  }

}

// Test states by running through each
void testStates() {
  static Metro stateChange(15000UL);

  if (!stateChange.check()) return;

  //lastState = stateMachine.getCurrentState();  
  
  if (stateMachine.isInState(Startup)) {
    lastState = Startup;
    Serial << "Changing test state" << " lastState: " << convStateToEnum(lastState) << endl;
    Serial << "Changing to Lonely" << endl;
    stateMachine.transitionTo(Lonely);
  }
  
  if (stateMachine.isInState(Lonely)) {
    lastState = Lonely;
    Serial << "Changing test state" << " lastState: " << convStateToEnum(lastState) << endl;
    Serial << "Changing to Ohai" << endl;
    stateMachine.transitionTo(Ohai);
  }

  if (stateMachine.isInState(Ohai)) {
    lastState = Ohai;
    Serial << "Changing test state" << " lastState: " << convStateToEnum(lastState) << endl;
    Serial << "Changing to Goodnuf" << endl;
    stateMachine.transitionTo(Goodnuf);
  }

  if (stateMachine.isInState(Goodnuf)) {
    lastState = Goodnuf;
    Serial << "Changing test state" << " lastState: " << convStateToEnum(lastState) << endl;
    Serial << "Changing to GoodJob" << endl;
    stateMachine.transitionTo(Goodjob);
  }

  if (stateMachine.isInState(Goodjob)) {
    lastState = Goodjob;
    
    Serial << "Changing test state" << " lastState: " << convStateToEnum(lastState) << endl;
    Serial << "Changing to Winning" << endl;
    stateMachine.transitionTo(Winning);
  }

  if (stateMachine.isInState(Winning)) {
    lastState = Winning;
    Serial << "Changing to Fanfare" << endl;
    stateMachine.transitionTo(Fanfare);
  }

  if (stateMachine.isInState(Fanfare)) {
    Serial << "Changing to Lonely" << endl;
    stateMachine.transitionTo(Lonely);
  }

  stateChange.reset();
}

// Mimic the controller by handling state transforms
void mimicController() {
  static Metro ohaiTimeout(4500UL);
  static Metro corrTimeout(30000UL);
  static Metro fanfareTimeout(30000UL);
  
  /*
  if (stateMachine.isInState(Online)) {
    if (checkForLonely()) {
      if (millis() - lastActivity > LONELY_TIME) {
        Serial << "Moving to Lonely state from Online" << endl;
        stateMachine.transitionTo(Lonely);
        return;
      }
    } else {
      Serial << "Moving to Ohai state" << endl;
      ohaiTimeout.reset();
      stateMachine.transitionTo(Ohai);
      return;
    }
  }
*/
  if (stateMachine.isInState(Lonely)) {
    if (!checkForLonely()) {
      Serial << "Moving to Ohai state" << endl;
      ohaiTimeout.reset();
      stateMachine.transitionTo(Ohai);
      return;
    }

    return;
  }

  if (stateMachine.isInState(Ohai)) {
      //erial << "Checking intro timeout" << endl;
      if ( ohaiTimeout.check() ) {
        Serial << "Ohai time done" << endl;
        corrTimeout.reset();
        lastActivity = millis();
        stateMachine.transitionTo(Goodnuf);
        return;
      }
  }
  
  if (stateMachine.isInState(Goodnuf) || stateMachine.isInState(Goodjob) || stateMachine.isInState(Winning)) {
    /*
    if (checkForIdle()) {
      if (millis() - lastActivity > IDLE_TIME) {
        Serial << "Moving to idle state from c*" << endl;
        stateMachine.transitionTo(Idle);
        return;
      }
    }
    */

    if ( corrTimeout.check() ) {
      Serial << "Correspondence Progression" << endl;
      if (stateMachine.isInState(Goodnuf)) {
        Serial << "Moving to GoodJob" << endl;
        corrTimeout.reset();
        lastActivity = millis();
        stateMachine.transitionTo(Goodjob);
        return;
      }
      if (stateMachine.isInState(Goodjob)) {
        Serial << "Moving to Winning" << endl;
        corrTimeout.reset();
        lastActivity = millis();
        stateMachine.transitionTo(Winning);
        return;
      }
      if (stateMachine.isInState(Winning)) {
        Serial << "Moving to Fanfare" << endl;
        corrTimeout.reset();
        fanfareTimeout.reset();
        lastActivity = millis();
        stateMachine.transitionTo(Fanfare);
        return;
      }
    }
  }

  if (stateMachine.isInState(Fanfare)) {
      if ( fanfareTimeout.check() ) {
        Serial << "Win time done" << endl;
        stateMachine.transitionTo(Lonely);
        return;
      }
  }

}

boolean checkForLonely() {
   int detected = 0;
   long totPower = 0;
   for ( byte up = 0; up < N_ARCH; up++ ) {
      if ( sAD[up].hasUpdate ) {
        // we have an update to distance information in sAD[i].dist
  
        // make some noise; simple treshold-based detection of distance-closer-than
        uint16_t thresh = sAD[up].dist.max / 4;
        for ( byte j = 0; j < N_SENSOR; j++ ) {
          if ( sAD[up].dist.prox[j] > thresh ) {
            detected++;
          }
          totPower += sAD[up].dist.prox[j];
        }
  
        // note that we've handled the update already
        sAD[up].hasUpdate = false;
      }
  }

/* . // This generates a fair bit of false positives
  if (detected > 1) {
    Serial << "Detected: " << detected << endl;
    lastActivity = millis();
    return false;
  }
*/
  if (detected > 1 && totPower > sAD[0].dist.max) {  // TODO: Smells like a magic number, might change in the field
    Serial << "Detected: " << detected << " power: " << totPower << endl;
    lastActivity = millis();
    return false;
  }

  return true;
}

void updateCannon() {
  if ( cT.hasUpdate ) {
    //Serial << " millis: " << millis() << endl;
    //Serial << "Cannon. left=" << cT.cannon.left << " right=" << cT.cannon.right << endl;
    if (cT.cannon.left || cT.cannon.right) {
      Serial << "Cannon triggered" << endl;
      // TODO: Restore this for the real game
      
      //if (!(stateMachine.isInState(Goodnuf) || stateMachine.isInState(Goodjob) || stateMachine.isInState(Winning) || stateMachine.isInState(Fanfare))) return; 
      //Serial << "State is go" << endl;
//      if (!tsunami.isTrackPlaying(cannonTrack)) {
      if (cannonTrack == -1 && gunfireLockout.check()) {
        cannonTrack = random(GUNFIRE_START, GUNFIRE_START+GUNFIRE_NUM);
        Serial << "Playing cannon sound: " << cannonTrack <<  endl;
 
        tsunami.trackGain(cannonTrack,0);   
        tsunami.trackLoop(cannonTrack,true); 
        tsunami.trackPlayPoly(cannonTrack, 0, true);   
        gunfireLockout.reset();
        delay(50);
      } 
    }else {
      if (cannonTrack != -1) {
        Serial << "Turning off cannon: " << cannonTrack << endl;
        tsunami.trackStop(cannonTrack);
        cannonTrack = -1;        
        delay(50);
      }
    }
    cT.hasUpdate = false;
  }
}
  
void updateBeat() {
    if (!beatChangeTimer.check()) return;
    
    if (stateMachine.isInState(Ohai) || stateMachine.isInState(Goodnuf) || stateMachine.isInState(Goodjob) || stateMachine.isInState(Winning)) {
      // Were playing, lets change the beat
      playNewBeat();
      beatChangeTimer.interval(BEAT_CHANGE_SPACING);
      beatChangeTimer.reset();  
    }
}

void playNewBeat() {
    if (BEAT_NUM <= 0) return;

    tsunami.trackStop(beatTrack);
    beatTrack = random(BEAT_START, BEAT_START+BEAT_NUM);
    Serial << "Playing beat track: " << beatTrack << endl;

    tsunami.trackGain(beatTrack,TWO_TRACKS_DB);   
    tsunami.trackLoop(beatTrack,true);    
    tsunami.trackPlayPoly(beatTrack, 0, true);
}

void stopBeat() {
    if (beatTrack > 0) {
      tsunami.trackStop(beatTrack);
      beatTrack = -1;
    }
}

void changeCorrEntry(systemState currState, systemState newState, boolean loop) {
  Serial << "changeCorr.  newState: " << ((byte)newState) << " currState: " << ((byte)currState) << endl;
  if (corrChangedTrack != -1) {
    tsunami.trackStop(corrChangedTrack);  // Stop the old corr change track
    corrChangedTrack = -1;
  }
  
  if (!loop) {
    if(COOR_NUM[(byte)newState] <= 0) return;

    corrTrack[(byte)newState] = random(COOR_START[(byte)newState], COOR_START[(byte)newState]+COOR_NUM[(byte)newState]);
    Serial << "Playing coor track: " << corrTrack[(byte)newState] << " state: " << ((byte)newState) << endl;
    tsunami.trackGain(corrTrack[(byte)newState],TWO_TRACKS_DB);   
    tsunami.trackLoop(corrTrack[(byte)newState],loop);    
    tsunami.trackPlayPoly(corrTrack[(byte)newState], 0, true); 
    
    delay(200);  // Delay enough so that isPlaying check will work
    tsunami.update();  // TODO: Shouldnt need
  } else {
    if ((byte)newState > (byte)currState) {
      if (corrChangedTrack > -1) tsunami.trackStop(corrChangedTrack);
    
      // play pos state change   
      corrChangedTrack = random(POS_CHANGE_START, POS_CHANGE_START+POS_CHANGE_NUM);
      Serial << "Playing pos corr changed track: " << corrChangedTrack << endl;
  
      tsunami.trackGain(corrChangedTrack,TWO_TRACKS_DB);   
      tsunami.trackLoop(corrChangedTrack,false);    
      tsunami.trackPlayPoly(corrChangedTrack, 0, true);
      delay(200);
    } else {
      if (corrChangedTrack > -1) tsunami.trackStop(corrChangedTrack);
    
      // play neg state change   
      corrChangedTrack = random(NEG_CHANGE_START, NEG_CHANGE_START+NEG_CHANGE_NUM);
      Serial << "Playing neg corr changed track: " << corrChangedTrack << endl;
  
      tsunami.trackGain(corrChangedTrack,TWO_TRACKS_DB);   
      tsunami.trackLoop(corrChangedTrack,false);    
      tsunami.trackPlayPoly(corrChangedTrack, 0, true);    
      delay(200);
    }
  }
}

void changeCorrUpdate(systemState state,boolean loop) {
  if (tsunami.isTrackPlaying(corrTrack[(byte)state])) return;
  if (tsunami.isTrackPlaying(corrChangedTrack)) {
    coorTimer.reset();
    return;
  }

  if (!coorTimer.check()) return;  // wait a bit after track ends
  
  if (loop) {
    if(COOR_NUM[(byte)state] <= 0) return;
    corrTrack[(byte)state] = random(COOR_START[(byte)state], COOR_START[(byte)state]+COOR_NUM[(byte)state]);
    Serial << "Playing coor track: " << corrTrack[(byte)state] << " state: " << ((byte)state) << endl;
    tsunami.trackGain(corrTrack[(byte)state],TWO_TRACKS_DB);   
    tsunami.trackLoop(corrTrack[(byte)state],loop);    
    tsunami.trackPlayPoly(corrTrack[(byte)state], 0, true); 
    
    delay(200);  // Delay enough so that isPlaying check will work
    tsunami.update();  // TODO: Shouldnt need
  }
}

void changeCorrExit(systemState state) {  
  Serial << "Stopping coor track: " << corrTrack[(byte)state] << " state: " << ((byte)state) << endl;
  tsunami.trackStop(corrTrack[(byte)state]);
}

void lonelyEnter() {
    tsunami.stopAllTracks();  // Keep this here as safe guard cleanup

    lonelyTrack = random(LONELY_START, LONELY_START+LONELY_NUM);
    Serial << "Playing lonely track: " << lonelyTrack << endl;

    tsunami.trackGain(lonelyTrack,SINGLE_TRACK_DB);   
    tsunami.trackLoop(lonelyTrack,false);   
    tsunami.trackPlayPoly(lonelyTrack, 0, true);   
    lonelyTimer.interval(LONELY_PLAY_SPACING);
    lonelyTimer.reset();
}

// The game is idle waiting for a user.  Mix in line in with attract sounds.  Randomly plays track idle_* every few minutes
void lonely() {
    if (!lonelyTimer.check()) return;

    tsunami.trackStop(lonelyTrack);
    
    lonelyTrack = random(LONELY_START, LONELY_START+LONELY_NUM);
    Serial << "Playing new lonely track: " << lonelyTrack << endl;

    tsunami.trackGain(lonelyTrack,SINGLE_TRACK_DB);   
    tsunami.trackPlayPoly(lonelyTrack, 0, true);   
    tsunami.trackLoop(lonelyTrack,false);   
    lonelyTimer.reset();
 }

 void lonelyExit() {
  Serial << "Stoping lonely track" << endl;
  tsunami.trackStop(lonelyTrack);   
 }

void ohaiEnter() {
  Serial << "Ohai enter" << " lastState: " << convStateToEnum(lastState) << endl;

  // Pick a new cannon track
  if (cannonTrack != -1) {
    tsunami.trackStop(cannonTrack);
  }
  //cannonTrack = random(GUNFIRE_START, GUNFIRE_START+GUNFIRE_NUM);
  changeCorrEntry((systemState)lastState.getId(), OHAI,false);
}
void ohai() {
  changeCorrUpdate(convStateToEnum(stateMachine.getCurrentState()),false);
}
void ohaiExit() {
  changeCorrExit(convStateToEnum(stateMachine.getCurrentState()));
}

void goodnufEnter() {
  Serial << "Goodnuf enter" << " lastState: " << convStateToEnum(lastState) << endl;
  changeCorrEntry((systemState)lastState.getId(), GOODNUF,false);
}
void goodnuf() {
  changeCorrUpdate(convStateToEnum(stateMachine.getCurrentState()),false);
}
void goodnufExit() {
  changeCorrExit(convStateToEnum(stateMachine.getCurrentState()));
}

void goodjobEnter() {
    Serial << "Goodjob enter" << " lastState: " << convStateToEnum(lastState) << endl;

  changeCorrEntry((systemState)lastState.getId(), GOODJOB,false);
}
void goodjob() {
  changeCorrUpdate(convStateToEnum(stateMachine.getCurrentState()),false);
}
void goodjobExit() {
  changeCorrExit(convStateToEnum(stateMachine.getCurrentState()));
}

void winningEnter() {
    Serial << "Winning enter" << " lastState: " << convStateToEnum(lastState) << endl;
  changeCorrEntry((systemState)lastState.getId(), WINNING,false);
}
void winning() {
  changeCorrUpdate(convStateToEnum(stateMachine.getCurrentState()),false);
}
void winningExit() {
  changeCorrExit(convStateToEnum(stateMachine.getCurrentState()));
}


void fanfareEnter() {
    stopBeat();

    // TODO: Add a definitive win sound
}

// Player reached the win condition.  Play a long(30s) win track
void fanfare() {
    static boolean playing = false;
    static int track;

    if (!playing) {
      tsunami.stopAllTracks();

      track = random(FANFARE_START, FANFARE_START+FANFARE_NUM);
      Serial << "Playing winning track: " << track << endl;
      tsunami.trackGain(track,SINGLE_TRACK_DB);   
      tsunami.trackPlayPoly(track, 0, true); 
      delay(200);
      tsunami.update();
      playing = true;
    } else {  
      if (!tsunami.isTrackPlaying(track)) {
        Serial << "Winning track done" << endl;
        tsunami.trackStop(track);
        tsunami.update();
        playing = false;
      }  
    }
}

void fanfareExit() {
}

// Play a sound at startup to confirm working hardware
void startup() {
    static boolean startupPlaying = false;
    static int startupTrack;

    if (!startupPlaying) {
      Serial << "Start Startup" << endl;
      startupTrack = FANFARE_START;
      tsunami.trackGain(startupTrack,SINGLE_TRACK_DB);   
      tsunami.trackPlayPoly(startupTrack, 0, true); 
      tsunami.update();
      startupPlaying = true;
      lastActivity = millis();    

      delay(100);
    } else {  
      static Metro startupTimeout(8000UL);
      if ( startupTimeout.check() ) {
        Serial << "Startup track done" << endl;
        tsunami.trackStop(startupTrack);
        tsunami.update();

        startupPlaying = false;
        stateMachine.transitionTo(Lonely);
      }
    }
}

void slaved() {
  // NOP, currently.  Will look for lighting directions, either through UDP (direct) or MQTT (procedural).
}

systemState convStateToEnum(const State & state) {
  if (&state == &Ohai) {
    return OHAI;
  }
  
  if (&state == &Goodnuf) {
    return GOODNUF;
  }

  if (&state == &Goodjob) {
    return GOODJOB;
  }

  if (&state == &Winning) {
    return WINNING;
  }

  if (&state == &Startup) {
    return STARTUP;
  }

  if (&state == &Fanfare) {
    return FANFARE;
  }

  Serial << "Unknown state" << endl;
  return WINNING;
}


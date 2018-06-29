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
//void offline(); State Offline = State(offline);
//void online(); State Online = State(online);
//void slaved(); State Slaved = State(slaved);

// Gameplay states
void lonelyEnter(); void lonely(); void lonelyExit(); State Lonely = State(lonelyEnter,lonely,lonelyExit);
void ohaiEnter(); void ohaiExit(); State Ohai = State(ohaiEnter,NULL,ohaiExit);
void goodnuf();  State Goodnuf = State(goodnuf);
void goodjob();  State Goodjob = State(goodjob);
void winning();  State Winning = State(winning);
void fanfare(); State Fanfare = State(fanfare);

// Sounds
#define LONELY_START 100
#define OHAI_START 200
#define GOODNUF_START 400
#define GOODJOB_START 500
#define WINNING_START 600
#define FANFARE_START 300

#define LONELY_NUM 8
#define OHAI_NUM 4
#define GOODNUF_NUM 1
#define GOODJOB_NUM 1
#define WINNING_NUM 1
#define FANFARE_NUM 4

#define LONELY_PLAY_SPACING 20000l

Metro lonelyTimer(1UL);
Metro ohaiLockoutTimer(1UL);



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

// there will be a correspondence structure getting sent.  just don't know what it is, yet.

// Our Tsunami object
Tsunami tsunami;

// some tracks to mess with
const int BOOTS = 17;
const int SPARKLE = 18;
const int CATS = 19;

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
  for ( byte i = 0; i < N_ARCH; i++ ) {
    Serial.printf("Subscribe to arch: %d\n",i);
    // subscribe to frequency and distance messages
    comms.subscribe(&sAF[i].freq, &sAF[i].hasUpdate, i);
    comms.subscribe(&sAD[i].dist, &sAD[i].hasUpdate, i);
  }

  Serial << F("Setup complete. Wait for subscriptions") << endl;
/*
  long start = millis();
  while(millis() - start < 5000) {
    delay(10);
    comms.update();
  }
*/
  Serial << "Done with Setup" << endl;
  // Allow time for subscriptions to land
  //delay(5000);  // Not sure if this is really necessary but we dont always connect 

/*
  // run through some calculations to explain what's in the SepalArchFrequency data item.
  Serial << F("Actual frequencies in each power bin:") << endl;
  for(byte j=0; j<N_FREQ_BINS; j++) {
    Serial << F("Bin=") << j;
    Serial << F("\tFreq=") << ((float)j+1.0)*(float)DISTANCE_SAMPLING_FREQ/(float)N_FREQ_SAMPLES;
    Serial << endl;
  }
*/
}

void loop() {
  // comms handling
  comms.update();
  tsunami.update();


  // check for settings update
  if ( sC.hasUpdate ) {
    switchState(sC.settings.state);
    sC.hasUpdate = false;
  }

  // do stuff
  stateMachine.update();

  //mimicController();  // Uncomment to mimic the controller
  //testStates();  // Uncomment to run through all the states
  
  // TODO: Add small delay here to allow interrupts to fire?
  delay(1);
}

void switchState(systemState state) {
  Serial << F("State.  Changing to ");
  switch ( state ) {
    case STARTUP: Serial << "Startup" << endl; stateMachine.transitionTo(Startup); break;
    //case OFFLINE: stateMachine.transitionTo(Offline); break;
    //case ONLINE: stateMachine.transitionTo(Online); break;
    //case SLAVED: stateMachine.transitionTo(Slaved); break;
    //case REBOOT: stateMachine.transitionTo(Reboot); break;
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

  Serial << "Changing test state" << endl;
  
  if (stateMachine.isInState(Startup)) {
    Serial << "Changing to Lonely" << endl;
    stateMachine.transitionTo(Lonely);
  }
  
  if (stateMachine.isInState(Lonely)) {
    Serial << "Changing to OHAI" << endl;
    stateMachine.transitionTo(Ohai);
  }

  if (stateMachine.isInState(Ohai)) {
    Serial << "Changing to GOODNUF" << endl;
    stateMachine.transitionTo(Goodnuf);
  }

  if (stateMachine.isInState(Goodnuf)) {
    Serial << "Changing to GoodJob" << endl;
    stateMachine.transitionTo(Goodjob);
  }

  if (stateMachine.isInState(Goodjob)) {
    Serial << "Changing to Winning" << endl;
    stateMachine.transitionTo(Winning);
  }

  if (stateMachine.isInState(Winning)) {
    Serial << "Changing to Fanfare" << endl;
    stateMachine.transitionTo(Fanfare);
  }

  if (stateMachine.isInState(Fanfare)) {
    Serial << "Changing to Idle" << endl;
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


int lonelyTrack;
void lonelyEnter() {
    tsunami.stopAllTracks();

    lonelyTrack = random(LONELY_START, LONELY_START+LONELY_NUM);
    Serial << "Playing idle track: " << lonelyTrack << endl;

    tsunami.trackGain(lonelyTrack,0);   
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

    tsunami.trackGain(lonelyTrack,0);   
    tsunami.trackPlayPoly(lonelyTrack, 0, true);   
    lonelyTimer.reset();
 }

 void lonelyExit() {
  Serial << "Stoping lonely track" << endl;
  tsunami.trackStop(lonelyTrack);   
 }

int ohaiTrack;

void ohaiEnter() {
    tsunami.stopAllTracks();

    ohaiTrack = random(OHAI_START, OHAI_START+OHAI_NUM);
    Serial << "Playing ohai track: " << ohaiTrack << endl;
    tsunami.trackGain(ohaiTrack,0);   
    tsunami.trackPlayPoly(ohaiTrack, 0, true); 
    delay(200);
    tsunami.update();  
}

void ohaiExit() {
  //Serial << "Stoping intro track" << endl;
  //tsunami.trackStop(ohaiTrack);  
}



void goodnuf() {
  static int track = 0;

  if (!tsunami.isTrackPlaying(track)) {
    track = random(GOODNUF_START, GOODNUF_START+GOODNUF_NUM);
    Serial << "Playing goodnuf track: " << track << endl;
    //tsunami.stopAllTracks();  // Goodnuf seems to come on top of ohai.  Let ohai finish.
    tsunami.trackGain(track,0);   
    tsunami.trackLoop(track,true);   
    tsunami.trackPlayPoly(track, 0, true); 
    delay(200); // Allow for track startup time
  }
}

void goodjob() {
  static int track = 0;

  if (!tsunami.isTrackPlaying(track)) {
    track = random(GOODJOB_START, GOODJOB_START+GOODJOB_NUM);
    Serial << "Playing goodjob track: " << track << endl;
    tsunami.stopAllTracks();
    tsunami.trackGain(track,0);   
    tsunami.trackLoop(track,true);   
    tsunami.trackPlayPoly(track, 0, true); 
    delay(200);
  }
}

void winning() {
  static int track = 0;

  if (!tsunami.isTrackPlaying(track)) {
    track = random(WINNING_START, WINNING_START+WINNING_NUM);
    Serial << "Playing winning track: " << track << endl;
    tsunami.stopAllTracks();
    tsunami.trackGain(track,0);   
    tsunami.trackLoop(track,true);   
    tsunami.trackPlayPoly(track, 0, true); 
    delay(200);
  }
}

// Player reached the win condition.  Play a long(30s) win track
void fanfare() {
    static boolean playing = false;
    static int track;

    if (!playing) {
      tsunami.stopAllTracks();

      track = random(FANFARE_START, FANFARE_START+FANFARE_NUM);
      Serial << "Playing winning track: " << track << endl;
      tsunami.trackGain(track,0);   
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


// Play a sound at startup to confirm working hardware
void startup() {
    static boolean startupPlaying = false;
    static int startupTrack;

    if (!startupPlaying) {
      Serial << "Start Startup" << endl;
      startupTrack = FANFARE_START;
      tsunami.trackGain(startupTrack,0);   
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

/*
void offline() {
  if ( comms.isConnected() ) {
    Serial << F("GOOD.  online!") << endl;
    lastActivity = millis();
    stateMachine.transitionTo(Online);
  }
}

void online() {
  if ( comms.isConnected() ) {
  } else {
    Serial << F("WARNING.  offline!") << endl;
    stateMachine.transitionTo(Offline);
  }
}
*/

/*      
  // loop across arches
  for ( byte up = 0; up < N_ARCH; up++ ) {
    if ( sAF[up].hasUpdate ) {
      // we have an update to frequency information in sAF[i].freq

      // make some noise
      // crappy
      uint16_t avgPower[N_SENSOR] = {0};
      // frequency power
      uint16_t power[N_SENSOR][N_FREQ_BINS] = {{0.0}};

      // do the boneheaded thing and sum up the bins across all sensors
      uint32_t sumSensors[N_FREQ_BINS] = {0};
      uint32_t maxSum = 0;
      for ( uint16_t j = 0; j < N_FREQ_BINS ; j++ ) {
        for ( byte i = 0; i < N_SENSOR; i++ ) {
          sumSensors[j] += sAF[up].freq.power[i][j];
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
        //    leftDown[j] = CHSV(archHue[myArch], archSat[myArch], brighten8_video(constrain(value, 0, 255)));
      }
      Serial << endl;
      
      // note that we've handled the update already
      sAF[up].hasUpdate = false;
    }
    if ( sAD[up].hasUpdate ) {
      // we have an update to distance information in sAD[i].dist

      // make some noise; simple treshold-based detection of distance-closer-than
      uint16_t thresh = sAD[up].dist.max >> 1; // div2
      for ( byte j = 0; j < N_SENSOR; j++ ) {
        if ( sAD[up].dist.prox[j] > thresh ) {
          tsunami.trackPlayPoly(j + 1, 0, false);
        }
      }

      // note that we've handled the update already
      sAD[up].hasUpdate = false;
    }
  }
  */

void slaved() {
  // NOP, currently.  Will look for lighting directions, either through UDP (direct) or MQTT (procedural).
}



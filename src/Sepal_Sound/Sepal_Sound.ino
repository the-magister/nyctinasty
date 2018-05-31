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
#define RX D1 // GPIO5
#define TX D2 // GPIO4
// talk to the Sound device device
SoftwareSerial mySerial(RX, TX, false, 1024);

// also used
// D4, GPIO2, BUILTIN_LED
#define LED BUILTIN_LED // our LED

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
    // subscribe to frequency and distance messages
    comms.subscribe(&sAF[i].freq, &sAF[i].hasUpdate, i);
    comms.subscribe(&sAD[i].dist, &sAD[i].hasUpdate, i);
  }
  
  Serial << F("Startup complete.") << endl;

  // Allow time for the Tsunami to respond with the version string and
  //  number of tracks.
  delay(100);

}

void loop() {
  // comms handling
  comms.update();

  // check for settings update
  if ( sC.hasUpdate ) {
    switchState(sC.settings.state);
    sC.hasUpdate = false;
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

// DANNE, replace this with a "sound check" at startup, then proceed to state==Offline
void startup() {

  // From TsunamiDemo.ino:
  int i;
  static Metro gLedMetro(500);           // LED blink interval timer
  static Metro gSeqMetro(6000);          // Sequencer state machine interval timer

  static byte gLedState = 0;             // LED State
  static int  gSeqState = 0;             // Main program sequencer state
  static int  gRateOffset = 0;           // Tsunami sample-rate offset
  static int  gNumTracks;                // Number of tracks on SD card

  static char gTsunamiVersion[VERSION_STRING_LEN];    // Tsunami version string

  // Call update on the Tsunami to keep the track playing status current.
  tsunami.update();

  // Check if the sequencer timer has elapsed and perform the appropriate
  //  state action if so. States 3 and 5 wait for tracks to stop playing and
  //  are therefore not in the metro event. They are instead polled after the
  //  metro check.
  if (gSeqMetro.check() == 1) {

    switch (gSeqState) {

      // State 0: Demonstrates how to fade in a music track
      case 0:
        // First retrieve and print the version and number of tracks
        if (tsunami.getVersion(gTsunamiVersion, VERSION_STRING_LEN)) {
          Serial.print(gTsunamiVersion);
          Serial.print("\n");
          gNumTracks = tsunami.getNumTracks();
          Serial.print("Number of tracks = ");
          Serial.print(gNumTracks);
          Serial.print("\n");
        }
        else
          Serial.print("WAV Trigger response not available");
        tsunami.samplerateOffset(0, 0);        // Reset sample rate offset to 0
        tsunami.masterGain(0, 0);              // Reset the master gain to 0dB

        tsunami.trackGain(2, -40);             // Preset Track 2 gain to -40dB
        tsunami.trackPlayPoly(2, 0, true);     // Start Track 2
        tsunami.trackFade(2, 0, 2000, false);  // Fade Track 2 to 0dB over 2 sec
        gSeqState = 1;                         // Advance to state 1
        break;

      // State 1: Demonstrates how to cross-fade music tracks
      case 1:
        tsunami.trackGain(1, -40);             // Preset Track 1 gain to -40dB
        tsunami.trackPlayPoly(1, 0, true);     // Start Track 1
        tsunami.trackFade(1, 0, 3000, false);  // Fade Track 1 up to 0db over 3 secs
        tsunami.update();
        delay(2000);                           // Wait 2 secs
        tsunami.trackFade(2, -40, 3000, true); // Fade Track 2 down to -40dB over 3 secs and stop
        Serial.print("Waiting for Track 2 to finish... ");
        gSeqState = 2;                         // Advance to state 2
        break;

      // State 3: Honk the horn 2 times
      case 3:
        tsunami.trackPlayPoly(5, 0, true);     // Start Track 5 poly
        tsunami.update();
        delay(500);
        tsunami.trackStop(5);                  // Stop Track 5
        tsunami.update();
        delay(250);
        tsunami.trackPlayPoly(5, 0, true);     // Start Track 5 poly
        tsunami.update();
        delay(500);
        tsunami.trackStop(5);                  // Stop Track 5
        gSeqState = 4;                         // Advance to state 4
        break;

      // State 4: Fade out and stop dialog
      case 4:
        tsunami.trackLoop(4, 0);               // Disable Track 4 looping
        tsunami.trackFade(4, -50, 5000, true); // Fade Track 4 to -50dB and stop
        Serial.print("Waiting for Track 4 to finish... ");
        gSeqState = 5;                         // Advance to state 5
        break;

      // State 6: Demonstrates preloading tracks and starting them in sample-
      //  sync, and real-time samplerate control (pitch bending);
      case 6:
        tsunami.trackLoad(6, 0, true);         // Load and pause Track 6
        tsunami.trackLoad(7, 0, true);         // Load and pause Track 7
        tsunami.trackLoad(8, 0, true);         // Load and pause Track 8
        tsunami.resumeAllInSync();             // Start all in sample sync

        // Decrement the sample rate offset from 0 to -32767 (1 octave down)
        //  in 10 ms steps
        gRateOffset = 0;
        for (i = 0; i < 127; i++) {
          gRateOffset -= 256;
          tsunami.samplerateOffset(0, gRateOffset);
          delay(10);
        }
        gRateOffset = -32767;
        tsunami.samplerateOffset(0, gRateOffset);

        // Hold for 1 second
        delay(1000);

        // Now increment to +32767 (1 octave up) in 10ms steps
        for (i = 0; i < 255; i++) {
          gRateOffset += 256;
          tsunami.samplerateOffset(0, gRateOffset);
          delay(10);
        }
        gRateOffset = 32767;
        tsunami.samplerateOffset(0, gRateOffset);

        // Hold for 1 second, the stop all tracks
        delay(1000);
        tsunami.stopAllTracks();               // Stop all
        gSeqState = 0;                         // Advance to state 0
        break;

    } // switch

  } // if (gSeqState.check() == 1)

  // State 2: Wait for Track 2 to stop, then fade down the music and start the
  //  dialog track looping.
  if (gSeqState == 2) {
    gSeqMetro.reset();                             // Reset the sequencer metro
    if (!tsunami.isTrackPlaying(2)) {
      Serial.print("Track 2 done\n");
      tsunami.trackFade(1, -6, 500, false);      // Lower the music volume
      tsunami.trackLoop(4, 1);                   // Enable Track 4 looping
      tsunami.trackPlayPoly(4, 0, true);         // Start Track 4 poly
      gSeqState = 3;                             // Advance to state 3;
    }
  }

  // State 5: Wait for Track 4 to stop, then play three tracks sequentially and
  //  stop all with a 5 sec fade to -50dB. This is how you can implement MIDI
  //  Note-On/Off control for: MIDI -> Arduino -> WAV Trigger.
  if (gSeqState == 5) {
    gSeqMetro.reset();
    if (!tsunami.isTrackPlaying(4)) {
      Serial.print("Track 4 done\n");
      tsunami.masterGain(0, -8);                 // Lower main volume
      tsunami.trackPlayPoly(6, 0, true);         // Play first note
      tsunami.update();
      delay(1000);
      tsunami.trackPlayPoly(7, 0, true);         // Play second note
      tsunami.update();
      delay(1000);
      tsunami.trackPlayPoly(8, 0, true);         // Play third note
      tsunami.update();
      delay(1000);
      tsunami.trackFade(6, -50, 5000, true);     // Fade Track 6 to -50dB and stop
      tsunami.trackFade(7, -50, 5000, true);     // Fade Track 7 to -50dB and stop
      tsunami.trackFade(8, -50, 5000, true);     // Fade Track 8 to -50dB and stop
      gSeqState = 6;
    }
  }

  // If time to do so, toggle the LED
  if (gLedMetro.check() == 1) {
    if (gLedState == 0) gLedState = 1;
    else gLedState = 0;
    digitalWrite(LED, gLedState);
  } // if (gLedMetro.check() == 1)

  // Delay 30 msecs
  delay(30);

  // after N seconds, transition to Offline, but we could easily get directed to Online before that.
  static Metro startupTimeout(10000UL);
//  if ( startupTimeout.check() ) stateMachine.transitionTo(Offline);

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
    stateMachine.transitionTo(Offline);
  }
}

// DANNE, this is the crucial part.  When we get distance and frequency information, how do those translate to sound?
void normal(boolean isOnline) {
  // what to do with the topics?
  if ( isOnline ) {

    // loop across arches
    for( byte i=0; i<N_ARCH; i++ ) {
      if( sAF[i].hasUpdate ) { 
        // we have an update to frequency information in sAF[i].freq

        // make some noise

        // note that we've handled the update already
        sAF[i].hasUpdate = false; 
      }
      if( sAD[i].hasUpdate ) { 
        // we have an update to distance information in sAD[i].dist

        // make some noise

        // note that we've handled the update already
        sAD[i].hasUpdate = false; 
      }
    }
  }
}

void slaved() {
  // NOP, currently.  Will look for lighting directions, either through UDP (direct) or MQTT (procedural).
}


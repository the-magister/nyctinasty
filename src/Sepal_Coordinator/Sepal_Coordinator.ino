// IDE Settings:
// Tools->Board : "WeMos D1 R2 & mini"
// Tools->Flash Size : "4M (3M SPIFFS)"
// Tools->CPU Frequency : "160 MHz"
// Tools->Upload Speed : "921600"

// keep an windward eye on dynamic memory usage: may need to go up to an ESP32
#include <Metro.h>
#include <Streaming.h>
#include <Eigen.h>
#include <Eigen/LU>
using namespace Eigen;    // simplifies syntax for declaration of matrices
#include <FastLED.h>      // seems to need to be after the namespace 
#include <FiniteStateMachine.h>
#include "Nyctinasty_Messages.h"
#include "Nyctinasty_Comms.h"

// define a state for every systemState
void startupUpdate(); State Startup = State(startupUpdate);
void normalUpdate(); State Normal = State(normalUpdate);
void idleUpdate(); State Idle = State(idleUpdate);
State Reboot = State(reboot);
void reprogram() {
  reprogram("Sepal_Coordinator.ino.bin");
} State Reprogram = State(reprogram);
FSM stateMachine = FSM(Startup); // initialize state machine

// incoming message storage and flag for update
SystemCommand settings;         boolean settingsUpdate = false;   String settingsTopic;

// incoming message storage and flag for update
SepalArchFreq freq[N_ARCHES];   boolean freqUpdate[N_ARCHES] = {false};

// This function sets up the leds and tells the controller about them
void setup() {
  Serial.begin(115200);

  Serial << endl << endl << endl << F("Startup.") << endl;

  // who am I?
  Id myId = commsBegin();

  // publish
  settingsTopic = commsTopicSystemCommand();
  Serial << F("Publishing: ") << settingsTopic << endl;

  // subscribe
  commsSubscribe(settingsTopic, &settings, &settingsUpdate, 1); // QoS 1
  for ( byte i = 0; i < N_ARCHES; i++ ) {
    commsSubscribe(commsTopicFrequency(myId.sepal, i), &freq[i], &freqUpdate[i]);
  }

  Serial << F("Startup. complete.") << endl;
}

void loop() {
  // comms handling
  commsUpdate();

  // bail out if not connected
  if ( ! commsConnected() ) return;

  // order these by priority:

  // check for settings update
  if ( settingsUpdate ) {
    switchState(settings.state);
    settingsUpdate = false;
  }

  // do stuff
  stateMachine.update();
}


void switchState(systemState state) {
  Serial << F("State.  Changing to ") << state << endl;
  switch ( state ) {
    case STARTUP: stateMachine.transitionTo(Startup); break;
    case NORMAL: stateMachine.transitionTo(Normal); break;
    case CENTRAL: stateMachine.transitionTo(Idle); break;
    case REBOOT: stateMachine.transitionTo(Reboot); break;
    case REPROGRAM: stateMachine.transitionTo(Reprogram); break;
    default:
      Serial << F("ERROR!  unknown state.") << endl;
  }
}

void startupUpdate() {
  // As a Coordinator, we're responsible for sending a NORMAL systemState after startup
  uint32_t startupDelay = 10000UL;
  static uint32_t tic = millis();
  if ( millis() < (tic + startupDelay) ) return;

  Serial << F("State.  Sending NORMAL...") << endl;

  settings.state = NORMAL;
  commsPublish(settingsTopic, &settings);

  // go to central update while we wait for our subscription to arrive
  stateMachine.transitionTo(Idle);
}

void idleUpdate() {
  // NOP; on hold until we hear from the Coordinator
}

void normalUpdate() {
  // check for an update to frequency
  for ( byte i = 0; i < N_ARCHES; i++ ) {
    if ( freqUpdate[i] ) {
      Serial << F("freqUpdate ") << i << endl;
      // reset
      freqUpdate[i] = false;
    }
  }
}

/*
  void distUpdate0() { distUpdate(0); }
  void distUpdate1() { distUpdate(1); }
  void distUpdate2() { distUpdate(2); }
  void distUpdate(byte i) {
  Serial << F("Dist update:") << i << endl;
  // map to lights
  mapDistanceToBar(i);
  // ship it, but bail out if no connection
  commsPublish(lightsTopic[i], &lights[i]);
  }

  void freqUpdate0() { freqUpdate(0); }
  void freqUpdate1() { freqUpdate(1); }
  void freqUpdate2() { freqUpdate(2); }
  void freqUpdate(byte i) {
  Serial << F("freqUpdate ") << i << endl;
  }
*/

// MatrixMath library
// https://playground.arduino.cc/Code/MatrixMath

// determinant function:
// http://paulbourke.net/miscellaneous/determinant/determinant.c

// C++ linear algebra library
// http://arma.sourceforge.net/download.html

// Eigen library
// https://stackoverflow.com/questions/15138634/eigen-is-there-an-inbuilt-way-to-calculate-sample-covariance

// FFT with Eigen
// https://stackoverflow.com/questions/36719364/how-to-do-fft-on-matrixxd-in-eigen

// PRINT MATRIX (double type)
// By: randomvibe
//-----------------------------
/*
  void print_mtx(const MatrixFreq & X) {
  int i, j, nrow, ncol;

  nrow = X.rows();
  ncol = X.cols();

  Serial.print("nrow: "); Serial.println(nrow);
  Serial.print("ncol: "); Serial.println(ncol);
  Serial.println();

  for (i = 0; i < nrow; i++)
  {
    for (j = 0; j < ncol; j++)
    {
      Serial.print(X(i, j), 6);  // print 6 decimal places
      Serial.print(", ");
    }
    Serial.println();
  }
  Serial.println();
  }
*/

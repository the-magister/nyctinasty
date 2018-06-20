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

// comms
NyctComms comms;

// define a state for every systemState
void idle(); State Idle = State(idle);
void normal(); State Normal = State(normal);
void central(); State Central = State(central);
void reboot() { comms.reboot(); }; State Reboot = State(reboot);
void reprogram() { comms.reprogram("Sepal_Coordinator.ino.bin"); } State Reprogram = State(reprogram);
FSM stateMachine = FSM(Idle); // initialize state machine

// my role
NyctRole role = Coordinator;

// incoming message storage and flag for update
struct sC_t { boolean hasUpdate=false; SystemCommand settings; } sC;

// incoming message storage and flag for update
typedef struct { boolean hasUpdate=false; SepalArchFrequency freq; } sAF_t;
sAF_t sAF[N_ARCH];

// This function sets up the leds and tells the controller about them
void setup() {
  Serial.begin(115200);

  Serial << endl << endl << endl << F("Startup.") << endl;

  // start comms
  comms.begin(role);

  // subscribe
  comms.subscribe(&sC.settings, &sC.hasUpdate);
  for ( byte i = 0; i < N_ARCH; i++ ) {
    comms.subscribe(&sAF[i].freq, &sAF[i].hasUpdate, comms.getSepal(), i);
  }

  Serial << F("Startup. complete.") << endl;
}

void loop() {
  // comms handling
  comms.update();

  // bail out if not connected
  if ( ! comms.isConnected() ) return;

  // order these by priority:

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
    case IDLE: stateMachine.transitionTo(Idle); break;
    case NORMAL: stateMachine.transitionTo(Normal); break;
    case CENTRAL: stateMachine.transitionTo(Central); break;
    case REBOOT: stateMachine.transitionTo(Reboot); break;
    case REPROGRAM: stateMachine.transitionTo(Reprogram); break;
    default:
      Serial << F("ERROR!  unknown state.") << endl;
  }
}

void idle() {
  // As a Coordinator, we're responsible for sending a NORMAL systemState after startup
  uint32_t startupDelay = 3000UL;
  static uint32_t tic = millis();
  if ( millis() < (tic + startupDelay) ) return;

  Serial << F("State.  Sending NORMAL...") << endl;

  sC.settings.state = NORMAL;
  comms.publish(&sC.settings);

  // go to central update while we wait for our subscription to arrive
  stateMachine.transitionTo(Idle);
}

void normal() {
  // check for an update to frequency
  for ( byte i = 0; i < N_ARCH; i++ ) {
    if ( sAF[i].hasUpdate ) {
      Serial << F("freqUpdate ") << i << endl;
      // reset
      sAF[i].hasUpdate = false;
    }
  }
}

void central() {
  // NOP
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

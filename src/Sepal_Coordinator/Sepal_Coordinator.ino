// Compile for Wemos D1 R2 & Mini
// keep an windward eye on dynamic memory usage: may need to go up to an ESP32
#include <Metro.h>
#include <Streaming.h>
#include <Eigen.h>
#include <Eigen/LU>
using namespace Eigen;    // simplifies syntax for declaration of matrices
#include <FastLED.h>      // seems to need to be after the namespace 
#include <EEPROM.h>
#include <FiniteStateMachine.h>
#include <ESP8266httpUpdate.h>
#include "Nyctinasty_Messages.h"
#include "Nyctinasty_Comms.h"


// define a state for every systemState
void startupUpdate(); State Startup = State(startupUpdate); 
void normalUpdate(); State Normal = State(normalUpdate);
void idleUpdate(); State Idle = State(idleUpdate);
State Reboot = State(reboot);
void reprogram() { reprogram("Sepal_Coordinator.ino.bin"); } State Reprogram = State(reprogram);
FSM stateMachine = FSM(Startup); //initialize state machine

// who am I?
const byte SepalNumber = 0;
const String id = commsIdSepalCoordinator(SepalNumber);

// ship settings
SystemCommand settings;
// in this topic
const String settingsTopic = commsTopicSystemCommand();
// and sets this true when an update arrives
boolean settingsUpdate = false;

// our distance updates arrive as this structure
SepalArchFreq freq[N_ARCHES];
// in these topics
const String freqTopic[N_ARCHES] = {
  commsTopicFreq(SepalNumber, 0),
  commsTopicFreq(SepalNumber, 1),
  commsTopicFreq(SepalNumber, 2)
};
// and sets this true when an update arrives
boolean freqUpdate[N_ARCHES] = {false};

// This function sets up the ledsand tells the controller about them
void setup() {
  Serial.begin(115200);

  Serial << endl << endl << F("Startup.") << endl;
  delay(500);

  commsBegin(id);
  commsSubscribe(settingsTopic, &settings, &settingsUpdate, 1); // QoS 1
  //  commsSubscribe(settingsTopic, &settings, settingsUpdate);
  for ( byte i = 0; i < N_ARCHES; i++ ) {
    commsSubscribe(freqTopic[i], &freq[i], &freqUpdate[i]);
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
  Metro startupDelay(1000UL);
  while( !startupDelay.check() ) {
    commsUpdate();
    yield();
  }
   
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

// compile for Nano
#include <Streaming.h>
#include <Metro.h>
#include <AnalogScanner.h>
#include <SoftwareSerial.h>
#include <SoftEasyTransfer.h>
#include <EEPROM.h>
#include "Skein_Messages.h"

// pin definitions
#define LED 13
#define RX 2
#define TX 3
// A0..A7 are also used
// wire 6.8kOhm resistor between +3.3 and AREF.

// for comms
SoftwareSerial mySerial(RX, TX); // cross pairs
SoftEasyTransfer ETin, ETout;

// track cycle time
unsigned long targetCycleTime = 20UL; // ms

// ship readings
SensorReading readings;

// ship settings
Command settings;

// Creates an instance of the analog pin scanner.
AnalogScanner scanner;

// to go with output
String msg = "NULL";

// compute maximum cycle time from target fps
unsigned long cycleTime(byte fps) {
  settings.fps = fps;

  unsigned long time = 1000UL / (unsigned long)settings.fps; // ms
  Serial << F("Settings.  fps=") << settings.fps << F(" cycle time=") << time << F(" ms") << endl;

  return ( time );
}

// send sensor readings
const boolean showReadings[N_SENSOR] = {true, true, true, true, true, true, true, true};
void sendReadings() {
  if( showReadings ) {
    const char sep[] = ",";
    for ( byte i = 0; i < N_SENSOR; i++ ) {
      if( showReadings[i] ) Serial << readings.dist[i] << sep;
    }
    Serial << readings.min << sep;
    Serial << readings.noise << sep;
    Serial << readings.max << sep;
    Serial << msg << endl;
  }
  
  // ship it.
  ETout.sendData();
}

// this is an ISR, so needs to be quick.  Just stash the values.
void getValue(int index, int pin, int value) {
  const uint16_t smoothing = 10;
  readings.dist[index] = (readings.dist[index]*(smoothing-1)+(uint32_t)value)/smoothing;
}

void toggleLED() {
  static boolean LEDstate = false;
  LEDstate = !LEDstate;
  digitalWrite(LED, LEDstate);
}

void setup() {
  // for local output
  Serial.begin(115200);

  // for remote output
  //  mySerial.begin(57600);
  mySerial.begin(115200);

  // messages
  ETin.begin(details(settings), &mySerial);
  ETout.begin(details(readings), &mySerial);

  // free run ADC reading
  int scanOrder[] = {A0, A1, A2, A3, A4, A5, A6, A7};
  scanner.setScanOrder(N_SENSOR, scanOrder);
  for ( byte i = 0; i < N_SENSOR; i++) scanner.setCallback(scanOrder[i], getValue);
  scanner.setAnalogReference(EXTERNAL); // tuned to 2.75V = 1023 reading
  // has 32K built-in resistor.
  // connect AREF to +3.3v via 6.8 kOhm resistor
  scanner.beginScanning();

  settings.fps = defaultFPS;
  targetCycleTime = cycleTime(settings.fps);

  readings.min = 0;
  readings.max = 1023;
  readings.noise = 50;
}

void loop() {
  // make a timer
  static Metro sendInterval(targetCycleTime);
  if ( sendInterval.check() ) {
    // reset timer
    sendInterval.reset();

    // flash the LED as we loop
    toggleLED();

    // gather up the readings
//    gatherReadings();

    // send readigns
    sendReadings();
  }

  // save and apply settings
  if ( ETin.receiveData() ) {
    targetCycleTime = cycleTime(settings.fps);
    sendInterval.interval(targetCycleTime);
  }

  // check for serial commands
  if ( Serial.available() ) {
    msg = Serial.readString();
  }
}

/*


  // read sensors, using oversampling and decimation to gain additional bit depth
  void readSensors(byte additionalBits) {
  // oversampling and decimation
  // http://www.atmel.com/images/doc8003.pdf
  const byte samplesByBit[] = {1, 2, 4, 8, 16, 32, 64}; // 10..16 bit depth ADC
  byte count = samplesByBit[constrain(additionalBits, 0, 6)];

  // storage
  uint32_t values[N_SENSOR] = {0};

  // get a bunch of readings
  for ( byte j = 0; j < count; j++ ) {
    for ( byte i = 0; i < N_SENSOR; i++ ) {
      values[i] += analogRead(i);
    }
  }

  // set the min and max from bit depth
  readings.min = (uint16_t)0;
  readings.max = ((uint16_t)1 << (10 + additionalBits)) - (uint16_t)1;
  // set the noise floor in LSB
  const byte dropBits = 1;
  readings.noise = ((uint16_t)1 << (10 + additionalBits - dropBits)) - (uint16_t)1;

  // decimate
  for ( byte i = 0; i < N_SENSOR; i++ ) {
    readings.dist[i] = readings.max - (uint16_t)values[i];
  }

  }
*/



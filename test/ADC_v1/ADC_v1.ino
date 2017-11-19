// compile for Nano
#include <Streaming.h>
#include <Metro.h>
#include <SoftwareSerial.h>
#include <SoftEasyTransfer.h>
#include <EEPROM.h>
#include "Skein_Messages.h"

// pin definitions
#define LED 13
#define RX 2
#define TX 3
#define VH 6
// A0..A7 are also used
// wire 6.8kOhm resistor between +3.3 and AREF.

// for comms
SoftwareSerial mySerial(RX, TX); // cross pairs
SoftEasyTransfer ETin, ETout;

// track cycle time
unsigned long targetCycleTime = 20UL; // ms

// store readings
SensorReading readings;

// store settings
Command settings;

void setup() {
  // for local output
  Serial.begin(115200);

  // send 5V to the level shifter for a reference.
  pinMode(VH, OUTPUT);
  digitalWrite(VH, HIGH);
  
  // for remote output
  mySerial.begin(57600);
  while( !mySerial ) delay(5);
  
  // messages
//  ETin.begin(details(settings), &mySerial);
  ETout.begin(details(readings), &mySerial);

  // put your setup code here, to run once:
  analogReference(EXTERNAL); // tuned to 2.75V = 1023 reading
  // has 32K built-in resistor.
  // connect AREF to +3.3v via 6.8 kOhm resistor

  settings = loadCommand();
  targetCycleTime = cycleTime(settings.fps);
}


void loop() {
  // track actual cycle time
  unsigned long timer = millis();

  // flash the LED as we loop
  static boolean LEDstate = false;
  LEDstate = !LEDstate;
  digitalWrite(LED, LEDstate);

  // track bit depth
  static byte additionalBits = 0;

  // read sensors
  readSensors( additionalBits );

  // send sensors
  sendSensors();

  // check cycle time against target
  timer = millis() - timer;
  if ( timer > targetCycleTime * 2 ) {
    // bit depth is too high
    if ( additionalBits > 0 ) --additionalBits;
    Serial << F("Timing.  cycle=") << timer << F("ms. Adjusting additionalBits down to ") << additionalBits << endl;
  } else if (timer < targetCycleTime / 2 ) {
    // bit depth is too low
    if ( additionalBits < 6 ) ++additionalBits;
    Serial << F("Timing.  cycle=") << timer << F("ms. Adjusting additionalBits up to ") << additionalBits << endl;
  }

  // save and apply settings
//  if ( ETin.receiveData() ) {  
//    targetCycleTime = cycleTime(settings.fps);
//    saveCommand(settings);
//  }
}

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

// compute maximum cycle time from target fps
unsigned long cycleTime(byte fps) {
  settings.fps = fps;

  unsigned long time = 1000UL / (unsigned long)settings.fps; // ms
  Serial << F("Settings.  fps=") << settings.fps << F(" cycle time=") << time << F(" ms") << endl;

  return ( time );
}

// send sensor readings
void sendSensors() {
  const char sep[] = ",";
  for ( byte i = 0; i < N_SENSOR; i++ ) Serial << readings.dist[i] << sep;
  Serial << readings.min << sep;
  Serial << readings.noise << sep;
  Serial << readings.max << endl;

  // ship it.
  ETout.sendData();
}


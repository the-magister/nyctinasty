// IDE Settings:
// Tools->Board : "Arduino Nano"
// Tools->Processor : "ATmega 328P"

// This uC simply samples the distance sensors as quickly as possible,
// then trasmits to Sepal_Arch over serial when requested.
//
// Code is intentionally "dumb as a brick".  No OTA update is possible to this uC, so
// we need this codebase to be stable and bullet-proof.

#define SHOW_SERIAL_DEBUG false

#include <Streaming.h>
#include <AnalogScanner.h>
#include <SoftwareSerial.h>
#include <SoftEasyTransfer.h>
#include "Nyctinasty_Messages.h"

// wire it up
#define BUILTIN_LED 13
// A0..A7 are also used
#define RX 3 // connected.
#define TX 2 // connected. 
// wire 6.8kOhm resistor between +3.3 and AREF.

// for comms
SoftwareSerial mySerial(RX, TX); // cross pairs
SoftEasyTransfer ETout;

// ship dist
volatile SepalArchDistance dist;

// Creates an instance of the analog pin scanner.
AnalogScanner scanner;

void setup() {
  // for local output
  Serial.begin(115200);

  // up
  Serial << F("Sepal_Arch_Distance (ADC) Startup.") << endl;

  // for remote output
  mySerial.begin(115200);

  // messages
  ETout.begin(details(dist), &mySerial);

  // after set up the input pin
  pinMode(RX, INPUT); // not pullup; 3.3v sensitive pin attached.

  // setup LED pin
  pinMode(BUILTIN_LED, OUTPUT);

  // free run ADC reading
  int scanOrder[] = {A0, A1, A2, A3, A4, A5, A6, A7};
  scanner.setScanOrder(N_SENSOR, scanOrder);
  // define callback
  for ( byte i = 0; i < N_SENSOR; i++) scanner.setCallback(scanOrder[i], ISR_getValue);
  scanner.setAnalogReference(EXTERNAL); // tuned to 2.75V = 1023 reading
  // has 32K built-in resistor.
  // connect AREF to +3.3v via 6.8 kOhm resistor
  scanner.beginScanning();

  // message information
  dist.min = 0;
  dist.max = 1023;
  dist.noise = 50;

  Serial << F("Startup complete.  Will print=") << SHOW_SERIAL_DEBUG << endl;
}

// this is an ISR, so needs to be quick.
uint32_t smoothing = 1;
volatile uint32_t count = 1; // intentional 1 index.
void ISR_getValue(int index, int pin, int value) {
  // anything that could be changed by this ISR should be volatile
  byte ri = A7 - pin;
  
  // exponential smoothing
  uint32_t smoothed =((uint32_t)dist.prox[ri] * (smoothing - 1) + (uint32_t)value) / smoothing; 
  dist.prox[ri] = (uint16_t)smoothed;
  
  // increment read counter
  count++;
}

// send sensor dist
void sendDistance() {

  if( SHOW_SERIAL_DEBUG ) {
    const char sep[] = ",";

    for ( byte i = 0; i < N_SENSOR; i++ ) Serial << dist.prox[i] << sep;
  
    Serial << dist.min << sep;
    Serial << dist.noise << sep;
    Serial << dist.max << sep;
    Serial << smoothing << sep;
    Serial << endl;
  }

  // ship it.
  ETout.sendData();

  // toggle the LED
  static boolean LEDstate = false;
  LEDstate = !LEDstate;
  digitalWrite(BUILTIN_LED, LEDstate);
}

void loop() {
  if ( digitalRead(RX) ) {
    // track updates per send loop, and adjust smoothing,
    // reset count=1 to never allow smoothing=0 that could be used in the ISR whenever
    const uint32_t smoothMult = 4;
    smoothing = (smoothMult*count) / (uint32_t)N_SENSOR;
    count = 1;

    // send readigns
    sendDistance();

    // wait 
    while( digitalRead(RX) );
  }
}


// IDE Settings:
// Tools->Board : "Arduino Nano"
// Tools->Processor : "ATmega 328P"

// This uC simply samples the distance sensors as quickly as possible,
// then trasmits to Sepal_Arch_Freq over serial every distanceSampleRate ms.

#include <Streaming.h>
#include <Metro.h>
#include <AnalogScanner.h>
#include <SoftwareSerial.h>
#include <SoftEasyTransfer.h>
#include "Nyctinasty_Messages.h"

// make a timer
Metro sendInterval(distanceSampleRate);

// pin definitions
#define BUILTIN_LED 13
// A0..A7 are also used
#define RX 3 // not connected, but don't use for something else
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

  // for remote output
  //  mySerial.begin(57600);
  mySerial.begin(115200);

  // messages
  ETout.begin(details(dist), &mySerial);

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
}

// this is an ISR, so needs to be quick.
uint16_t smoothing = 1;
volatile uint16_t count = 0;
void ISR_getValue(int index, int pin, int value) {
  // anything that could be changed by this ISR should be volatile
  byte ri = A7 - pin;
  
  // exponential smoothing
  uint32_t smoothed =((uint32_t)dist.prox[ri] * (uint32_t)(smoothing - 1) + (uint32_t)value) / (uint32_t)smoothing; 
  dist.prox[ri] = smoothed;
  
  // increment read counter
  count++;
}

// send sensor dist
void sendDistance() {

  const char sep[] = ",";
  for ( byte i = 0; i < N_SENSOR; i++ ) Serial << dist.prox[i] << sep;

  Serial << dist.min << sep;
  Serial << dist.noise << sep;
  Serial << dist.max << sep;
  Serial << smoothing << sep;
  Serial << endl;

  // ship it.
  ETout.sendData();

  // toggle the LED
  static boolean LEDstate = false;
  LEDstate = !LEDstate;
  digitalWrite(BUILTIN_LED, LEDstate);
}

void loop() {
  if ( sendInterval.check() ) {
    // track updates per send loop, and adjust smoothing,
    // taking care to never allow smoothing=0 that could be used in the ISR whenever
    const uint16_t smoothMult = 4;
    uint16_t newSmoothing = (smoothMult*count) / (uint16_t)N_SENSOR;
    if ( newSmoothing < 1 ) newSmoothing = 1;
    smoothing = newSmoothing;
    count = 0;

    // send readigns
    sendDistance();
  }
}


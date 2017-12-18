// compile for Nano
#include <Streaming.h>
#include <Metro.h>
#include <AnalogScanner.h>
#include <SoftwareSerial.h>
#include <SoftEasyTransfer.h>
#include "Nyctinasty_Messages.h"

// pin definitions
#define RX 2 // not connected
#define TX 3 // connected
#define BUILTIN_LED 13 // blinky
// A0..A7 are also used
// wire 6.8kOhm resistor between +3.3 and AREF.

// for comms
SoftwareSerial mySerial(RX, TX); // cross pairs
SoftEasyTransfer ETout;
// on TX, use a voltage divider 3.3=5* R2/(R1+R2)
// TX_Nano - R1 - RX_ESP - R2 - GND
// R1 = 510 Ohm
// R2 = 1000 Ohm

// ship dist
SepalArchDistance dist;

// Creates an instance of the analog pin scanner.
AnalogScanner scanner;

// send sensor dist
void senddist() {
  
  const char sep[] = ",";
  for ( byte i = 0; i < N_SENSOR; i++ ) Serial << dist.prox[i] << sep;

  Serial << dist.min << sep;
  Serial << dist.noise << sep;
  Serial << dist.max;
  Serial << endl;

  // ship it.
  ETout.sendData();
}

// this is an ISR, so needs to be quick.  Just stash the values.
uint16_t smoothing = 1;
uint16_t count = 0;
void getValue(int index, int pin, int value) {
  // exponential smoothing
  dist.prox[index] = (dist.prox[index]*(smoothing-1)+(uint16_t)value)/smoothing;
  // increment read counter
  count++;
}

void toggleLED() {
  static boolean LEDstate = false;
  LEDstate = !LEDstate;
  digitalWrite(BUILTIN_LED, LEDstate);
}

void setup() {
  // for local output
  Serial.begin(115200);

  // for remote output
  //  mySerial.begin(57600);
  mySerial.begin(115200);

  // setup LED
  pinMode(BUILTIN_LED, OUTPUT);

  // messages
  ETout.begin(details(dist), &mySerial);

  // free run ADC reading
  int scanOrder[] = {A0, A1, A2, A3, A4, A5, A6, A7};
  scanner.setScanOrder(N_SENSOR, scanOrder);
  for ( byte i = 0; i < N_SENSOR; i++) scanner.setCallback(scanOrder[i], getValue);
  scanner.setAnalogReference(EXTERNAL); // tuned to 2.75V = 1023 reading
  // has 32K built-in resistor.
  // connect AREF to +3.3v via 6.8 kOhm resistor
  scanner.beginScanning();

  // message information
  dist.min = 0;
  dist.max = 1023;
  dist.noise = 50;
}

void loop() {
  // make a timer
  static Metro sendInterval(distanceSampleRate);
  if ( sendInterval.check() ) {
    // reset timer
    sendInterval.reset();

    // flash the LED as we loop
    toggleLED();

    // track updates per send loop, and adjust smoothing,
    // taking care to never allow smoothing=0 that could be used in the ISR whenever
    uint16_t newSmoothing = count/N_SENSOR;
    if( newSmoothing<1 ) newSmoothing=1;
    smoothing = newSmoothing;
    count = 0;

    // send readigns
    senddist();
  }
}


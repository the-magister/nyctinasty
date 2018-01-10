// IDE Settings:
// Tools->Board : "Arduino Nano"
// Tools->Processor : "ATmega 328P"

// This uC simply samples the distance sensors as quickly as possible,
// then trasmits to Sepal_Arch_Freq over serial every distanceSampleRate ms.

#include <Streaming.h>
#include <Metro.h>
#include <FastLED.h>
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

// Fast LED
#define LED_DATA  11  // use the hardware SPI pin
#define NUM_LEDS 16  // LED count
CRGBArray<NUM_LEDS> leds;
#define COLOR_ORDER RGB
#define COLOR_CORRECTION TypicalLEDStrip

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

  // message information
  dist.min = 0;
  dist.max = 1023;
  dist.noise = 50;
  
  // setup LED pin
  pinMode(BUILTIN_LED, OUTPUT);

  // LEDS
  FastLED.addLeds<WS2811, LED_DATA, RGB>(leds, NUM_LEDS).setCorrection(COLOR_CORRECTION);
  // run test pattern
  runTestPattern();

  // free run ADC reading
  int scanOrder[] = {A0, A1, A2, A3, A4, A5, A6, A7};
  scanner.setScanOrder(N_SENSOR, scanOrder);
  // define callback
  for ( byte i = 0; i < N_SENSOR; i++) scanner.setCallback(scanOrder[i], ISR_getValue);
  scanner.setAnalogReference(EXTERNAL); // tuned to 2.75V = 1023 reading
  // has 32K built-in resistor.
  // connect AREF to +3.3v via 6.8 kOhm resistor
  scanner.beginScanning();

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

void loop() {
  if ( sendInterval.check() ) {
    // shut down the scanner so we don't fire ISR
    scanner.endScanning();
    
    // track updates per send loop, and adjust smoothing,
    // taking care to never allow smoothing=0 that could be used in the ISR whenever
    const uint16_t smoothMult = 4;
    uint16_t newSmoothing = (smoothMult*count) / (uint16_t)N_SENSOR;
    if ( newSmoothing < 1 ) newSmoothing = 1;
    smoothing = newSmoothing;
    count = 0;

    // send readings
    sendDistance();

    // update lights
    mapDistanceToBar();
    pushToHardware();
    
    // enable scanning
    scanner.beginScanning();
  }
}


void mapDistanceToBar() {

  // indexes to ease access loop installation
  const byte barIndex1[N_SENSOR]={ 3,  2,  1,  0, 15, 14, 13, 12};
  const byte barIndex2[N_SENSOR]={ 4,  5,  6,  7,  8,  9, 10, 11};
  
  const fract16 scale = 1 << 14; // for scaling operation
  
  // track hue
  static byte hue=random8(0,255);
  
  for ( byte i = 0; i < N_SENSOR; i++ ) {
    // note that we're "smooshing" proximities that don't meet the noise 
    // threshold back down to zero.
    if( dist.prox[i] <= dist.noise ) dist.prox[i] = dist.min;
//    uint16_t intensity = map(
//                           dist.prox[i],
//                           dist.min, dist.max,
//                           (uint16_t)0, (uint16_t)255
//                         );
    // take a 2^10 value and crunch it down to 2^8.
    uint16_t intensity = scale16( dist.prox, scale );
                         
    leds[barIndex1[i]] = CHSV(hue++, 255, (byte)intensity);
    leds[barIndex2[i]] = leds[barIndex1[i]]; // mirror
  }
}

void runTestPattern() {
  // zero out all of the prox data
  for( byte j=0; j<N_SENSOR; j++ ) dist.prox[j]=0;

  // move through each sensor
  for( byte i=0; i<N_SENSOR; i++ ) {
    // set one to "very close"
    dist.prox[i] = dist.max;
    // show it
    mapDistanceToBar();
    pushToHardware();
    // wait
    delay(300);    
    // clean up
    dist.prox[i] = 0;
  }
}

void pushToHardware() {
  FastLED.show();

  // show our frame rate, periodically
  EVERY_N_SECONDS( 5 ) {
    uint16_t reportedFPS = FastLED.getFPS();
    Serial << F("FPS reported (Hz): ") << reportedFPS << endl;
  }
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



/*
   This module is responsible for polling the LIDAR sensors and publishing that data.
   We can have multiple sensors, and this is the i-th sensor.

   Publishes: skein/range/i/[0-7]: distance information, mm
              skein/range/i/oor: distance corresponding to out-of-range, mm
*/

// You'll need to add http://arduino.esp8266.com/stable/package_esp8266com_index.json to the Additional Board Managers URL entry in Preferences.
// Compile for Adafruit HUZZAH Feather
#include <Streaming.h>
#include <Metro.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include "Skein_Messages.h"
#include <SoftEasyTransfer.h>
#define FASTLED_ESP8266_RAW_PIN_ORDER
#include <FastLED.h>
#include <arduinoFFT.h>

// pin definitions
#define RED_LED 0
#define BLU_LED 2
#define RX 12
#define TX 14
#define L1 4
#define L2 5

SoftwareSerial mySerial(RX, TX, false, 256);
SoftEasyTransfer ETin, ETout;

// store readings
SensorReading readings;

// store settings
Command settings;

const byte NUM_LEDS = 12;
CRGBArray<NUM_LEDS> light0;
CRGBArray<NUM_LEDS> light1;
const byte NUM_RANGE = 4;
const byte NUM_POWER = NUM_LEDS - NUM_RANGE;

// FFT object
arduinoFFT FFT = arduinoFFT();

void setup(void)  {
  Serial.begin(115200);
  delay(10);
  Serial << endl << endl << "Startup." << endl;

  pinMode(BLU_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  // for remote output
  mySerial.begin(57600);
  while ( !mySerial ) delay(5);

  // messages
  ETin.begin(details(readings), &mySerial);
  ETout.begin(details(settings), &mySerial);

  // load settings
  //  EEPROM.begin(512); // required for ESPers
  //  settings = loadCommand();
  settings.fps = defaultFPS;
  //  saveCommand(settings);
  //  EEPROM.commit();

  Serial << "Startup. fps=" << settings.fps << endl;
  //  ETout.sendData(); // send target FPS

  delay(1000);

  FastLED.addLeds<WS2811, L1, RGB>(light0, NUM_LEDS);
  FastLED.addLeds<WS2811, L2, RGB>(light1, NUM_LEDS);
  FastLED.setMaxRefreshRate(settings.fps);

  light0(0, NUM_LEDS - 1) = CRGB::White;
  light1(0, NUM_LEDS - 1) = CRGB::White;
  FastLED.show();
  delay(500);
  light0(0, NUM_LEDS - 1) = CRGB::Black;
  light1(0, NUM_LEDS - 1) = CRGB::Black;
  FastLED.show();

  Serial << "Startup complete." << endl;
}

void loop(void) {
  // can't use FastLED's .show() routine, as that has a blocking loop/wait
  static Metro showInterval(1000UL / settings.fps);
  if ( showInterval.check() ) {
    FastLED.show();
    showInterval.reset();
  }


  // check for data
  static boolean LEDred = false;
  if ( ETin.receiveData() ) {
    LEDred = !LEDred;
    digitalWrite(RED_LED, LEDred);

    // show range data on the overhead lights
    showRange();

    // compute the FFT
    computeFFT();

    FastLED.show();
    showInterval.reset();
  }

}

// https://cdn-learn.adafruit.com/downloads/pdf/fft-fun-with-fourier-transforms.pdf
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
// Nyquist's Sampling Theorem: only frequencies up to half the sampling rate can
// can be detected.  So, for 33 fps (Hz) we can detect signals no faster than 33/2 Hz.  
double samplingFrequency = settings.fps;
const uint16_t samples = 64;  // This value MUST ALWAYS be a power of 2
/* 
 Finally, the output of the FFT on real data has a few interesting properties:
  * The very first bin (bin zero) of the FFT output represents the average power of the signal.
Be careful not to try interpreting this bin as an actual frequency value!
* Only the first half of the output bins represent usable frequency values. This means the
range of the output frequencies detected by the FFT is half of the sample rate. Don't try to
interpret bins beyond the first half in the FFT output as they won't represent real frequency
values!
*/

void computeFFT() {
  // storage
  static double vReal[N_SENSOR][samples];
  static double vImag[N_SENSOR][samples];

  // current ring index
  static uint16_t cInd = 0;

  // assign readings.dist[0]
  vReal[0][cInd] = readings.dist[0];

  // increment ring index
  cInd++;

  // until we fill the ring, don't compute
  static boolean ringReady = false;

  // check to see if we're OOB
  if ( cInd >= samples ) {
    cInd = 0;
    ringReady = true; // and we now have a full ring
  }

  // bail out if the ring isn't ready/full
  if ( ! ringReady ) return;

  // otherwise, zero out the imaginary part
  for ( uint16_t i = 0; i < samples; i++ ) {
    vImag[0][i] = 0;
  }

  // and compute
  Serial.println("Data:");
  PrintVector(vReal[0], samples, SCL_TIME);
  FFT.Windowing(vReal[0], samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  Serial.println("Weighed data:");
  PrintVector(vReal[0], samples, SCL_TIME);
  FFT.Compute(vReal[0], vImag[0], samples, FFT_FORWARD); /* Compute FFT */
  Serial.println("Computed Real values:");
  PrintVector(vReal[0], samples, SCL_INDEX);
  Serial.println("Computed Imaginary values:");
  PrintVector(vImag[0], samples, SCL_INDEX);
  FFT.ComplexToMagnitude(vReal[0], vImag[0], samples); /* Compute magnitudes */
  Serial.println("Computed magnitudes:");
  PrintVector(vReal[0], (samples >> 1), SCL_FREQUENCY);
  double x = FFT.MajorPeak(vReal[0], samples, samplingFrequency);
  Serial.println(x, 6);

}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType) {
  for (uint16_t i = 0; i < bufferSize; i++)   {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
        break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
        break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
        break;
    }
    Serial.print(abscissa, 6);
    Serial.print(" ");
    Serial.print(vData[i], 4);
    Serial.println();
  }
  Serial.println();
}

// show range on the first set of lights.
void showRange() {
  // set color, maximal
  light0(0, NUM_RANGE - 1) = CRGB::Green;
  light1(0, NUM_RANGE - 1) = CRGB::Blue;

  // scale readings
  uint16_t scaled[N_SENSOR] = {0};
  for (byte i = 0; i < N_SENSOR; i++ ) {
    scaled[i] = map(readings.dist[0], readings.min, readings.max, (uint16_t)255, (uint16_t)0);
  }

  // dim/fade lighting by scaled distance
  light0[0] %= scaled[3];
  light0[1] %= scaled[2];
  light0[2] %= scaled[1];
  light0[3] %= scaled[0];
  light1[0] %= scaled[4];
  light1[1] %= scaled[5];
  light1[2] %= scaled[6];
  light1[3] %= scaled[7];
}


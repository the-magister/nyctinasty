/*
   This module is responsible for polling the LIDAR sensors and publishing that data.
   We can have multiple sensors, and this is the i-th sensor.

   Publishes: skein/range/i/[0-7]: distance information, mm
              skein/range/i/oor: distance corresponding to out-of-range, mm
*/

// You'll need to add http://arduino.esp8266.com/stable/package_esp8266com_index.json to the Additional Board Managers URL entry in Preferences.
// Compile for Adafruit HUZZAH Feather
#include <Eigen.h>     // Calls main Eigen matrix class library
#include <Eigen/LU>             // Calls inverse, determinant, LU decomp., etc.
using namespace Eigen;    // Eigen related statement; simplifies syntax for declaration of matrices

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

SoftwareSerial mySerial(RX, TX, false, 1024);
SoftEasyTransfer ETin, ETout;

// store readings
SensorReading readings;

// store settings
Command settings;

const byte NUM_LEDS = 12 + 20;
CRGBArray<NUM_LEDS> light0;
CRGBArray<NUM_LEDS> light1;
const byte NUM_RANGE = 4;
const byte NUM_POWER = NUM_LEDS - NUM_RANGE;

// FFT object
arduinoFFT FFT = arduinoFFT();

void setup(void)  {
  Serial.begin(115200);
  Serial << endl << endl << "Startup." << endl;

  pinMode(BLU_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(BLU_LED, LOW);

  // for remote output
  //  mySerial.begin(57600);
  mySerial.begin(115200);

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

  FastLED.addLeds<WS2811, L1, RGB>(light0, NUM_LEDS);
  FastLED.addLeds<WS2811, L2, RGB>(light1, NUM_LEDS);

  light0(0, NUM_LEDS - 1) = CRGB::White;
  light1(0, NUM_LEDS - 1) = CRGB::White;
  FastLED.show();
  delay(500);
  light0(0, NUM_LEDS - 1) = CRGB::Black;
  light1(0, NUM_LEDS - 1) = CRGB::Black;
  FastLED.show();

  digitalWrite(BLU_LED, HIGH);

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
    //    showRange();

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
const uint16_t samples = 16;  // This value MUST ALWAYS be a power of 2
// storage for frequency magnitudes and peak frequency
const uint16_t realLength = samples / 2;
typedef Matrix<uint16_t, N_SENSOR, realLength> MatrixFreq;
void print_mtx(const MatrixFreq & X);

/*
  Finally, the output of the FFT on real data has a few interesting properties:
    The very first bin (bin zero) of the FFT output represents the average power of the signal.
  Be careful not to try interpreting this bin as an actual frequency value!
   Only the first half of the output bins represent usable frequency values. This means the
  range of the output frequencies detected by the FFT is half of the sample rate. Don't try to
  interpret bins beyond the first half in the FFT output as they won't represent real frequency
  values!
*/

void computeFFT() {
  // storage
  static double vReal[N_SENSOR][samples];
  static double vImag[N_SENSOR][samples];

  // current buffer index
  static uint16_t cInd = 0;

  // store readings
  for ( byte i = 0; i < N_SENSOR; i++ ) {
    vReal[i][cInd] = readings.dist[i];
    vImag[i][cInd] = 0; // zero out the imaginary part as we go.
  }

  // increment buffer index
  cInd++;

  // check to see if we've filled the buffer
  if ( cInd != samples ) {
    // bail out if the buffer isn't full
    return;
  } else {
    // start at the top next time we add readings
    cInd = 0;
  }


  static uint16_t peakFreq[N_SENSOR], magFreq[N_SENSOR][realLength];

  // crunch the buffer; expensive
  static uint8_t exponent = FFT.Exponent(samples); // can precompute to save a little time.
  for ( byte i = 0; i < N_SENSOR; i++ ) {

    // show buffer
    //    Serial.println("Data:"); PrintVector(vReal[i], samples, SCL_TIME);

    // weigh data
    FFT.Windowing(vReal[i], samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
    yield();
    //    Serial.println("Weighed data:"); PrintVector(vReal[i], samples, SCL_TIME);

    // compute FFT
    FFT.Compute(vReal[i], vImag[i], samples, exponent, FFT_FORWARD); /* Compute FFT */
    yield();
    //    Serial.println("Computed Real values:"); PrintVector(vReal[i], samples, SCL_INDEX);
    //    Serial.println("Computed Imaginary values:"); PrintVector(vImag[i], samples, SCL_INDEX);

    // compute magnitudes
    FFT.ComplexToMagnitude(vReal[i], vImag[i], samples); /* Compute magnitudes */
    yield();
    //    Serial.println("Computed magnitudes:");  PrintVector(vReal[i], (samples >> 1), SCL_FREQUENCY);

    // magnitudes
    for ( uint16_t j = 0; j < realLength ; j++ ) magFreq[i][j] = vReal[i][j];
    //    Serial.print("Average power: "); Serial.println(magFreq[i][0], 6);

    // peak frequency
    peakFreq[i] = FFT.MajorPeak(vReal[i], samples, samplingFrequency);
  }
//  for ( uint16_t i = 2; i < realLength ; i++ ) Serial << magFreq[0][i] << ",";
//  Serial << endl;

  static uint16_t maxMag[N_SENSOR][realLength] = {0.0};
  static uint16_t scaledMag[N_SENSOR][realLength];

  for ( byte j = 0; j < N_SENSOR; j++ ) {
    for ( uint16_t i = 0; i < realLength ; i++ ) {
      // set constraints
      maxMag[j][i] = magFreq[j][i] > maxMag[j][i] ? magFreq[j][i] : maxMag[j][i];
      // scale
      scaledMag[j][i] = ((uint16_t)255 * magFreq[j][i] ) / maxMag[j][i];
      if ( j < 2 ) {
        light1[i + j * realLength] = j == 0 ? CRGB::Green : CRGB::Blue;
        light1[i + j * realLength] %= 255 - (byte)scaledMag[j][i];
      }
      maxMag[j][i] -= maxMag[j][i] > 0 ? 1 : 0; // keep the ceiling low.
    }
  }

  // map to matrix
  Map<MatrixFreq> mf((uint16_t*)&magFreq);
  // center matrix
  MatrixFreq centered = mf.rowwise() - mf.colwise().mean();
  // compute covariance matrix
  MatrixFreq cov = (centered.adjoint() * centered) / (mf.rows() - 1);
//  print_mtx(cov);

   // Mean
   // these are the typedefs I like to work with
typedef Matrix< double, Dynamic, 1, ColMajor > EVector;
typedef Matrix< double, Dynamic, Dynamic, ColMajor > EMatrix;


   Matrix<uint16_t, N_SENSOR, 1> mean = mf.colwise().mean;  
   for( byte i=0; i<mean.rows(); i++ ) Serial << mean(i) << ",";
   Serial << endl;
   
  // Standard Deviation
  //EVector std = (X.rowwise() - mean.transpose()).array().pow(2).colwise().sum() / X.rows();


  // Serial << cov(0,0) << "," << cov(1,1) << "," << cov(0,1) << endl;
  //  Serial << cov(0,1) << endl;

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
        abscissa = ((i * 1.0 * samplingFrequency) / (double)samples);
        break;
    }
    Serial.print(abscissa, 6);
    Serial.print(" ");
    Serial.print(vData[i], 4);
    Serial.println();

    // Only the first half of the output bins represent usable frequency values.
    if ( scaleType == SCL_FREQUENCY && i >= (bufferSize / 2 - 1) ) return;
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
void print_mtx(const MatrixFreq & X) {
   int i, j, nrow, ncol;
   
   nrow = X.rows();
   ncol = X.cols();

   Serial.print("nrow: "); Serial.println(nrow);
   Serial.print("ncol: "); Serial.println(ncol);       
   Serial.println();
   
   for (i=0; i<nrow; i++)
   {
       for (j=0; j<ncol; j++)
       {
           Serial.print(X(i,j), 6);   // print 6 decimal places
           Serial.print(", ");
       }
       Serial.println();
   }
   Serial.println();
}

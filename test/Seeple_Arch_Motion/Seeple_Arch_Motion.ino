// compile for WEMOS LOLIN32
#include <Eigen.h>     // Calls main Eigen matrix class library
#include <Eigen/LU>             // Calls inverse, determinant, LU decomp., etc.
using namespace Eigen;    // Eigen related statement; simplifies syntax for declaration of matrices
#include <Streaming.h>
#include <Metro.h>
#include "Nyctinasty_Messages.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include "Nyctinasty_Comms.h"
#include <arduinoFFT.h>

// pin definitions
// C:\Program Files (x86)\Arduino\hardware\espressif\esp32\variants\lolin32\pins_arduino.h
#define LED LED_BUILTIN // GPOIO5

// ADC on ESP32
// https://esp-idf.readthedocs.io/en/v2.0/api/peripherals/adc.html
uint8_t adc[N_SENSOR] = {
  A4, A5, A6, A7, A17, A16, A15, A14
  // 32, 33, 34, 35, 27,  14,  13,  13   <-  labels on topside of board
};

// track cycle time
unsigned long targetCycleTime = 20UL; // ms

// ship readings
DistanceReading readings;

// ship settings
SystemCommand settings;

// to go with output
String msg = "NULL";

// FFT object
arduinoFFT FFT = arduinoFFT();

// compute maximum cycle time from target fps
unsigned long cycleTime(byte fps) {
  settings.fps = fps;

  unsigned long time = 1000UL / (unsigned long)settings.fps; // ms
  Serial << F("Settings.  fps=") << settings.fps << F(" cycle time=") << time << F(" ms") << endl;

  return ( time );
}

void toggleLED() {
  static boolean LEDstate = false;
  LEDstate = !LEDstate;
  digitalWrite(LED, LEDstate);
}

const uint16_t samples = 256;  // This value MUST ALWAYS be a power of 2
static double buffer[N_SENSOR][samples];
boolean fillBuffer() {
  // track the active pin
  static byte sensor = 0;
  static uint16_t index = 0;

  // read
  buffer[sensor][index] = analogRead( adc[sensor] );

  // increment
  sensor++;
  if (sensor >= N_SENSOR) {
    sensor = 0;
    index++;
  }

  if ( index >= samples ) {
    index = 0;
    return ( true );
  }

  return ( false );
}

// send sensor readings
void dumpBuffer() {
  static uint16_t count = 0;

  const char sep = ',';

  for ( uint16_t j = 0; j < samples; j++) {
    for ( uint16_t i = 0; i < N_SENSOR; i++ ) {
      Serial << (uint16_t)buffer[i][j] << sep;
    }
    Serial << msg << sep << count << endl;
  }
  count ++;
}

void setup() {
  // for local output
  Serial.begin(115200);

  // configure ADC
  // C:\Program Files (x86)\Arduino\hardware\espressif\esp32\cores\esp32\esp32-hal-adc.h
  analogSetWidth(12); // sweet. 12-bit depth.
  analogSetAttenuation(ADC_6db); // gives full-scale voltage 2.2V, so we'll clip 2.75-2.2V
  analogSetCycles(8); // default=8

  settings.fps = defaultFPS;
  targetCycleTime = cycleTime(1);

  readings.min = 0;
  readings.max = 4095;
  readings.noise = 128;

  pinMode(LED, OUTPUT);
}

void loop() {
  // read ADCs
  static uint32_t tic = millis();
  if ( fillBuffer() ) {

    uint32_t toc = millis();
    Serial << endl << toc - tic << endl << endl;

    // flash the LED as we loop
    toggleLED();

    // dump the buffer
    dumpBuffer();

    tic = millis();
  }

  // NOT WORKING:
  // check for serial commands
  if ( Serial.available() ) {
    msg = Serial.readString();
  }

}


// https://cdn-learn.adafruit.com/downloads/pdf/fft-fun-with-fourier-transforms.pdf
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
// Nyquist's Sampling Theorem: only frequencies up to half the sampling rate can
// can be detected.  So, for 33 fps (Hz) we can detect signals no faster than 33/2 Hz.
double samplingFrequency = settings.fps;
//const uint16_t samples = 16;  // This value MUST ALWAYS be a power of 2
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


  /*
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


    //   Matrix<uint16_t, N_SENSOR, 1> mean = mf.colwise().mean;
    //   for( byte i=0; i<mean.rows(); i++ ) Serial << mean(i) << ",";
    //   Serial << endl;

    // Standard Deviation
    //EVector std = (X.rowwise() - mean.transpose()).array().pow(2).colwise().sum() / X.rows();


    // Serial << cov(0,0) << "," << cov(1,1) << "," << cov(0,1) << endl;
    //  Serial << cov(0,1) << endl;
  */

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


// Compile for Wemos D1 R2 & Mini
// keep an windward eye on dynamic memory usage: may need to go up to an ESP32
#include <Metro.h>
#include <Streaming.h>
#include <Eigen.h>
#include <Eigen/LU>
using namespace Eigen;    // simplifies syntax for declaration of matrices
#include <FastLED.h>      // seems to need to be after the namespace 
#include <EEPROM.h>
#include "Nyctinasty_Messages.h"
#include "Nyctinasty_Comms.h"

// who am I?
const byte SepalNumber = 0;
const String id = commsIdSepalCoordinator(SepalNumber);

// ship settings
SystemCommand settings;
// in this topic
const String settingsTopic = commsTopicSystemCommand();
// and sets this true when an update arrives
boolean settingsUpdate = false;

// our led updates send as this structure
SepalArchLight lights[N_ARCHES];
// in these topics
const String lightsTopic[N_ARCHES] = {
  commsTopicLight(SepalNumber, 0),
  commsTopicLight(SepalNumber, 1),
  commsTopicLight(SepalNumber, 2)
};

// our distance updates arrive as this structure
SepalArchDistance dist[N_ARCHES];
// in these topics
const String distTopic[N_ARCHES] = {
  commsTopicDistance(SepalNumber, 0),
  commsTopicDistance(SepalNumber, 1),
  commsTopicDistance(SepalNumber, 2)
};
// and sets this true when an update arrives
boolean distUpdate[N_ARCHES] = {false};

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
  commsSubscribe(settingsTopic, &settings, &settingsUpdate);
  //  commsSubscribe(settingsTopic, &settings, settingsUpdate);
  for ( byte i = 0; i < N_ARCHES; i++ ) {
    commsSubscribe(distTopic[i], &dist[i], &distUpdate[i]);
    commsSubscribe(freqTopic[i], &freq[i], &freqUpdate[i]);
    Serial << F("Publishing: ") << lightsTopic[i] << endl;
  }
  /*
    commsSubscribe(distTopic[0], &dist[0], distUpdate0);
    commsSubscribe(distTopic[1], &dist[1], distUpdate1);
    commsSubscribe(distTopic[2], &dist[2], distUpdate2);
    commsSubscribe(freqTopic[0], &freq[0], freqUpdate0);
    commsSubscribe(freqTopic[1], &freq[1], freqUpdate1);
    commsSubscribe(freqTopic[2], &freq[2], freqUpdate2);

    for ( byte i = 0; i < N_ARCHES; i++ ) {
      Serial << F("Publishing: ") << lightsTopic[i] << endl;
    }
  */
  Serial << F("Startup. complete.") << endl;
}

void mapDistanceToBar(byte index) {
  for ( byte i = 0; i < N_SENSOR; i++ ) {
    uint16_t intensity = map(
                           dist[index].prox[i] > dist[index].noise ? dist[index].prox[i] : 0,
                           dist[index].min, dist[index].max,
                           (uint16_t)0, (uint16_t)255
                         );
    lights[index].bar[i] = CHSV(HUE_BLUE, 0, (byte)intensity);
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
void loop() {
  // comms handling
  commsUpdate();

  // bail out if not connected
  if ( ! commsConnected() ) return;

  // order these by priority:

  // check for settings update
  if ( settingsUpdate ) {
    Serial << F("settingsUpdate.") << endl;
    settingsUpdate = false;
  }

  // check for an update to distance
  for ( byte i = 0; i < N_ARCHES; i++ ) {
    if ( distUpdate[i] ) {
      Serial << F("distUpdate ") << i << F(":") << millis() << endl;
      // map to lights
      mapDistanceToBar(i);
      // reset
      distUpdate[i] = false;
    }
  }

  // check for an update to frequency
  for ( byte i = 0; i < N_ARCHES; i++ ) {
    if ( freqUpdate[i] ) {
      Serial << F("freqUpdate ") << i << endl;
      // reset
      freqUpdate[i] = false;
    }
  }

  // simulate up and down, for now
  EVERY_N_MILLISECONDS( 25 ) {
    simulateUpDown();
  }

  EVERY_N_MILLISECONDS( 25 ) {
    // ship it, but bail out if no connection
    for ( byte i = 0; i < N_ARCHES; i++ ) {
      commsPublish(lightsTopic[i], &lights[i]);
    }
  }


}

// publishing this is the job of Sepal_Coordinator
void simulateUpDown() {
/*
  // adjust the bar
  static byte counter = 0;
  counter ++;
  for ( byte i = 0; i < N_ARCHES; i++ ) {
    CRGBSet bar(lights[i].bar, N_SENSOR);

    bar.fadeToBlackBy(128);
    bar[counter % bar.size()] = CRGB::White;
  }
*/
  // show a throbbing rainbow on the down segments
  static byte fadeBy = 0;
  static byte legHue = 0;
  static int legHueDelta = 255 / 8;
  static byte legBrightness = 32;
  for ( byte i = 0; i < N_ARCHES; i++ ) {
    CRGBSet leftDown(lights[i].leftDown, N_LEDS_DOWN);
    CRGBSet rightDown(lights[i].rightDown, N_LEDS_DOWN);

    leftDown.fill_rainbow(++legHue, legHueDelta); // paint
    leftDown.fadeToBlackBy(++fadeBy % 128);
    rightDown.fill_rainbow(legHue + 128, -legHueDelta); // paint, noting we're using the other side of the wheel
    rightDown.fadeToBlackBy((fadeBy + 128) % 128);
  }

  // trails
  // bpm (rate of dot travel) changes with time
  const byte bpm = 16; // 1/minute
  // fade everything
  for ( byte i = 0; i < N_ARCHES; i++ ) {
    CRGBSet leftUp(lights[i].leftUp, N_LEDS_UP);
    CRGBSet rightUp(lights[i].rightUp, N_LEDS_UP);

    leftUp.fadeToBlackBy(bpm);
    rightUp.fadeToBlackBy(bpm);
    // set the speed the pixel travels, see: lib8tion.h
    const uint16_t endUp = rightUp.size() - 1;
    uint16_t posVal = beatsin16(bpm, 0, (uint16_t)endUp);
    // cycle through hues
    static byte hue = 0;
    // paint
    leftUp[posVal] = CHSV(++hue, 255, 255);
    // mirrored direction and hue
    rightUp[endUp - posVal] = CHSV(hue + 128, 255, 255);
  }

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

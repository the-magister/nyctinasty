#include <Streaming.h>
#include <Metro.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "Skein_Comms.h"

//
// wiring
//
// drive the ADC board at 3.3Vcc, and drive the IR boards at 5.0Vcc.  
// IR boards will return a voltage <= 2.75V, which makes the gain=1 scale 
// on the ADC board perfect. 
//

// really need to save this to EEPROM
// should we ever need to extend this to more uCs, this will generate an offset.
byte subsetIndex = 1;

const byte Nsensor = 8;
boolean useSensor[Nsensor];

Adafruit_ADS1015 ads[2] = {
  Adafruit_ADS1015(ADS1015_ADDRESS),    // 0x48 (1001000) ADR -> GND
  Adafruit_ADS1015(ADS1015_ADDRESS+1)   // 0x49 (1001001) ADR -> VDD
};
word range[Nsensor];
const word outOfRange = (1 << 11) - 1; // coresponds to a reading that's out-of-range, 2047 mm.

// connect to the MQTT network with this id
String id = "skeinSensor" + subsetIndex;

// subscribe and process these topics
String control = "skein/control/#";

void setup(void) {
  Serial.begin(115200);
  Serial.println("Hello!");
  
  Serial.println("Getting single-ended readings...");
  Serial.println("ADC Range: 1x gain   +/- 4.096V  1 bit = 2mV      ");
  
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads[0].setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  ads[1].setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
 
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  // don't use these.  just calls Wire.begin();
//  ads[0].begin();
//  ads[1].begin();
  Wire.begin();

  commsBegin(id);
  commsSubscribe(control);

}

void loop(void) {
  // comms handling
  commsUpdate();

  // bail out if no connection
  if ( ! commsConnected() ) return;

  // sensor handling

  word adc[8]; // 12 bit readings

//  adc[0] = distance(ads[0].readADC_SingleEnded(0));
  adc[0] = ads[0].readADC_SingleEnded(0); yield();
  adc[1] = ads[0].readADC_SingleEnded(1); yield();
  adc[2] = ads[0].readADC_SingleEnded(2); yield();
  adc[3] = ads[0].readADC_SingleEnded(3); yield();
  Serial << adc[0] << "," << adc[1] << "," << adc[2] << "," << adc[3];

  adc[4] = adc[5] = adc[6] = adc[7] = 0;
/*
  adc[4] = ads[1].readADC_SingleEnded(0); yield();
  adc[5] = ads[1].readADC_SingleEnded(1); yield();
  adc[6] = ads[1].readADC_SingleEnded(2); yield();
  adc[7] = ads[1].readADC_SingleEnded(3); yield();
  Serial << "," << adc[4] << "," << adc[5] << "," << adc[6] << "," << adc[7];
*/  
  Serial << endl;

  for(byte i=0; i<4; i++ ) {
    range[i] = distance(adc[i]);
    publishRange(i); yield();
  }
  for(byte i=4; i<Nsensor; i++ ) {
    range[i] = outOfRange;
    publishRange(i); yield();
  }
  delay(40); // read cyles are ~ 40ms, so no point in sampling more frequently.

  // send oor informaton once every 30 seconds
  static Metro oorPubInterval(30UL * 1000UL);
  if ( commsConnected() && oorPubInterval.check() ) {
    commsPublish("skein/range/oor", String(outOfRange, 10));
    oorPubInterval.reset();
  }

}

// take an ADC reading and return distance, in mm.
float distance(word reading) {
  // 1 bit = 2mV
  const float mVPerBit = 2.0;
  float voltage = (float)reading*mVPerBit / 1000.0;
  
  if( voltage < 0.4 ) voltage = 0.4; // edge case, and protect from div0
  return( ( 73.2242*(1.0/voltage) - 9.0036)*10.0 ); // mm
}


void publishRange(byte index) {

  String topic = "skein/range/" + String(subsetIndex, 10) + "/" + String(index, 10);
  String message = String(range[index], 10);

  commsPublish(topic, message);
}

void commsProcess(String topic, String message) {
  Serial << "<- " << topic << " " << message << "\t=> ";

  Serial << F("WARNING. unknown topic. continuing.");

  Serial << endl;
}


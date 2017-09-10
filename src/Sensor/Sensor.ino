/*
   This module is responsible for polling the LIDAR sensors and publishing that data.
   We can have multiple sensors, and this is the i-th sensor.

   Publishes: skein/range/i/[0-7]: distance information, mm
              skein/range/oor: distance corresponding to out-of-range, mm
*/

// You'll need to add http://arduino.esp8266.com/stable/package_esp8266com_index.json to the Additional Board Managers URL entry in Preferences.
// Compile for NodeMCU 1.0 (ESP-12E Module), 80 Mhz, 921600 Upload Speed, 4M (3M SPIFFS).
#include <Streaming.h>
#include <Metro.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "Skein_Comms.h"

// really need to save this to EEPROM
// should we ever need to extend this to more uCs, this will generate an offset.
byte subsetIndex = 0;

const byte Nsensor = 8;
VL53L0X sensor[] = {
  VL53L0X(), VL53L0X(), VL53L0X(), VL53L0X(),
  VL53L0X(), VL53L0X(), VL53L0X(), VL53L0X()
};
boolean useSensor[Nsensor];
word range[Nsensor];
const word outOfRange = (1 << 11) - 1; // coresponds to a reading that's out-of-range, 2047 mm.
const boolean LONG_RANGE = true;
const boolean HIGH_SPEED = false;
const boolean HIGH_ACCURACY = true;

const int measurePeriod = 50;

// connect to the MQTT network with this id
String id = "skeinSensor" + subsetIndex;

// subscribe and process these topics
String control = "skein/control/#";

void setup(void)  {
  Serial.begin(115200);
  delay(10);
  Serial << endl << endl << "Startup." << endl;

  // reset the I2C expander
  Serial << "Startup.  resetting I2C expander." << endl;
  const byte resetPin = 14;
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, LOW);
  delay(100);
  digitalWrite(resetPin, HIGH);
  Serial << "Startup.  reset I2C expander." << endl;

  // initialize the sensors
  Serial << "Startup. Wire start." << endl;
  Wire.begin(); // SDA=GPIO4=D2; SCL=GPIO5=D1
  //delay(100);
  Serial << "Startup. Wire complete." << endl;

  for ( byte i = 0; i < Nsensor; i++) {
//    if (i == 5 || i == 6) continue;
    if (i == 5) continue;
    
    Serial << "Startup. Initializing sensor " << i << endl;
    selectSensor(i);
    delay(100);
    Serial << "Startup. sensor selected " << i << endl;
    //delay(100);

    useSensor[i] = sensor[i].init();
    Serial << "Startup. sensor initialized " << i << " with return=" << useSensor[i] << endl;
    //delay(100);
    if ( !useSensor[i] ) continue;

    sensor[i].setTimeout(500);
    Serial << "Startup. sensor timeout set " << i << endl;
    //delay(100);

    if ( LONG_RANGE ) {
      // lower the return signal rate limit (default is 0.25 MCPS)
      sensor[i].setSignalRateLimit(0.1); yield();
      // increase laser pulse periods (defaults are 14 and 10 PCLKs)
      sensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18); yield();
      sensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14); yield();
    }
    if ( HIGH_SPEED ) {
      // reduce timing budget to 20 ms (default is about 33 ms)
      sensor[i].setMeasurementTimingBudget(20000); yield();

    } else if ( HIGH_ACCURACY ) {
      // increase timing budget to 100 ms
      sensor[i].setMeasurementTimingBudget(100UL * 1000UL); yield();

    }
    delay(100);
  }


  // Start continuous back - to - back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  if ( useSensor[0] ) {
    selectSensor(0);
    sensor[0].startContinuous(measurePeriod);
    delay(100);
  }
  if ( useSensor[2] ) {
    selectSensor(2);
    sensor[2].startContinuous(measurePeriod);
    delay(100);
  }
  if ( useSensor[4] ) {
    selectSensor(4);
    sensor[4].startContinuous(measurePeriod);
    delay(100);
  }
  if ( useSensor[6] ) {
    selectSensor(6);
    sensor[6].startContinuous(measurePeriod);
    delay(100);
  }
  
  if ( useSensor[1] ) {
    selectSensor(1);
    sensor[1].startContinuous(measurePeriod);
    delay(100);
  }
  if ( useSensor[3] ) {
    selectSensor(3);
    sensor[3].startContinuous(measurePeriod);
    delay(100);
  }
  if ( useSensor[5] ) {
    selectSensor(5);
    sensor[5].startContinuous(measurePeriod);
    delay(100);
  }
  if ( useSensor[7] ) {
    selectSensor(7);
    sensor[7].startContinuous(measurePeriod);
    delay(100);
  }


  Serial << "Startup. continuous sensing started." << endl;



  commsBegin(id);
  commsSubscribe(control);

}

int count = 0;
void loopOld(void) {
  count++;
  if (count % 100) {
    Serial << ".";
  }
  delay(100);
}

void loop(void) {
  // comms handling
  commsUpdate();

  // bail out if no connection
  if ( ! commsConnected() ) return;

  // sensor handling

  // poll the sensors
  static byte index = 99;
  index ++;
  if ( index >= Nsensor ) {
    index = 0;
    Serial << endl;
  }
  if ( !useSensor[index] ) return; // bail out and go again for the next sensor

  selectSensor(index);
  range[index] = sensor[index].readRangeContinuousMillimeters();
  //range[index] = sensor[index].readRangeSingleMillimeters();

  if (range[index] >= outOfRange) {
    range[index] = outOfRange;
  }

  Serial << range[index] << ",";


  // publish
  publishRange(index);

  // send oor information once every 30 seconds
  static Metro oorPubInterval(30UL * 1000UL);
  if ( commsConnected() && oorPubInterval.check() ) {
    commsPublish("skein/range/oor", String(outOfRange, 10));
    oorPubInterval.reset();
  }
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

// handy helper
void selectSensor(uint8_t i) {
  const byte addr = 0x70;
  if (i > 7) return;

  Wire.beginTransmission(addr);
  Wire.write(1 << i);
  Wire.endTransmission();
}


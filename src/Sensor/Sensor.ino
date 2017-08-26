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

// really need to save this to EEPROM
// should we ever need to extend this to more uCs, this will generate an offset.
byte subsetIndex = 0; 

const byte Nsensor = 8;
VL53L0X sensor[] = {
  VL53L0X(), VL53L0X(), VL53L0X(), VL53L0X(),
  VL53L0X(), VL53L0X(), VL53L0X(), VL53L0X()
};
word range[Nsensor];
const word outOfRange = (1<<11)-1; // coresponds to a reading that's out-of-range, 2047 mm.
const boolean LONG_RANGE = true;
const boolean HIGH_SPEED = false;
const boolean HIGH_ACCURACY = true;

WiFiClient espClient;
PubSubClient mqtt;

// labels for the pins on the NodeMCU are weird:
// "C:\Users\MikeD\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\2.3.0\variants\nodemcu\pins_arduino.h"
#define BLUE_LED 2 // labeled "D4" on NodeMCU boards; HIGH=off
#define RED_LED 16 // labeled "D0" on NodeMCU boards; HIGH=off

void setup(void)  {
  Serial.begin(115200);
  Serial << "Startup." << endl;

  // initialize the sensors
  Wire.begin(); // SDA=GPIO4=D2; SCL=GPIO5=D1
  for ( byte i = 0; i < Nsensor; i++) {
    selectSensor(i);

    sensor[i].init();
    sensor[i].setTimeout(100);

    if ( LONG_RANGE ) {
      // lower the return signal rate limit (default is 0.25 MCPS)
      sensor[i].setSignalRateLimit(0.1);
      // increase laser pulse periods (defaults are 14 and 10 PCLKs)
      sensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
      sensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    }
    if ( HIGH_SPEED ) {
      // reduce timing budget to 20 ms (default is about 33 ms)
      sensor[i].setMeasurementTimingBudget(20000);

    } else if ( HIGH_ACCURACY ) {
      // increase timing budget to 100 ms
      sensor[i].setMeasurementTimingBudget(100UL * 1000UL);

    }
    // Start continuous back-to-back mode (take readings as
    // fast as possible).  To use continuous timed mode
    // instead, provide a desired inter-measurement period in
    // ms (e.g. sensor.startContinuous(100)).
    sensor[i].startContinuous(50);

  }

  pinMode(BLUE_LED, OUTPUT);  // Shows sensor activity; flashing with polling
  pinMode(RED_LED, OUTPUT);   // Shows WiFi activity; solid on when there's an issue; flashing with MQTT messages

  // don't allow the WiFi module to sleep.
  WiFi.setSleepMode(WIFI_NONE_SLEEP);

  mqtt.setClient(espClient);
  const char* mqtt_server = "broker";
  mqtt.setServer(mqtt_server, 1883);
  mqtt.setCallback(callback);

}

void loop(void) {
  // comms handling
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  } else if (!mqtt.connected()) {
    connectMQTT();
  } else {
    mqtt.loop(); // look for a message
  }

  // sensor handling

  // track the time
  unsigned long tic = millis();

  // poll the sensors
  for ( byte i = 0; i < Nsensor; i++) {
    // toggle the BLUE LED when we poll the sensors
    static boolean ledState = false;
    ledState = !ledState;
    digitalWrite(BLUE_LED, ledState);

    selectSensor(i);
    range[i] = sensor[i].readRangeContinuousMillimeters();

    // publish
    publishRange(i);

    Serial << range[i] << ",";
  }
  Serial << endl;

  unsigned long toc = millis();
  Serial << "Ranging Duration (ms): " << toc - tic << "\t fps (Hz):" << 1000 / (toc - tic) << endl;
}

void publishRange(byte index) {
  if ( !mqtt.connected() ) return;

  // toggle the RED LED when we send a message
  static boolean ledState = false;
  ledState = !ledState;
  digitalWrite(RED_LED, ledState);

  String topic = "skein/range/" + String(subsetIndex,10) + "/" + String(index,10);
  String message = String(range[index], 10);

  mqtt.publish(topic.c_str(), message.c_str());
}

// the real meat of the work is done here, where we process messages.
void callback(char* topic, byte* payload, unsigned int length) {
  // toggle the RED LED when we get a new message
  static boolean ledState = false;
  ledState = !ledState;
  digitalWrite(RED_LED, ledState);

  // String class is much easier to work with
  String t = topic;
  Serial << F("<- ") << t;

  // list of topics that we'll process
  const String msgExample = "skein/control/foo";

  if ( t.equals(msgExample) ) {
    byte state = payload[0];
    Serial << F(" = ") << state;
  } else {
    Serial << F(" WARNING. unknown topic. continuing.");
  }

  Serial << endl;
}

void connectWiFi() {
  // turn on the RED LED when we're not connected
  digitalWrite(RED_LED, LOW);

  // Update these.
  const char* ssid = "GamesWithFire";
  const char* password = "safetythird";

  static Metro connectInterval(5000UL);
  if ( connectInterval.check() ) {

    Serial << F("Attempting WiFi connection to ") << ssid << endl;
    // Attempt to connect
    WiFi.begin(ssid, password);

    connectInterval.reset();
  }
}

void connectMQTT() {
  // turn on the RED LED when we're not connected
  digitalWrite(RED_LED, LOW);

  String id = "skeinSensor" + subsetIndex;
  const char* sub = "skein/control/#";

  static Metro connectInterval(500UL);
  if ( connectInterval.check() ) {

    Serial << F("Attempting MQTT connection...") << endl;
    // Attempt to connect
    if (mqtt.connect(id.c_str())) {
      Serial << F("Connected.") << endl;
      // subscribe
      Serial << F("Subscribing: ") << sub << endl;
      mqtt.subscribe(sub);

      String message = String(outOfRange, 10);
      Serial << F("Publishing oor: ") << message << endl;
      mqtt.publish("skein/range/oor", message.c_str(), true); // retained

      // turn off the RED LED when we're connected
      digitalWrite(RED_LED, HIGH);

    } else {
      Serial << F("Failed. state=") << mqtt.state() << endl;
    }

    connectInterval.reset();
  }
}

// handy helper
void selectSensor(uint8_t i) {
  const byte addr = 0x70;
  if (i > 7) return;

  Wire.beginTransmission(addr);
  Wire.write(1 << i);
  Wire.endTransmission();
}


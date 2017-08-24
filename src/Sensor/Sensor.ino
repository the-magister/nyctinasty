/*
 * This module is responsible for polling the LIDAR sensors and publishing that data
 * 
 * Publishes: skein/range/[0-7]: distance information, mm
 *            skein/range/oor: distance corresponding to out-of-range, mm
 */
 
// You'll need to add http://arduino.esp8266.com/stable/package_esp8266com_index.json to the Additional Board Managers URL entry in Preferences.
// Compile for NodeMCU 1.0 (ESP-12E Module), 80 Mhz, 921600 Upload Speed, 4M (3M SPIFFS). 
#include <Streaming.h>
#include <Metro.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define NLOX 8
Adafruit_VL53L0X lox[NLOX] = {
  Adafruit_VL53L0X(), Adafruit_VL53L0X(), Adafruit_VL53L0X(), Adafruit_VL53L0X(),
  Adafruit_VL53L0X(), Adafruit_VL53L0X(), Adafruit_VL53L0X(), Adafruit_VL53L0X() 
};
VL53L0X_RangingMeasurementData_t range[NLOX];
const int outOfRange = -1; // coresponds to a reading that's out-of-range
const byte topicOffset = 0; // should we ever need to extend this to more uCs, this will generate an offset.

WiFiClient espClient;
PubSubClient mqtt;

// pinouts on the NodeMCU:
// "C:\Users\MikeD\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\2.3.0\variants\nodemcu\pins_arduino.h"
#define BLUE_LED D4 // HIGH=off
#define RED_LED D0 // HIGH=off

// handy helper 
void selectLOX(uint8_t i) {
  const byte addr = 0x70;
  if (i > 7) return;
 
  Wire.beginTransmission(addr);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup(void)  {
  Serial.begin(115200);
  Serial << "Startup." << endl;

  // initialize the sensors
  Wire.begin(SDA, SCL); // SDA=GPIO4=D2; SCL=GPIO5=D1
  for( byte i=0;i<NLOX;i++) {
    selectLOX(i);
    if( ! lox[i].begin() ) {
      Serial << "FAILED to initialize LOX " << i << endl;
    } else {
      Serial << "Initialized LOX " << i << endl;
    }
  }

  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED pin as an output

  // don't allow the WiFi module to sleep.  
  WiFi.setSleepMode(WIFI_NONE_SLEEP);

  mqtt.setClient(espClient);
//  const char* mqtt_server = "broker.mqtt-dashboard.com";
  const char* mqtt_server = "asone-console";
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
  for( byte i=0;i<NLOX;i++) {
    Serial << "Ranging " << i << ": ";

    digitalWrite(LED_BUILTIN, HIGH);
    selectLOX(i); // select the sensor
    lox[i].rangingTest(&range[i], false); // ask it to range once
    digitalWrite(LED_BUILTIN, LOW);
    
    if( range[i].RangeStatus != 4 ) {
      Serial << range[i].RangeMilliMeter << " mm." << endl;  
    } else {
      Serial << "out of range." << endl;
    }
    
    // publish
    publishRange(i);
  }
  unsigned long toc = millis();
  Serial << "Ranging Duration (ms): " << toc-tic << "\t fps (Hz):" << 1000/(toc-tic) << endl;
  
}

void publishRange(byte index) {
  if( !mqtt.connected() ) return;
  
  String topic = "skein/range/" + (index+(NLOX*topicOffset));

  int r = outOfRange;
  if( range[index].RangeStatus != 4 ) r = range[index].RangeMilliMeter;
  String message = String(r,10);

  mqtt.publish(topic.c_str(), message.c_str());
}

// the real meat of the work is done here, where we process messages.
void callback(char* topic, byte* payload, unsigned int length) {
  // toggle the LED when we get a new message
  static boolean ledState = false;
  ledState = !ledState;
  digitalWrite(LED_BUILTIN, ledState);

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
  digitalWrite(LED_BUILTIN, HIGH);

  // Update these.
//  const char* ssid = "Looney_Ext";
//  const char* password = "TinyandTooney";
  const char* ssid = "AsOne";
  const char* password = "fuckthapolice";

  static Metro connectInterval(500UL);
  if ( connectInterval.check() ) {

    Serial << F("Attempting WiFi connection to ") << ssid << endl;
    // Attempt to connect
    WiFi.begin(ssid, password);

    connectInterval.reset();
  }
}


void connectMQTT() {
  digitalWrite(LED_BUILTIN, HIGH);

  const char* id = "skeinSensor";
  const char* sub = "skein/control/#";

  static Metro connectInterval(500UL);
  if ( connectInterval.check() ) {

    Serial << F("Attempting MQTT connection...") << endl;
    // Attempt to connect
    if (mqtt.connect(id)) {
      Serial << F("Connected.") << endl;
      // subscribe
      Serial << F("Subscribing: ") << sub << endl;
      mqtt.subscribe(sub);

      String message = String(outOfRange,10);
      Serial << F("Publishing oor: ") << message << endl;
      mqtt.publish("skein/range/oor", message.c_str(), true); // retained
      
      digitalWrite(LED_BUILTIN, LOW);

    } else {
      Serial << F("Failed. state=") << mqtt.state() << endl;
    }

    connectInterval.reset();
  }
}


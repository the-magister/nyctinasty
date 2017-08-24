/*
 * This module is responsible for running lighting based on the Lidar sensor
 * 
 * Subscribes: skein/range/#
 */
 
// You'll need to add http://arduino.esp8266.com/stable/package_esp8266com_index.json to the Additional Board Managers URL entry in Preferences.
// Compile for NodeMCU 1.0 (ESP-12E Module), 80 Mhz, 921600 Upload Speed, 4M (3M SPIFFS). 
#include <Streaming.h>
#include <Metro.h>
#include <FastLED.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define NLOX 8
int range[NLOX];
int outOfRange; // coresponds to a reading that's out-of-range

WiFiClient espClient;
PubSubClient mqtt;

// pinouts on the NodeMCU:
// "C:\Users\MikeD\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\2.3.0\variants\nodemcu\pins_arduino.h"

void setup(void)  {
  Serial.begin(115200);
  Serial << "Startup." << endl;

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

  // lights handling

  // do stuff with FastLED to map range[] to lights
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
  const String msgOOR = "skein/range/oor";
  const String msgRange = "skein/range/";

  if ( t.equals(msgOOR) ) {
    outOfRange = String((char*)payload).toInt();
    Serial << F(" = ") << outOfRange;
  } else if ( t.startsWith(msgRange) ) {
    t.remove(0, msgRange.length());
    byte i = t.toInt();
    byte m = String((char*)payload).toInt();
    range[i] = m;
    Serial << F(" = ") << i << ": " << range[i];
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

  const char* id = "skeinLights";
  const char* sub = "skein/range/#";

  static Metro connectInterval(500UL);
  if ( connectInterval.check() ) {

    Serial << F("Attempting MQTT connection...") << endl;
    // Attempt to connect
    if (mqtt.connect(id)) {
      Serial << F("Connected.") << endl;
      // subscribe
      Serial << F("Subscribing: ") << sub << endl;
      mqtt.subscribe(sub);

      digitalWrite(LED_BUILTIN, LOW);

    } else {
      Serial << F("Failed. state=") << mqtt.state() << endl;
    }

    connectInterval.reset();
  }
}


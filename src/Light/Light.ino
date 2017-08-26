/*
 * This module is responsible for running lighting based on the Lidar sensor
 * 
 * Subscribes: skein/range/i/#
 */
 
// You'll need to add http://arduino.esp8266.com/stable/package_esp8266com_index.json to the Additional Board Managers URL entry in Preferences.
// Compile for NodeMCU 1.0 (ESP-12E Module), 80 Mhz, 921600 Upload Speed, 4M (3M SPIFFS). 
#include <Streaming.h>
#include <Metro.h>
#include <FastLED.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// really need to save this to EEPROM
// should we ever need to extend this to more uCs, this will generate an offset.
const byte subsetIndex = 0; 

const byte Nsensor = 8;
word range[Nsensor];
word outOfRange = (1<<11)-1; // coresponds to a reading that's out-of-range

// lighting
byte value[Nsensor];
// linearize perception to value
float R = (outOfRange)/log2(255+1);

WiFiClient espClient;
PubSubClient mqtt;

// labels for the pins on the NodeMCU are weird:
// "C:\Users\MikeD\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\2.3.0\variants\nodemcu\pins_arduino.h"
#define BLUE_LED 2 // labeled "D4" on NodeMCU boards; HIGH=off
#define RED_LED 16 // labeled "D0" on NodeMCU boards; HIGH=off

void setup(void)  {
  Serial.begin(115200);
  Serial << "Startup." << endl;

  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED pin as an output

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

  // lights handling

  // do stuff with FastLED to map range[] to lights
}

// the real meat of the work is done here, where we process messages.
void callback(char* topic, byte* payload, unsigned int length) {
  // toggle the LED when we get a new message
  static boolean ledState = false;
  ledState = !ledState;
  digitalWrite(RED_LED, ledState);

  // String class is much easier to work with
  String t = topic;
  Serial << F("<- ") << t;

  // list of topics that we'll process
  const String msgOOR = "skein/range/oor";
  static String msgRange = "skein/range/" + String(subsetIndex,10) + "/";

  if ( t.equals(msgOOR) ) {
    outOfRange = String((char*)payload).toInt();
    R = (outOfRange)/log2(255+1);

    Serial << F(" = ") << outOfRange;
  } else if ( t.startsWith(msgRange) ) {
    t.remove(0, msgRange.length());
    byte i = t.toInt();
    word m = String((char*)payload).toInt();
    
    range[i] = m;
    // see: https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms/
    value[i] = round( pow((float)range[i]/R, 2.0)-1.0 );

    Serial << F(" = ") << i << ": " << range[i];
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
  digitalWrite(RED_LED, LOW);

  String id = "skeinLight" + subsetIndex;
  String sub = "skein/range/" + String(subsetIndex,10) + "/#";

  static Metro connectInterval(500UL);
  if ( connectInterval.check() ) {

    Serial << F("Attempting MQTT connection...") << endl;
    // Attempt to connect
    if (mqtt.connect(id.c_str())) {
      Serial << F("Connected.") << endl;
      // subscribe
      Serial << F("Subscribing: ") << sub << endl;
      mqtt.subscribe(sub.c_str());

      digitalWrite(RED_LED, HIGH);

    } else {
      Serial << F("Failed. state=") << mqtt.state() << endl;
    }

    connectInterval.reset();
  }
}


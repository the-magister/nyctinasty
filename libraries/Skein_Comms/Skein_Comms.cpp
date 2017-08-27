#include "Skein_Comms.h"

WiFiClient espClient;
PubSubClient mqtt;

String myID;

const byte maxTopics = 8;
byte nTopics = 0;
String topics[maxTopics];

// labels for the pins on the NodeMCU are weird:
// "C:\Users\MikeD\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\2.3.0\variants\nodemcu\pins_arduino.h"
#define BLUE_LED 2 // labeled "D4" on NodeMCU boards; HIGH=off
#define RED_LED 16 // labeled "D0" on NodeMCU boards; HIGH=off

void commsBegin(String id) {
  // don't allow the WiFi module to sleep.  
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.disconnect();
  
  myID = id;
  mqtt.setClient(espClient);
  mqtt.setCallback(commsCallback);

  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
}

boolean commsConnected() {
	mqtt.connected();
}

// the real meat of the work is done here, where we process messages.
void commsCallback(char* topic, byte* payload, unsigned int length) {
  // toggle the LED when we get a new message
  static boolean ledState = false;
  ledState = !ledState;
  digitalWrite(RED_LED, ledState);

  // String class is much easier to work with
  String t = topic;
  String m = String( (char*)payload).substring(0,length);

  commsProcess(t, m);
}

void commsUpdate() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  } else if (!mqtt.connected()) {
    connectMQTT();
  } else {
    mqtt.loop(); // look for a message
  }
}

void commsSubscribe(String topic) {
  topics[nTopics++] = topic;
}

void commsPublish(String topic, String message, boolean showPub) {
  if ( !mqtt.connected() ) return;
  
  // toggle the RED LED when we send a message
  static boolean ledState = false;
  ledState = !ledState;
  digitalWrite(RED_LED, ledState);

  mqtt.publish(topic.c_str(), message.c_str());
  
  if( showPub ) {
    Serial << "-> " << topic << " " << message << endl;
  }
}

void connectWiFi(String ssid, String passwd, unsigned long interval) {
  // solid red light while not connected to WiFi or MQTT
  digitalWrite(RED_LED, LOW);

  static Metro connectInterval(interval);
  if ( connectInterval.check() ) {

    Serial << F("Attempting WiFi connection to ") << ssid << endl;
    // Attempt to connect
    WiFi.begin(ssid.c_str(), passwd.c_str());

    connectInterval.reset();
  }
}

void connectMQTT(String broker, word port, unsigned long interval) {
  // solid red light while not connected to WiFi or MQTT
  digitalWrite(RED_LED, LOW);

  static Metro connectInterval(interval);
  if ( connectInterval.check() ) {

    Serial << F("Attempting MQTT connection as ") << myID << " to " << broker << ":" << port << endl;
    mqtt.setServer(broker.c_str(), port);

    // Attempt to connect
    if (mqtt.connect(myID.c_str())) {
      Serial << F("Connected.") << endl;

      // (re)subscribe
      for(byte i=0; i<nTopics; i++) {
        Serial << F("Subscribing: ") << topics[i] << endl;
        mqtt.subscribe(topics[i].c_str());
      }

      // turn off the RED led with WiFi and MQTT connection
      digitalWrite(RED_LED, HIGH);
    } else {
      Serial << F("Failed. state=") << mqtt.state() << endl;
      Serial << F("WiFi status connected? ") << (WiFi.status()==WL_CONNECTED) << endl;
    }

    connectInterval.reset();
  }
}


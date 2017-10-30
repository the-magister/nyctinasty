#include "Skein_MQTT.h"

WiFiClient espClient;
PubSubClient mqtt;

// save
String myID;
byte myLED;
boolean ledState = false;

const byte maxTopics = 8;
byte nTopics = 0;
String subTopic[maxTopics];
void * subStorage[maxTopics];
boolean * subUpdate[maxTopics];

// labels for the pins on the NodeMCU are weird:
// "C:\Users\MikeD\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\2.3.0\variants\nodemcu\pins_arduino.h"
#define BLUE_LED 2 // labeled "D4" on NodeMCU boards; HIGH=off
#define RED_LED 16 // labeled "D0" on NodeMCU boards; HIGH=off

void commsBegin(String id, byte ledPin) {
	Serial << "commsBegin with id: " << id << endl;
	
	// don't allow the WiFi module to sleep.	
	WiFi.setSleepMode(WIFI_NONE_SLEEP);
	WiFi.disconnect();
	
	myID = id;
	mqtt.setClient(espClient);
	mqtt.setCallback(commsCallback);

	myLED = ledPin;
	pinMode(myLED, OUTPUT);
}

boolean commsConnected() {
	(mqtt.connected()==true) && (WiFi.status()==WL_CONNECTED);
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

// the real meat of the work is done here, where we process messages.
void commsCallback(char* topic, byte* payload, unsigned int length) {
	// toggle the LED when we get a new message
	ledState = !ledState;
	digitalWrite(myLED, ledState);

	// String class is much easier to work with
	String t = topic;
	
	// run through topics we're subscribed to
	for( byte i=0;i < nTopics;i++ ) {
		if( t.equals(subTopic[i]) ) {
			memcpy( subStorage[i], (void*)payload, length );
			*subUpdate[i] = true;
			return;
		}
	}
}


void commsSubscribe(String topic, SensorReading * reading, boolean * updateFlag) {
	commsSubscribe(topic, (void *) reading, updateFlag);
}
void commsSubscribe(String topic, Command * command, boolean * updateFlag) {
	commsSubscribe(topic, (void *) command, updateFlag);
}
void commsSubscribe(String topic, void * storage, boolean * updateFlag) {
	subTopic[nTopics++] = topic;
	subStorage[nTopics] = storage;
	subUpdate[nTopics] = updateFlag; 
}

// publish to a topic
boolean commsPublish(String topic, SensorReading * reading) {
	// toggle the LED when we get a new message
	ledState = !ledState;
	digitalWrite(myLED, ledState);

	return mqtt.publish(topic.c_str(), (uint8_t *)reading, sizeof(reading));
}
boolean commsPublish(String topic, Command * command) {
	// toggle the LED when we get a new message
	ledState = !ledState;
	digitalWrite(myLED, ledState);

	return mqtt.publish(topic.c_str(), (uint8_t *)command, sizeof(command));
}

void connectWiFi(String ssid, String passwd, unsigned long interval) {
	static Metro connectInterval(interval);
	if ( connectInterval.check() ) {

		Serial << F("Attempting WiFi connection to ") << ssid << endl;
		// Attempt to connect
		WiFi.begin(ssid.c_str(), passwd.c_str());

		connectInterval.reset();
	}
}

void connectMQTT(String broker, word port, unsigned long interval) {
	static Metro connectInterval(interval);
	if ( connectInterval.check() ) {

		Serial << F("Attempting MQTT connection as ") << myID << " to " << broker << ":" << port << endl;
		mqtt.setServer(broker.c_str(), port);

		// Attempt to connect
		if (mqtt.connect(myID.c_str())) {
			Serial << F("Connected.") << endl;

			// (re)subscribe
			for(byte i=0; i<nTopics; i++) {
				Serial << F("Subscribing: ") << subTopic[i] << endl;
				mqtt.subscribe(subTopic[i].c_str());
			}

		} else {
			Serial << F("Failed. state=") << mqtt.state() << endl;
			Serial << F("WiFi status connected? ") << (WiFi.status()==WL_CONNECTED) << endl;
		if( mqtt.state()==MQTT_CONNECT_FAILED ) {
				Serial << "Restarting WiFi..." << endl;
				commsBegin(myID, myLED);
			}
		}

		connectInterval.reset();
	}
}


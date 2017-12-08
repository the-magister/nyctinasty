#include "Nyctinasty_Comms.h"

WiFiClient espClient;
PubSubClient mqtt;

// save
String myID;
byte myLED;
boolean ledState = false;

const byte maxTopics = 32;
byte nTopics = 0;
String subTopic[maxTopics];
void * subStorage[maxTopics];
boolean * subUpdate[maxTopics];

// Id handling
const String preface = "nyc";
const String nameSep = "/";
const String topicSep = "/";
const String settingsFunction = "Settings";
const String coordinatorFunction = "Coordinator";
const String lightFunction = "Light";
const String motionFunction = "Motion";
const String freqFunction = "Freq";

String commsIdSeepleCoordinator(byte seepleNumber) {
	return(
		preface + nameSep + 
			coordinatorFunction + nameSep + 
				String(seepleNumber,10)  
	);
}
String commsIdSeepleArchLight(byte seepleNumber, byte archNumber) {
	return( 
		preface + nameSep + 
			lightFunction + nameSep + 
				String(seepleNumber,10) + nameSep + 
					String(archNumber,10)
	);
}
String commsIdSeepleArchMotion(byte seepleNumber, byte archNumber) {
	return( 
		preface + nameSep + 
			motionFunction + nameSep + 
				String(seepleNumber,10) + nameSep + 
					String(archNumber,10)
	);

}

void commsBegin(String id, byte ledPin) {
	Serial << "commsBegin with id: " << id << endl;
	
	// don't allow the WiFi module to sleep.	
//	WiFi.setSleepMode(WIFI_NONE_SLEEP);
	WiFi.disconnect();
	
	myID = id;
	mqtt.setClient(espClient);
	mqtt.setCallback(commsCallback);

	myLED = ledPin;
	pinMode(myLED, OUTPUT);

}

void toggleLED() {
	// toggle the LED when we get a new message
	ledState = !ledState;
	digitalWrite(myLED, ledState);
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

	// String class is much easier to work with
	String t = topic;
//	Serial << topic << " received." << endl;
	
	// run through topics we're subscribed to
	for( byte i=0;i < nTopics;i++ ) {
		if( t.equals(subTopic[i]) ) {
			// toggle the LED when we GET a new message
			toggleLED();
			memcpy( subStorage[i], (void*)payload, length );
			*subUpdate[i] = true;
			return;
		}
	}
}

// subscription topics
String commsTopicSystemCommand() {
	return( 
		preface + topicSep + 
			settingsFunction 
	);

}
String commsTopicLight(byte seepleNumber, byte archNumber) {
	return( 
		preface + topicSep + 
			lightFunction + topicSep + 
				String(seepleNumber,10) + topicSep + 
					String(archNumber,10)
	);
}
String commsTopicDistance(byte seepleNumber, byte archNumber) {
	return( 
		preface + topicSep + 
			motionFunction + topicSep + 
				String(seepleNumber,10) + topicSep + 
					String(archNumber,10)
	);

}
String commsTopicFreq(byte seepleNumber, byte archNumber) {
	return( 
		preface + topicSep + 
			freqFunction + topicSep + 
				String(seepleNumber,10) + topicSep + 
					String(archNumber,10)
	);

}

// subscribe to a topic
void commsSubscribe(String topic, void * storage, boolean * updateFlag) {
	// check to see if we've execeed the buffer size
	if( nTopics >= maxTopics ) {
		Serial << F("Increase Nyctinasty_Comms.cpp maxTopics!!!  Halting.") << endl;
		while(1) yield();
	}
	// queue subscriptions.  actual subscriptions happen at every (re)connect to the broker.
	subTopic[nTopics] = topic;
	subStorage[nTopics] = storage;
	subUpdate[nTopics] = updateFlag; 
	nTopics++;
}

// publish to a topic
boolean commsPublish(String topic, uint8_t * msg, unsigned int msgBytes) {
	// bit of an edge case, but let's check
	if( msgBytes >= MQTT_MAX_PACKET_SIZE ) {
		Serial << F("Increase PubSubClient.h MQTT_MAX_PACKET_SIZE!!!  Halting.") << endl;
		while(1) yield();
	}
	// bail out if we're not connected
	if( !commsConnected() ) return( false );
	// toggle the LED when we send a new message
	toggleLED();
	// ship it
	return mqtt.publish(topic.c_str(), msg, msgBytes);
}

// connect to the WiFi
void connectWiFi(String ssid, String passwd, unsigned long interval) {
	static Metro connectInterval(interval);
	if ( connectInterval.check() ) {

		Serial << F("Attempting WiFi connection to ") << ssid << endl;
		// Attempt to connect
		WiFi.begin(ssid.c_str(), passwd.c_str());

		connectInterval.reset();
	}
}

// connect to MQTT broker
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


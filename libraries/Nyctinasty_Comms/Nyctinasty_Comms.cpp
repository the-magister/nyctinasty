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
//void (* subCallback[maxTopics])();


void getIdEEPROM(Id id) {
	EEPROM.begin(512);
	EEPROM.get(0, id);
	EEPROM.commit();
	EEPROM.end();
}
void putIdEEPROM(Id id) {
	EEPROM.begin(512);
	EEPROM.get(0, id);
	EEPROM.end();
}


/*
Project:	nyc
	Role: 		Coordinator, Motion, Light, Sound, Fx, UI
		Sepal:		0-2
			Arch:		0-2
*/

// ID and topics
const String project = "nyc";
const String sep = "/";
const String oneWild = "+";
const String roles[] = {"Motion","Coordinator","Light","Sound","Fx","UI"};

String commsIdSepalArchMotion(byte sepalNumber, byte archNumber) {
	return( 
		project + sep + 
			roles[0] + sep + 
				String(sepalNumber,10) + sep + 
					String(archNumber,10)
	);

}
String commsIdSepalCoordinator(byte sepalNumber) {
	return(
		project + sep + 
			roles[1] + sep + 
				String(sepalNumber,10)  
	);
}
String commsIdSepalArchLight(byte sepalNumber, byte archNumber) {
	return( 
		project + sep + 
			roles[2] + sep + 
				String(sepalNumber,10) + sep + 
					String(archNumber,10)
	);
}

// subscription topics
const String messages[] = {"Settings","Light","Dist","Freq"};

String commsTopicSystemCommand() {
	return( 
		project + sep + 
			messages[0]
	);

}
String commsTopicLight(byte sepalNumber, byte archNumber) {
	return( 
		project + sep + 
			messages[1] + sep + 
				(sepalNumber>=N_SEPALS ? oneWild : String(sepalNumber,10)) + sep + 
					(archNumber>=N_ARCHES ? oneWild : String(archNumber,10))
	);
}
String commsTopicDistance(byte sepalNumber, byte archNumber) {
	return( 
		project + sep + 
			messages[2] + sep + 
				(sepalNumber>=N_SEPALS ? oneWild : String(sepalNumber,10)) + sep + 
					(archNumber>=N_ARCHES ? oneWild : String(archNumber,10))
	);

}
String commsTopicFreq(byte sepalNumber, byte archNumber) {
	return( 
		project + sep + 
			messages[3] + sep + 
				(sepalNumber>=N_SEPALS ? oneWild : String(sepalNumber,10)) + sep + 
					(archNumber>=N_ARCHES ? oneWild : String(archNumber,10))
	);

}

void commsBegin(String id, byte ledPin) {
	Serial << "commsBegin with id: " << id << endl;
	
	if( WiFi.status()==WL_CONNECTED ) WiFi.disconnect();
	
// variation in WiFi library calls btw ESP8266 and ESP32
#ifdef ARDUINO_ARCH_ESP32
	// station mode
	WiFi.mode(WIFI_MODE_STA);
#endif 
#ifdef ARDUINO_ARCH_ESP8266
	// don't allow the WiFi module to sleep.	
	WiFi.setSleepMode(WIFI_NONE_SLEEP);
	// station mode
	WiFi.mode(WIFI_STA);
#endif 

	myID = id;
	mqtt.setClient(espClient);
	mqtt.setCallback(commsCallback);

	myLED = ledPin;
	pinMode(myLED, OUTPUT);

}

void toggleLED() {
	// toggle the LED when we process a message
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
			// copy memory
			memcpy( subStorage[i], (void*)payload, length );
			*subUpdate[i] = true;
			// run the user-defined callback
//			subCallback[i]();
			return;
		}
	}
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
/*
void commsSubscribe(String topic, void * storage, void (*callBackFunction)()) {
	// check to see if we've execeed the buffer size
	if( nTopics >= maxTopics ) {
		Serial << F("Increase Nyctinasty_Comms.cpp maxTopics!!!  Halting.") << endl;
		while(1) yield();
	}
	// queue subscriptions.  actual subscriptions happen at every (re)connect to the broker.
	subTopic[nTopics] = topic;
	subStorage[nTopics] = storage;
	subCallback[nTopics] = callBackFunction; 
	nTopics++;
}
*/

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

		Serial << F("WiFi status=") << WiFi.status();
		Serial << F("\tAttempting WiFi connection to ") << ssid << F(" password ") << passwd << endl;
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
				Serial << F("Subscribing: ") << subTopic[i];
				mqtt.loop(); // in case messages are in.
				boolean ret = mqtt.subscribe(subTopic[i].c_str());
				Serial << F(" res=") << ret << endl;
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


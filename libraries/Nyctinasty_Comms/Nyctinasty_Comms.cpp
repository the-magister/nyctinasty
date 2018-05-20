#include "Nyctinasty_Comms.h"

// mqtt callback is external to class
void MQTTCallback(char* topic, byte* payload, unsigned int length);

#define BUILTIN_LED_ON_STATE LOW
void setOnLED();
void setOffLED();
void toggleLED();

// helpful decodes for the humans.  must be unique.
const String NyctRoleString[] = {
	"Coordinator",
	"Arch-0",
	"Arch-1",
	"Arch-2",
	"Sound",
	"Cannon"
};

// public methods

void NyctComms::begin(NyctRole role) {
	Serial << F("Startup. commsBegin starts.") << endl;

	// prep the LED
	pinMode(BUILTIN_LED, OUTPUT);
	digitalWrite(BUILTIN_LED, !BUILTIN_LED_ON_STATE);

	// figure out who I am.
	getsetEEPROM(role);
	myName = "nyc-" + NyctRoleString[this->role];
	Serial << F("Who's your daddy? ") << myName << endl;
	
	// comms setup
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

	mqtt.setClient(wifi);
	mqtt.setCallback(MQTTCallback);

	// enable OTA push programming, too.
	ArduinoOTA.setHostname(myName.c_str());
	ArduinoOTA.onStart([]() {
		Serial.println("Start");
	});
	ArduinoOTA.onEnd([]() {
		Serial.println("\nEnd");
	});
	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
		toggleLED();
	});
	ArduinoOTA.onError([](ota_error_t error) {
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
		else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
		else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
		else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
		else if (error == OTA_END_ERROR) Serial.println("End Failed");
	});
	
	Serial << F("Startup. commsBegin ends.") << endl;
}
// subscriptions. cover the messages in Nyctinasty_Messages.h
const String settingsString = "nyc/Setting";
const String distanceString = "nyc/Distance";
const String frequencyString = "nyc/Frequency";
const String simonString = "nyc/Fx/Simon";
const String sep = "/";
void NyctComms::subscribe(SystemCommand *storage, boolean *trueWithUpdate) {
	subscribe(
		settingsString, 
		(void *)storage, trueWithUpdate, 1
	);
}
void NyctComms::subscribe(SimonSystemState *storage, boolean *trueWithUpdate) {
	subscribe(
		simonString, 
		(void *)storage, trueWithUpdate, 0
	);
}
void NyctComms::subscribe(SepalArchDistance *storage, boolean *trueWithUpdate, uint8_t archNumber) {
	String a = String( archNumber, 10 );
	subscribe(
		distanceString + sep + a, 
		(void *)storage, trueWithUpdate, 0
	);
}
void NyctComms::subscribe(SepalArchFrequency *storage, boolean *trueWithUpdate, uint8_t archNumber) {
	String a = String( archNumber, 10 );
	subscribe(
		frequencyString + sep + a, 
		(void *)storage, trueWithUpdate, 0
	);
}

// publications.  cover the messages in Nyctinasty_Messages.h
boolean NyctComms::publish(SystemCommand *storage) {
	return publish(
		settingsString, 
		(uint8_t *)storage, (unsigned int)sizeof(SystemCommand)
	);
}
boolean NyctComms::publish(SimonSystemState *storage) {
	return publish(
		simonString, 
		(uint8_t *)storage, (unsigned int)sizeof(SimonSystemState)
	);
}
boolean NyctComms::publish(SepalArchDistance *storage, uint8_t archNumber) {
	String a = String( archNumber, 10 );
	return publish(
		distanceString + sep + a, 
		(uint8_t *)storage, (unsigned int)sizeof(SepalArchDistance)
	);
}
boolean NyctComms::publish(SepalArchFrequency *storage, uint8_t archNumber) {
	String a = String( archNumber, 10 );
	return publish(
		frequencyString + sep + a, 
		(uint8_t *)storage, (unsigned int)sizeof(SepalArchFrequency)
	);
}

// call this very frequently
void NyctComms::update() {
	if (WiFi.status() != WL_CONNECTED) {
		connectWiFi();
	} else if (!mqtt.connected()) {
		connectServices();
	} else {
		mqtt.loop(); // look for a message
		ArduinoOTA.handle(); // see if we need to reprogram
	}
}
// check connection
boolean NyctComms::isConnected() {
	return (mqtt.connected()==true) && (WiFi.status()==WL_CONNECTED);
}

// useful functions.
void NyctComms::reboot() {
	setOnLED();

	Serial << endl << F("*** REBOOTING ***") << endl;
	delay(100);
	ESP.restart();
}
void NyctComms::reprogram(String binaryName) {
	setOnLED();
	
	String updateURL = "http://192.168.4.1:80/images/" + binaryName;
	Serial << endl << F("*** RELOADING ***") << endl;
	Serial << F("Getting payload from: ") << updateURL << endl;

	delay(random(10, 500)); // so we all don't hit the server at the same time.

	// enable OTA pull programming.  do not reboot after programming.
	ESPhttpUpdate.rebootOnUpdate(false);

	t_httpUpdate_return ret = ESPhttpUpdate.update(updateURL.c_str());

	switch (ret) {
		case HTTP_UPDATE_FAILED:
			Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
			break;

		case HTTP_UPDATE_NO_UPDATES:
			Serial.println("HTTP_UPDATE_NO_UPDATES");
			// now reboot
			reboot();
			break;

		case HTTP_UPDATE_OK:
			Serial.println("HTTP_UPDATE_OK");
			// now reboot
			reboot();
			break;
	}
}
NyctRole NyctComms::getRole() { return(this->role); }

// private methods

typedef struct {
	NyctRole role; 
} EEPROMstruct;

void NyctComms::getsetEEPROM(NyctRole role) {
	EEPROMstruct save;
	EEPROM.begin(512);
	
	if( role != N_ROLES ) {
		// set
		save.role = role;
		
		EEPROM.put(0, save);
		EEPROM.commit();
	} else {
		// get
		EEPROM.get(0, save);
		
		if( save.role >= N_ROLES ) {
			Serial << F("*** No role information in EEPROM. HALTING. ***") << endl;
			while(1) delay(5);
		}
	}

	EEPROM.end();
	
	this->role = save.role;
	
	Serial << F("Role information in EEPROM:");
	Serial << F(" role=") << this->role;
	Serial << F(" (") << NyctRoleString[this->role] << F(")");
	Serial << endl;

}


// memory maintained outside of the class.  :(

uint8_t MQTTnTopics = 0;
String MQTTsubTopic[MQTT_MAX_SUBSCRIPTIONS];
void * MQTTsubStorage[MQTT_MAX_SUBSCRIPTIONS];
boolean * MQTTsubUpdate[MQTT_MAX_SUBSCRIPTIONS];
uint8_t MQTTsubQoS[MQTT_MAX_SUBSCRIPTIONS];

// the real meat of the work is done here, where we process messages.
void MQTTCallback(char* topic, byte* payload, unsigned int length) {

	// String class is much easier to work with
	String t = topic;
	
	// run through topics we're subscribed to
	for( uint8_t i=0;i < MQTTnTopics;i++ ) {
		if( t.equals(MQTTsubTopic[i]) ) {
			// copy memory
			memcpy( MQTTsubStorage[i], (void*)payload, length );
			*MQTTsubUpdate[i] = true;
			
			// toggle the LED when we GET a new message
			toggleLED();
			return;
		}
	}
}

// subscribe to a topic
void NyctComms::subscribe(String topic, void * storage, boolean * updateFlag, uint8_t QoS) {
	// check to see if we've execeed the buffer size
	if( MQTTnTopics >= MQTT_MAX_SUBSCRIPTIONS ) {
		Serial << F("Increase Nyctinasty_Comms.h MQTT_MAX_SUBSCRIPTIONS!!!  Halting.") << endl;
		while(1) yield();
	}
	// queue subscriptions.  actual subscriptions happen at every (re)connect to the broker.
	MQTTsubTopic[MQTTnTopics] = topic;
	MQTTsubStorage[MQTTnTopics] = storage;
	MQTTsubUpdate[MQTTnTopics] = updateFlag; 
	MQTTsubQoS[MQTTnTopics] = QoS;
	MQTTnTopics++;
}

// publish to a topic
boolean NyctComms::publish(String topic, uint8_t * msg, unsigned int msgBytes) {
	// bit of an edge case, but let's check
	if( msgBytes >= MQTT_MAX_PACKET_SIZE ) {
		Serial << F("Increase PubSubClient.h MQTT_MAX_PACKET_SIZE >") << msgBytes << endl;;
		Serial << F("!!!  Halting.") << endl;
		while(1) yield();
	}
	// bail out if we're not connected
	if( !this->isConnected() ) return( false );
	// toggle the LED when we send a new message
	toggleLED();
	// ship it
	return mqtt.publish(topic.c_str(), msg, msgBytes);
}

// connect to the WiFi
void NyctComms::connectWiFi(String ssid, String pwd, uint32_t interval) {
	static Metro connectInterval(interval);
	static uint32_t retryCount = 0;
	
	if (WiFi.status()==WL_CONNECTED) retryCount = 0;
	
	if ( connectInterval.check() ) {
		retryCount++;
		toggleLED();

/*		
		Serial << F("WiFi status=") << WiFi.status();
		Serial << F(" Retry #") << retryCount;
		Serial << F(" Attempting WiFi connection to ") << ssid << F(" password ") << pwd<< endl;
*/
		// Attempt to connect
		WiFi.begin(ssid.c_str(), pwd.c_str());
		
		connectInterval.reset();
	}
	
//	if( retryCount >= 10 ) reboot();
}

// connect to MQTT broker and other services
void NyctComms::connectServices(String broker, word port, uint32_t interval) {
	static Metro connectInterval(interval);
	static uint32_t retryCount = 0;

	if ( connectInterval.check() ) {

		retryCount++;
		toggleLED();
		
		if ( MDNS.begin ( myName.c_str() ) ) {
			Serial << F("mDNS responder started: ") << myName << endl;
		} else {
			Serial << F("Could NOT start mDNS!") << endl;
		}
		
		Serial << F("OTA started: ") << myName << endl;
		ArduinoOTA.begin();
		
		Serial << F("Attempting MQTT connection as ") << myName << " to " << broker << ":" << port << endl;
		mqtt.setServer(broker.c_str(), port);

		// Attempt to connect
		if (mqtt.connect(myName.c_str())) {
			Serial << F("Connected.") << endl;

			// (re)subscribe
			for(byte i=0; i<MQTTnTopics; i++) {
				Serial << F("Subscribing: ") << MQTTsubTopic[i];
				Serial << F(" QoS=") << MQTTsubQoS[i];
				mqtt.loop(); // in case messages are in.
				boolean ret = mqtt.subscribe(MQTTsubTopic[i].c_str(), MQTTsubQoS[i]);
				Serial << F(". OK? ") << ret << endl;
			}
			
			retryCount = 0;

		} else {
			Serial << F("Failed. state=") << mqtt.state() << endl;
			Serial << F("WiFi status connected? ") << (WiFi.status()==WL_CONNECTED) << endl;
		}

		connectInterval.reset();
	}

//	if( retryCount >= 10 ) reboot();
}


boolean ledState = !BUILTIN_LED_ON_STATE;
void setOnLED() {
	if( ledState!=BUILTIN_LED_ON_STATE ) toggleLED();
}
void setOffLED() {
	if( ledState==BUILTIN_LED_ON_STATE ) toggleLED();
}
void toggleLED() {
	// toggle the LED when we process a message
	ledState = !ledState;
	digitalWrite(BUILTIN_LED, ledState);
}

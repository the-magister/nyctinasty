#include "Nyctinasty_Comms.h"

WiFiClient espClient;
PubSubClient mqtt;

// save
String myID, otaID;
byte myLED;
boolean ledState = false;
boolean ledOnState = false;

const byte maxTopics = 32;
byte nTopics = 0;
String subTopic[maxTopics];
void * subStorage[maxTopics];
boolean * subUpdate[maxTopics];
byte subQoS[maxTopics];

Id getIdEEPROM(boolean resetRole) {
	Id id;
	EEPROM.begin(512);
	EEPROM.get(0, id);
	
	if( resetRole || id.checksum != 8675309) {
		// Yikes.  Fresh uC with no information on its role in life.  
		setOnLED();

		// bootstrap
		id.checksum = 8675309;
		Serial << F("*** No role information in EEPROM ***") << endl;
		
		Serial.setTimeout(10);
		
		Serial << F("Enter Role. ") << endl;
		while(! Serial.available()) yield();
		String m = Serial.readString();
		m.toCharArray(id.role, sizeof(id.role));
		
		Serial << F("Enter Sepal. ") << N_SEPALS << F(" for N/A.") << endl;
		while(! Serial.available()) yield();
		m = Serial.readString();
		id.sepal = (byte)m.toInt();
		
		Serial << F("Enter Arch. ") << N_ARCHES << F(" for N/A.") << endl;
		while(! Serial.available()) yield();
		m = Serial.readString();
		id.arch = (byte)m.toInt();
		
		putIdEEPROM(id);
	} else {
		Serial << F("from EEPROM.");
		Serial << F(" role=") << id.role;
		Serial << F(" sepal=") << id.sepal;
		Serial << F(" arch=") << id.arch;
		Serial << F(" checksum=") << id.checksum;
		Serial << endl;
	}
	
	EEPROM.end();

	return( id );
}
void putIdEEPROM(Id id) {
	Serial << F("to EEPROM.");
	Serial << F(" role=") << id.role;
	Serial << F(" sepal=") << id.sepal;
	Serial << F(" arch=") << id.arch;
	Serial << F(" checksum=") << id.checksum;
	Serial << endl;

	EEPROM.put(0, id);
}


/*
Project:	nyc
	Role: 		Coordinator, Frequency, Light, Sound, Fx, UI
		Sepal:		0-2
			Arch:		0-2
*/

// ID and topics
String commsIdSepalArchFrequency(byte s, byte a) { return commsStringConstructor("Frequency", s, a); }
String commsIdSepalCoordinator(byte s) { return commsStringConstructor("Coordinator", s, N_ARCHES); }
String commsIdSepalArchLight(byte s, byte a) { return commsStringConstructor("FxSimon", s, a); }
String commsIdFlowerSimonBridge() { return commsStringConstructor("Fx-Simon"); }

// subscription topics
// note that wildcards are not supported, as we use string matching to determine incoming messages
String commsTopicSystemCommand() { return commsStringConstructor("Settings"); }
String commsTopicLight(byte s, byte a) { return commsStringConstructor("Light", s, a); }
String commsTopicDistance(byte s, byte a) { return commsStringConstructor("Dist", s, a); }
String commsTopicFrequency(byte s, byte a) { return commsStringConstructor("Freq", s, a); }
String commsTopicFxSimon() { return commsStringConstructor("Fx/Simon"); }

String commsStringConstructor(String topic, byte sepalNumber, byte archNumber) {
	const String project = "nyc";
	const String sep = "/";
	
	String sepal = (sepalNumber < N_SEPALS) ? ( sep + String(sepalNumber,10) ) : "";
	String arch = (archNumber < N_ARCHES && sepalNumber < N_SEPALS) ? ( sep + String(archNumber,10) ) : "";

	String sub = project + sep + topic + sepal + arch;
//	Serial << sub << endl;
	return sub;
}

Id commsBegin(boolean resetRole, byte ledPin) {
	Serial << "Startup. commsBegin starts." << endl;

	// prep the LED
	myLED = ledPin;
	pinMode(myLED, OUTPUT);
	toggleLED();
	setOffLED();

	// figure out who I am.
	Id id = getIdEEPROM(resetRole);
	
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

	myID = commsStringConstructor(id.role, id.sepal, id.arch);
	otaID = id.role;
	otaID.replace("/","-");
	Serial << F("Startup. MQTT id=") << otaID << endl;
	
	mqtt.setClient(espClient);
	mqtt.setCallback(commsCallback);

	// enable OTA pull programming.  reboots after a new payload.
	ESPhttpUpdate.rebootOnUpdate(true);

	// enable OTA push programming, too.
	ArduinoOTA.setHostname(otaID.c_str());
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
	
	
	Serial << "Startup. commsBegin ends." << endl;
	
	return( id );
}

void setOnLED() {
	if( ledState!=ledOnState ) toggleLED();
}
void setOffLED() {
	if( ledState==ledOnState ) toggleLED();
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
		ArduinoOTA.handle(); // see if we need to reprogram
	}
}

// the real meat of the work is done here, where we process messages.
void commsCallback(char* topic, byte* payload, unsigned int length) {

	// String class is much easier to work with
	String t = topic;
	
	// run through topics we're subscribed to
	for( byte i=0;i < nTopics;i++ ) {
		if( t.equals(subTopic[i]) ) {
			// copy memory
			memcpy( subStorage[i], (void*)payload, length );
			*subUpdate[i] = true;
			
			// toggle the LED when we GET a new message
			toggleLED();
			return;
		}
	}
}

// subscribe to a topic
void commsSubscribe(String topic, void * storage, boolean * updateFlag, uint8_t QoS) {
	// check to see if we've execeed the buffer size
	if( nTopics >= maxTopics ) {
		Serial << F("Increase Nyctinasty_Comms.cpp maxTopics!!!  Halting.") << endl;
		while(1) yield();
	}
	// queue subscriptions.  actual subscriptions happen at every (re)connect to the broker.
	subTopic[nTopics] = topic;
	subStorage[nTopics] = storage;
	subUpdate[nTopics] = updateFlag; 
	subQoS[nTopics] = QoS;
	nTopics++;
}

// publish to a topic
boolean commsPublish(String topic, uint8_t * msg, unsigned int msgBytes) {
	// bit of an edge case, but let's check
	if( msgBytes >= MQTT_MAX_PACKET_SIZE ) {
		Serial << F("Increase PubSubClient.h MQTT_MAX_PACKET_SIZE >") << msgBytes << endl;;
		Serial << F("!!!  Halting.") << endl;
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
	static byte retryCount = 0;
	
	if (WiFi.status()==WL_CONNECTED) retryCount = 0;
	
	if ( connectInterval.check() ) {
		retryCount++;
		
		Serial << F("WiFi status=") << WiFi.status();
		Serial << F("\tAttempting WiFi connection to ") << ssid << F(" password ") << passwd << endl;
		// Attempt to connect
		WiFi.begin(ssid.c_str(), passwd.c_str());
		
		connectInterval.reset();
	}
	
	if( retryCount >= 10 ) reboot();
}

// connect to MQTT broker
void connectMQTT(String broker, word port, unsigned long interval) {
	static Metro connectInterval(interval);
	static byte retryCount = 0;

	if ( connectInterval.check() ) {

		retryCount++;

		if ( MDNS.begin ( otaID.c_str() ) ) {
			Serial << F("mDNS responder started: ") << otaID << endl;
		} else {
			Serial << F("Could NOT start MDNS!") << endl;
		}
		
		Serial << F("OTA started: ") << otaID << endl;
		ArduinoOTA.begin();
		
		Serial << F("Attempting MQTT connection as ") << otaID << " to " << broker << ":" << port << endl;
		mqtt.setServer(broker.c_str(), port);

		// Attempt to connect
		if (mqtt.connect(otaID.c_str())) {
			Serial << F("Connected.") << endl;

			// (re)subscribe
			for(byte i=0; i<nTopics; i++) {
				Serial << F("Subscribing: ") << subTopic[i];
				Serial << F(" QoS=") << subQoS[i];
				mqtt.loop(); // in case messages are in.
				boolean ret = mqtt.subscribe(subTopic[i].c_str(), subQoS[i]);
				Serial << F(". OK? ") << ret << endl;
			}
			
			retryCount = 0;

		} else {
			Serial << F("Failed. state=") << mqtt.state() << endl;
			Serial << F("WiFi status connected? ") << (WiFi.status()==WL_CONNECTED) << endl;
		}

		connectInterval.reset();
	}

	if( retryCount >= 10 ) reboot();
}

// reboot
void reboot() {
	setOnLED();

	Serial << endl << F("*** REBOOTING ***") << endl;
	delay(100);
	ESP.restart();
}

// reprogram
void reprogram(String binaryName) {
	setOnLED();
	
	String updateURL = "http://192.168.4.1:80/images/" + binaryName;
	Serial << endl << F("*** RELOADING ***") << endl;
	Serial << F("Getting payload from: ") << updateURL << endl;

	delay(random(100, 1000)); // so we all don't hit the server at the same time.

	t_httpUpdate_return ret = ESPhttpUpdate.update(updateURL.c_str());

	switch (ret) {
		case HTTP_UPDATE_FAILED:
		Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
		break;

		case HTTP_UPDATE_NO_UPDATES:
		Serial.println("HTTP_UPDATE_NO_UPDATES");
		break;

		case HTTP_UPDATE_OK:
		Serial.println("HTTP_UPDATE_OK");
		break;
	}
}

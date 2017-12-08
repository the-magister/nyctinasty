#ifndef Nyctinasty_Comms_h
#define Nyctinasty_Comms_h

#include <Arduino.h>

#include <Streaming.h>
#include <Metro.h>

#include "Nyctinasty_Messages.h"

// variation in WiFi library names btw ESP8266 and ESP32
#ifdef ARDUINO_ARCH_ESP32
#include <WiFi.h>
#endif 
#ifdef ARDUINO_ARCH_ESP8266
#include <ESP8266WiFi.h>
#endif 

#include <PubSubClient.h>

// call this very frequently
void commsUpdate();

// check connection
boolean commsConnected(); 

// don't need to mess with these
void connectWiFi(String ssid="GamesWithFire", String passwd="safetythird", unsigned long interval=5000UL);
void connectMQTT(String broker="broker", word port=1883, unsigned long interval=500UL);
void commsCallback(char* topic, byte* payload, unsigned int length);
void commsSubscribe(String topic, void * msg, boolean * updateFlag);
boolean commsPublish(String topic, uint8_t * msg, unsigned int msgBytes);
void toggleLED();

// build an MQTT id, which must be unique.
String commsIdSeepleCoordinator(byte seepleNumber);
String commsIdSeepleArchLight(byte seepleNumber, byte archNumber);
String commsIdSeepleArchMotion(byte seepleNumber, byte archNumber);

// startup.  use unique id that's built by the commsId* helpers. 
void commsBegin(String id, byte ledPin=BUILTIN_LED);

// build a MQTT topic, for use with subscribe and publish routines.
String commsTopicSystemCommand();
String commsTopicLight(byte seepleNumber, byte archNumber);
String commsTopicDistance(byte seepleNumber, byte archNumber);
String commsTopicFreq(byte seepleNumber, byte archNumber);

// subscribe to a topic, provide storage for the payload, provide a flag for update indicator
template <class T>
void commsSubscribe(String topic, T * msg, boolean * updateFlag) {
	commsSubscribe(topic, (void *)msg, updateFlag);
}

// publish to a topic
template <class T>
boolean commsPublish(String topic, T * msg) {
	return commsPublish(topic, (uint8_t *)msg, (unsigned int)sizeof(T));
}

#endif


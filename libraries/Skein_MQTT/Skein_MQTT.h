#ifndef Skein_MQTT_h
#define Skein_MQTT_h

#include <Arduino.h>

#include <Streaming.h>
#include <Metro.h>

#include "Skein_Messages.h"

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// startup.  use unique id!  will toggle pin as Rx/Tx occurs
void commsBegin(String id, byte ledPin);

// subscribe to topic, provide storage for the payload, provide a flag for update indicator
void commsSubscribe(String readingTopic, SensorReading * reading, boolean * updateFlag);
void commsSubscribe(String commandTopic, Command * command, boolean * updateFlag);

// call this very frequently
void commsUpdate();

// check connection
boolean commsConnected(); 

// publish to a topic
boolean commsPublish(String readingTopic, SensorReading * reading);
boolean commsPublish(String commandTopic, Command * command);

// don't need to mess with these
void connectWiFi(String ssid="GamesWithFire", String passwd="safetythird", unsigned long interval=5000UL);
void connectMQTT(String broker="broker", word port=1883, unsigned long interval=500UL);
void commsCallback(char* topic, byte* payload, unsigned int length);
void commsSubscribe(String topic, void * storage, boolean * update);

#endif


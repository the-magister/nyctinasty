#ifndef Skein_Comms_h
#define Skein_Comms_h

#include <Arduino.h>

#include <Streaming.h>
#include <Metro.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

void commsBegin(String id);
void commsUpdate();

void commsSubscribe(String topic);
extern void commsProcess(String topic, String message);

void commsPublish(String topic, String message, boolean showPub=false);

// don't need to mess with these
void connectWiFi(String ssid="GamesWithFire", String passwd="safetythird", unsigned long interval=5000UL);
void connectMQTT(String broker="broker", word port=1883, unsigned long interval=500UL);
void commsCallback(char* topic, byte* payload, unsigned int length);

#endif


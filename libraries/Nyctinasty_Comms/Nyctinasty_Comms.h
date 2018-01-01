#ifndef Nyctinasty_Comms_h
#define Nyctinasty_Comms_h

#include <Arduino.h>

#include <Streaming.h>
#include <Metro.h>
#include <EEPROM.h>

// variation in WiFi library names btw ESP8266 and ESP32
#ifdef ARDUINO_ARCH_ESP32
#include <WiFi.h>
#endif 
#ifdef ARDUINO_ARCH_ESP8266
#include <ESP8266WiFi.h>
#endif 

#include <ESP8266httpUpdate.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <PubSubClient.h>

#include "Nyctinasty_Messages.h"
#include "Simon_Common.h"

// roles for each microcontroller
enum NyctRole {
  Distance=0, 
  Frequency=1,
  Light=2,
  Sound=3,
  FxSimon=4,
  Coordinator=5,
  
  N_ROLES=6
};

// how many subscription topics should we malloc() for?
#define MQTT_MAX_SUBSCRIPTIONS 32

// during publishing and subscribing, we can use a short-hand of "my sepal and arch".
#define MY_INDEX 99

// prosecute communications
class NyctComms {
public:

	// startup. 
	void begin(NyctRole role, boolean resetRole=false);

	// subscriptions. cover the messages in Nyctinasty_Messages.h
	void subscribe(SystemCommand *storage, boolean *trueWithUpdate);
	void subscribe(SimonSystemState *storage, boolean *trueWithUpdate);
	void subscribe(SepalArchDistance *storage, boolean *trueWithUpdate, uint8_t sepalNumber=MY_INDEX, uint8_t archNumber=MY_INDEX);
	void subscribe(SepalArchFrequency *storage, boolean *trueWithUpdate, uint8_t sepalNumber=MY_INDEX, uint8_t archNumber=MY_INDEX);
	
	// publications.  cover the messages in Nyctinasty_Messages.h
	boolean publish(SystemCommand *message);
	boolean publish(SimonSystemState *message);
	boolean publish(SepalArchDistance *message, uint8_t sepalNumber=MY_INDEX, uint8_t archNumber=MY_INDEX);
	boolean publish(SepalArchFrequency *message, uint8_t sepalNumber=MY_INDEX, uint8_t archNumber=MY_INDEX);
	
	// call this very frequently
	void update();
	// check connection
	boolean isConnected(); 
	
	// useful functions.
	void reboot();
	void reprogram(String binaryName);
	byte getSepal();
	byte getArch();
	
private:
	// role and indexing
	NyctRole role;
	uint8_t sepal, arch;
	void getsetEEPROM(NyctRole role, boolean resetRole);
	
	// WiFi
	WiFiClient wifi;
	String wifiPassword; // get it from EEPROM
	void connectWiFi(String ssid="GamesWithFire", unsigned long interval=5000UL);
	
	// MQTT
	PubSubClient mqtt;
	String myName;
	void connectServices(String broker="192.168.4.1", word port=1883, unsigned long interval=500UL);
	void subscribe(String topic, void * storage, boolean * updateFlag, uint8_t QoS);
	boolean publish(String topic, uint8_t * msg, unsigned int msgBytes);
	
};

#endif


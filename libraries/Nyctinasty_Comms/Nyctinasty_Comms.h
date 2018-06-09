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
	Coordinator=0,
	Arch0,
	Arch1,
	Arch2,
	Sound,
	WaterPumps,
	WaterRoute,
  
	N_ROLES
};
// helpful decodes for the humans.  must be unique.
const String NyctRoleString[] = {
	"Coordinator",
	"Arch-0",
	"Arch-1",
	"Arch-2",
	"Sound",
	"WaterPumps",
	"WaterRoute"
};

// how many subscription topics should we malloc() for?
#define MQTT_MAX_SUBSCRIPTIONS 32

// prosecute communications
class NyctComms {
public:

	// startup. 
	void begin(NyctRole role=N_ROLES);

	// subscriptions. cover the messages in Nyctinasty_Messages.h
	void subscribe(SystemCommand *storage, boolean *trueWithUpdate);
	void subscribe(SimonSystemState *storage, boolean *trueWithUpdate);
	void subscribe(SepalArchDistance *storage, boolean *trueWithUpdate, uint8_t archNumber);
	void subscribe(SepalArchFrequency *storage, boolean *trueWithUpdate, uint8_t archNumber);
	void subscribe(WaterWorks *storage, boolean *trueWithUpdate);
	
	// publications.  cover the messages in Nyctinasty_Messages.h
	boolean publish(SystemCommand *message);
	boolean publish(SimonSystemState *message);
	boolean publish(SepalArchDistance *message, uint8_t archNumber);
	boolean publish(SepalArchFrequency *message, uint8_t archNumber);
	boolean publish(WaterWorks *message);
	
	// call this very frequently
	void update();
	// check connection
	boolean isConnected(); 
	
	// useful functions.
	void reboot();
	void reprogram(String binaryName);
	NyctRole getRole();
	
private:
	// role and indexing
	NyctRole role;
	void getsetEEPROM(NyctRole role);
	
	// WiFi
	WiFiClient wifi;
	void connectWiFi(String ssid="GamesWithFire", String pwd="safetythird", uint32_t interval=5000UL);
	
	// MQTT
	PubSubClient mqtt;
	String myName;
	void connectServices(String broker="192.168.4.1", word port=1883, uint32_t interval=500UL);	
//	void connectServices(String broker="broker", word port=1883, uint32_t interval=500UL);
	void subscribe(String topic, void * storage, boolean * updateFlag, uint8_t QoS);
	boolean publish(String topic, uint8_t * msg, unsigned int msgBytes);
	
};

#endif


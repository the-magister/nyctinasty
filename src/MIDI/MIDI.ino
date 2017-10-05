/*
 * This module is responsible for running MIDI based on the Lidar sensor data.
 * Prototyping a few different ways to communicate MIDI.  We have a FeatherWing that has a local midi synth.  Or
 * we might use Apple's TCP Midi to use external midi devices.  The FeatherWing seems to only work on Serial instead of Serial1
 * 
 * Subscribes: skein/range/#
 */
// You'll need to add http://arduino.esp8266.com/stable/package_esp8266com_index.json to the Additional Board Managers URL entry in Preferences.
// Compile for NodeMCU 1.0 (ESP-12E Module), 80 Mhz, 921600 Upload Speed, 4M (3M SPIFFS).
#include <Streaming.h>
#include <Metro.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "Skein_Comms.h"
#include "SoftwareSerial.h"

#define BLUE_LED 2 
#define RED_LED 16 // labeled "D0" on NodeMCU boards; HIGH=off

// really need to save this to EEPROM
// should we ever need to extend this to more uCs, this will generate an offset.
const byte subsetIndex = 1;

const byte Nsensor = 8;
word range[Nsensor];
word outOfRange = (1 << 11) - 1; // coresponds to a reading that's out-of-range

byte value[Nsensor];
unsigned long avgValue[Nsensor];
// linearize perception to value
float R = (float)(outOfRange) / log2(127.0 + 1.0);

// connect to the MQTT network with this id
String id = "skeinMIDI" + subsetIndex;

// subscribe and process these topics
//String ranges = "skein/range/" + String(subsetIndex, 10) + "/#";
String ranges = "skein/range/#";
String oor = "skein/range/oor";

// Midi

// See http://www.vlsi.fi/fileadmin/datasheets/vs1053.pdf Pg 31
#define VS1053_BANK_DEFAULT 0x00
#define VS1053_BANK_DRUMS1 0x78
#define VS1053_BANK_DRUMS2 0x7F
#define VS1053_BANK_MELODY 0x79

// See http://www.vlsi.fi/fileadmin/datasheets/vs1053.pdf Pg 32 for more!
#define VS1053_GM1_HARPSICORD 7
#define VS1053_GM1_VIOLIN 41
#define VS1053_GM1_ORCHESTIRAL_HARP 47
#define VS1053_GM1_PICCOLO 73
#define VS1053_GM1_OCARINA 80

#define MIDI_NOTE_ON  0x90
#define MIDI_NOTE_OFF 0x80
#define MIDI_CHAN_MSG 0xB0
#define MIDI_CHAN_BANK 0x00
#define MIDI_CHAN_VOLUME 0x07
#define MIDI_CHAN_PROGRAM 0xC0

#if defined(__AVR_ATmega32U4__) || defined(ARDUINO_SAMD_FEATHER_M0) || defined(TEENSYDUINO) || defined(ARDUINO_STM32_FEATHER)
  #define VS1053_MIDI Serial1
#elif defined(ESP32)
  HardwareSerial Serial1(2);
  #define VS1053_MIDI Serial1
#elif defined(ESP8266)
  SoftwareSerial swSer(14,12,false,256);
  #define VS1053_MIDI swSer
#endif

#define MIDI_ENABLED true
#define INST_MODE_DRUMS 0              // Drum machine
#define INST_MODE_STRINGS_OCTIVE 1     // Strings with different octive based on height
#define INST_MODE_STRINGS_ATTACK 2     // Strings with attack based on height
#define INST_MODE_DRUMS_ATTACK 3       // Drum machine with attacked based on height

static uint8_t OCTIVE_4_NOTES[8] = { 53, 55, 57, 59, 48, 50, 50, 52};  // 12 spacing
static uint8_t DRUMS[8] = { 44, 51, 50, 38, 55, 55, 56, 39}; 

/*
static uint8_t instMode[Nsensor] = {INST_MODE_STRINGS_OCTIVE,INST_MODE_STRINGS_OCTIVE,INST_MODE_STRINGS_ATTACK,INST_MODE_STRINGS_ATTACK,
                                    INST_MODE_DRUMS,INST_MODE_DRUMS,INST_MODE_DRUMS_ATTACK,INST_MODE_DRUMS_ATTACK};                                                                        

static uint8_t instMode[Nsensor] = {INST_MODE_STRINGS_OCTIVE,INST_MODE_STRINGS_OCTIVE,INST_MODE_STRINGS_OCTIVE,INST_MODE_STRINGS_OCTIVE,
                                    INST_MODE_STRINGS_OCTIVE,INST_MODE_STRINGS_OCTIVE,INST_MODE_STRINGS_OCTIVE,INST_MODE_STRINGS_OCTIVE};              
                                                          
static uint8_t instMode[Nsensor] = {INST_MODE_STRINGS_ATTACK,INST_MODE_STRINGS_ATTACK,INST_MODE_STRINGS_ATTACK,INST_MODE_STRINGS_ATTACK,
                                    INST_MODE_STRINGS_ATTACK,INST_MODE_STRINGS_ATTACK,INST_MODE_STRINGS_ATTACK,INST_MODE_STRINGS_ATTACK};
                                    /*
static uint8_t instMode[Nsensor] = {INST_MODE_STRINGS_OCTIVE,INST_MODE_STRINGS_OCTIVE,INST_MODE_STRINGS_OCTIVE,INST_MODE_STRINGS_OCTIVE,
                                    INST_MODE_DRUMS,INST_MODE_DRUMS,INST_MODE_DRUMS,INST_MODE_DRUMS};

                                   
static uint8_t instMode[Nsensor] = {INST_MODE_DRUMS,INST_MODE_DRUMS,INST_MODE_DRUMS,INST_MODE_DRUMS,
                                    INST_MODE_DRUMS,INST_MODE_DRUMS,INST_MODE_DRUMS,INST_MODE_DRUMS};
*/

                                    static uint8_t instMode[Nsensor] = {INST_MODE_DRUMS_ATTACK,INST_MODE_DRUMS_ATTACK,INST_MODE_DRUMS_ATTACK,INST_MODE_DRUMS_ATTACK,
                                    INST_MODE_DRUMS_ATTACK,INST_MODE_DRUMS_ATTACK,INST_MODE_DRUMS_ATTACK,INST_MODE_DRUMS_ATTACK};

unsigned long noteLength[Nsensor];
int octiveMin[Nsensor]; 
int octiveMax[Nsensor]; 
unsigned long lastStart[Nsensor];  // Last time we started this note

void setup(void)  {
  Serial.begin(115200);
  delay(20);
  Serial << endl << endl << "Startup." << endl;
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(BLUE_LED, false);

  commsBegin(id);
  commsSubscribe(ranges);
  commsSubscribe(oor);

#ifdef MIDI_ENABLED
    VS1053_MIDI.begin(31250); // MIDI uses a 'strange baud rate'
    for(byte i=0; i < Nsensor; i++) {
      switch(instMode[i]) {
        case INST_MODE_DRUMS:
          midiSetChannelBank(i, VS1053_BANK_DRUMS1);
          noteLength[i] = 300l;
          octiveMin[i] = -1;
          octiveMax[i] = 1;
          break;
        case INST_MODE_DRUMS_ATTACK:
          midiSetChannelBank(i, VS1053_BANK_DRUMS1);
          noteLength[i] = 300l;
          octiveMin[i] = 0;
          octiveMax[i] = 0;
          break;
        case INST_MODE_STRINGS_OCTIVE:
        case INST_MODE_STRINGS_ATTACK:
          midiSetChannelBank(i, VS1053_BANK_MELODY);
          noteLength[i] = 750l;
          octiveMin[i] = -1;
          octiveMax[i] = 1;
          break;      
      }
      
      midiSetChannelVolume(i, 127);
      midiSetInstrument(i, 3); // acoustic grand
    }
#endif
}

void processMIDI(unsigned long currTime) {
//  const byte minVal = 2;  // Good for lidar
  const int minVal = 8;  // Good for sharp
  
  //Serial << avgValue[0] << endl;
  for ( byte i = 0; i < Nsensor; i++ ) {
    int val = avgValue[i];
    if (val < minVal) {
      if (lastStart[i] != 0) {
        if (lastStart[i] - currTime > noteLength[i]) {
          lastStart[i] = 0;
          midiNoteOff(i, OCTIVE_4_NOTES[i],127);              
        }
      }
      continue; // filter out noise
    }

    if (currTime - lastStart[i] > noteLength[i]) {
      int octive = 0;
      uint8_t note;
      uint8_t attack;
            
      switch(instMode[i]) {
        case INST_MODE_DRUMS:
          // This is really changing instruments.  Maybe just make a list of desired per channel?
          //octive = map(val,minVal,127,octiveMin[i],octiveMax[i]);          
          octive = constrain(map(val,minVal,127,octiveMin[i],octiveMax[i]+1),octiveMin[i],octiveMax[i]);          
          note = OCTIVE_4_NOTES[i] + octive * 12;
          //if (i == 7) Serial << "s:0 val: " << val << " omin: " << octiveMin[i] << " max: " << octiveMax[i] << " oct: " << octive << endl;
          attack = 127;  
          break;
        case INST_MODE_DRUMS_ATTACK:
          note = OCTIVE_4_NOTES[i];
          //attack = map(val,minVal,127,80,127);  
          attack = constrain(map(val,minVal,127,80,128),80,127);          
          break;
        case INST_MODE_STRINGS_OCTIVE:
          //octive = map(val,minVal,128,octiveMin[i],octiveMax[i]);          
          octive = constrain(map(val,minVal,127,octiveMin[i],octiveMax[i]+1),octiveMin[i],octiveMax[i]);          
          note = OCTIVE_4_NOTES[i] + octive * 12;  
          attack = 64;
          break;
        case INST_MODE_STRINGS_ATTACK:
          note = OCTIVE_4_NOTES[i];
          //attack = map(val,minVal,127,80,127);  
          attack = constrain(map(val,minVal,127,80,128),80,127);          
          break;      
      }
      
      midiNoteOn(i, note,attack);              
      lastStart[i] = currTime;
    }
  }
    #ifndef MIDI_ENABLED
      Serial << endl;
    #endif
  //delay(10);
}

void loop(void) {
  unsigned long currTime = millis();
  // comms handling
  commsUpdate();

  // lights handling
  if ( commsConnected() ) {
    /*
    for ( byte i = 0; i < Nsensor; i++ ) {
      Serial << avgValue[i] << ",";
    }
    Serial << 255 << endl;
    */
    processMIDI(currTime);
  }
}

void commsProcess(String topic, String message) {

  //  Serial << "<- " << topic << " " << message << "\t=> ";

  if ( topic.equals(oor) ) {

    outOfRange = message.toInt();
    R = (float)(outOfRange) / log2(127.0 + 1.0);
    //Serial << "oor=" << outOfRange << "\tR=" << R;
  } else if ( topic.startsWith("skein/range/0") ) {
    // take the last character of the topic as the range index
    topic.remove(0, topic.length() - 1);
    byte i = topic.toInt();
    word m = message.toInt();

    if (i > 3) { // only process right side lidar
      // cap range
      range[i] = m < outOfRange ? m : outOfRange;
      // see: https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms/
      value[i] = round( pow(2.0, (float)(outOfRange - range[i]) / R) - 1.0 );
      // average
      avgValue[i] = value[i];
    }
    /*
    const byte smoothing = 1.5;
    avgValue[i] = (avgValue[i] * (smoothing - 1) + value[i]) / smoothing;
    */
  } else if ( topic.startsWith("skein/range/1") ) {
    // take the last character of the topic as the range index
    topic.remove(0, topic.length() - 1);
    byte i = topic.toInt();
    word m = message.toInt();

    if (i < 4) { // only process left side sharp
      // cap range
      range[i] = m < outOfRange ? m : outOfRange;
      // see: https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms/
      value[i] = round( pow(2.0, (float)(outOfRange - range[i]) / R) - 1.0 );
      // average
      avgValue[i] = value[i];
    }
    /*
    const byte smoothing = 1.5;
    avgValue[i] = (avgValue[i] * (smoothing - 1) + value[i]) / smoothing;
    */
  }  
  else {
    //Serial << F("WARNING. unknown topic. continuing.");
  }

  //Serial << endl;
}


void midiSetInstrument(uint8_t chan, uint8_t inst) {
  if (chan > 15) return;
  inst --; // page 32 has instruments starting with 1 not 0 :(
  if (inst > 127) return;
  
  VS1053_MIDI.write(MIDI_CHAN_PROGRAM | chan);  
  delay(10);
  VS1053_MIDI.write(inst);
  delay(10);
}


void midiSetChannelVolume(uint8_t chan, uint8_t vol) {
  if (chan > 15) return;
  if (vol > 127) return;
  
  VS1053_MIDI.write(MIDI_CHAN_MSG | chan);
  VS1053_MIDI.write(MIDI_CHAN_VOLUME);
  VS1053_MIDI.write(vol);
}

void midiSetChannelBank(uint8_t chan, uint8_t bank) {
  if (chan > 15) return;
  if (bank > 127) return;
  
  VS1053_MIDI.write(MIDI_CHAN_MSG | chan);
  VS1053_MIDI.write((uint8_t)MIDI_CHAN_BANK);
  VS1053_MIDI.write(bank);
}

void midiNoteOn(uint8_t chan, uint8_t n, uint8_t vel) {
  if (chan > 15) return;
  if (n > 127) return;
  if (vel > 127) return;
  
  VS1053_MIDI.write(MIDI_NOTE_ON | chan);
  VS1053_MIDI.write(n);
  VS1053_MIDI.write(vel);
}

void midiNoteOff(uint8_t chan, uint8_t n, uint8_t vel) {
  if (chan > 15) return;
  if (n > 127) return;
  if (vel > 127) return;
  
  VS1053_MIDI.write(MIDI_NOTE_OFF | chan);
  VS1053_MIDI.write(n);
  VS1053_MIDI.write(vel);
}


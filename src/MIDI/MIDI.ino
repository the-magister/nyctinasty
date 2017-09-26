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

#define BLUE_LED 2 
#define RED_LED 16 // labeled "D0" on NodeMCU boards; HIGH=off

// really need to save this to EEPROM
// should we ever need to extend this to more uCs, this will generate an offset.
const byte subsetIndex = 0;

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
String ranges = "skein/range/" + String(subsetIndex, 10) + "/#";
String oor = "skein/range/oor";

// Midi
unsigned long lastStart[Nsensor];  // Last time we started this note
bool drumMode = true;

unsigned long noteLength = 500;  // 1500 good for melody
//unsigned long noteLength = 250;  // 1500 good for melody

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

static uint8_t OCTIVE_4_NOTES[8] = { 53, 55, 57, 59, 48, 50, 50, 52};  // 12 spacing
static uint8_t DRUMS[8] = { 44, 51, 50, 38, 55, 55, 56, 39}; 

#if defined(__AVR_ATmega32U4__) || defined(ARDUINO_SAMD_FEATHER_M0) || defined(TEENSYDUINO) || defined(ARDUINO_STM32_FEATHER)
  #define VS1053_MIDI Serial1
#elif defined(ESP32)
  HardwareSerial Serial1(2);
  #define VS1053_MIDI Serial1
#elif defined(ESP8266)
  #define VS1053_MIDI Serial
#endif

#define MIDI_ENABLED true

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
//      midiSetChannelBank(i, VS1053_BANK_MELODY);
      midiSetChannelBank(i, VS1053_BANK_DRUMS1);
      midiSetChannelVolume(i, 127);
//      midiSetInstrument(i, VS1053_GM1_HARPSICORD);
      midiSetInstrument(i, 3); // acoustic grand
    }
    /*
    for(byte i=0; i < Nsensor/2; i++) {
      midiSetChannelBank(i, VS1053_BANK_MELODY);
      midiSetChannelVolume(i, 127);
      midiSetInstrument(i, VS1053_GM1_HARPSICORD);
    }

    for(byte i=Nsensor/2; i < Nsensor; i++) {
      midiSetChannelBank(i, VS1053_BANK_DRUMS2);
      midiSetChannelVolume(i, 127);
      midiSetInstrument(i, VS1053_GM1_HARPSICORD);
    }
    */
#endif
}

void processMIDI(unsigned long currTime) {
  const byte minVal = 10;
  
  //Serial << avgValue[0] << endl;
  for ( byte i = 0; i < Nsensor; i++ ) {
    byte val = avgValue[i];
    if (val < minVal) {
      if (lastStart[i] != 0) {
        if (lastStart[i] - currTime > noteLength) {
          lastStart[i] = 0;
          midiNoteOff(i, OCTIVE_4_NOTES[i],127);              
        }
      }
      continue; // filter out noise
    }

    if (currTime - lastStart[i] > noteLength) {
      byte octive = 0;
      const byte intervals = 3;
      /*
      // use location to determine octive
      if (val < 42) octive = -1;
      else if (val < 84) octive = 0;
      else octive = 1;
*/
      byte rval = val - minVal;
      octive = (val+1) / (128 / (intervals)) - (intervals-1);
       
      byte note = OCTIVE_4_NOTES[i] + octive * 12;
      midiNoteOn(i, note,127);              
      lastStart[i] = currTime;
    }
    /*
    if (avgValue[i] > 10) {
        byte val = avgValue[i];
        if (val > 127) val = 127;
        if (val < 64) val = 64;
  #ifdef MIDI_ENABLED      
        midiNoteOn(i, DRUMS[i], val);
//        midiNoteOn(i, val, val);
  #endif 
  
      //digitalWrite(BLUE_LED,LOW);
    } else {
      //digitalWrite(BLUE_LED,HIGH);
  #ifdef MIDI_ENABLED        
          midiNoteOff(i,DRUMS[i],127);
  #endif 
    }
    */
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
  } else if ( topic.startsWith("skein/range") ) {
    // take the last character of the topic as the range index
    topic.remove(0, topic.length() - 1);
    byte i = topic.toInt();
    word m = message.toInt();

    // cap range
    range[i] = m < outOfRange ? m : outOfRange;
    // see: https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms/
    value[i] = round( pow(2.0, (float)(outOfRange - range[i]) / R) - 1.0 );
    // average
    avgValue[i] = value[i];
    /*
    const byte smoothing = 1.5;
    avgValue[i] = (avgValue[i] * (smoothing - 1) + value[i]) / smoothing;
    */
  } else {
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


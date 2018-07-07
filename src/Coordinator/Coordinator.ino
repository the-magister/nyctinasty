// IDE Settings:
// Tools->Board : "WeMos D1 R2 & mini"
// Tools->Flash Size : "4M (3M SPIFFS)"
// Tools->CPU Frequency : "160 MHz"
// Tools->Upload Speed : "921600"

#include <Streaming.h>
#include <Metro.h>
#include <SoftwareSerial.h>
#include <FiniteStateMachine.h>
#include "Nyctinasty_Messages.h"
#include "Nyctinasty_Comms.h"

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts\Picopixel.h>
#define FH 6 // six pixels high per character
#define FW 4 // four pixels wide per character

// spammy?
#define SHOW_PLAYER_DEBUG false
#define SHOW_COORD_DEBUG false
#define DUMP_DISTANCE_DATA false

// ##################
// Master settings:

// don't allow wild oscillations in state transitions. See resetTransitionLockout()
const uint32_t startupMinTime = 3UL * 1000UL;
const uint32_t lonelyMinTime = 1UL * 1000UL;
const uint32_t ohaiMinTime = 10UL * 1000UL;
const uint32_t goodnufMinTime = 10UL * 1000UL;
const uint32_t goodjobMinTime = 10UL * 1000UL;
const uint32_t winningMinTime = 10UL * 1000UL;
const uint32_t fanfareMinTime = 30UL * 1000UL;

// defined below
double calculateSmoothing(double updateInterval, double halfTime);

// storage for running metrics
double meanProxAvg[N_ARCH] = {30, 30, 30};
double meanProxSD[N_ARCH] = {5, 5, 5};
double meanProxCV[N_ARCH] = {5/30*100, 5/30*100, 5/30*100};
double meanProxVar[N_ARCH] = {5*5, 5*5, 5*5};

// See decidePlayerState()
const double isPlayerThreshold[N_ARCH] = {40, 40, 40};
double playerSmoothing = calculateSmoothing( DISTANCE_SAMPLING_RATE, 1000.0 );

// See decideCoordination()
const double areCoordinatedThreshold[N_ARCH] = {25, 25, 25};

// if we "win", how long do we do stuff for?
const uint32_t fanfareDuration = 30UL * 1000UL; // seconds

// ##################

// My role and arch number
NyctRole myRole = Coordinator; // see Nyctinasty_Comms.h; set N_ROLES to pull from EEPROM

// wire it up
// pin def: C:\Users\MikeD\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\2.3.0\variants\d1_mini
// D4, GPIO2, BUILTIN_LED
// SCL GPIO5 is D1
// SDA GPIO4 is D2
#define OLED_RESET 0  // GPIO0 is D3
Adafruit_SSD1306 display(OLED_RESET);

#if (SSD1306_LCDHEIGHT != 48)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

// sizes
const uint16_t H = SSD1306_LCDHEIGHT;

// comms
NyctComms comms;

// define a state for every systemState
void startupEnter(), startupUpdate(); State Startup = State(startupEnter, startupUpdate, NULL);
void lonelyUpdate(), lonelyEnter(); State Lonely = State(lonelyEnter, lonelyUpdate, NULL);
void ohaiEnter(), ohaiUpdate(); State Ohai = State(ohaiEnter, ohaiUpdate, NULL);
void goodnufEnter(), goodnufUpdate(); State Goodnuf = State(goodnufEnter, goodnufUpdate, NULL);
void goodjobEnter(), goodjobUpdate(); State Goodjob = State(goodjobEnter, goodjobUpdate, NULL);
void winningEnter(), winningUpdate(); State Winning = State(winningEnter, winningUpdate, NULL);
void fanfareEnter(), fanfareUpdate(); State Fanfare = State(fanfareEnter, fanfareUpdate, NULL);
void reboot() {
  comms.reboot();
}; State Reboot = State(reboot);
FSM stateMachine = FSM(Startup); // initialize state machine

// we can manually peg states
boolean stateLock = false;

// publish system state
SystemCommand sC;

// distance updates receive as this structure as this topic
typedef struct {
  boolean hasUpdate = false;
  SepalArchDistance dist;
} sAD_t;
sAD_t sAD[N_ARCH];

// frequency updates receive as this structure as this topic
typedef struct {
  boolean hasUpdate = false;
  SepalArchFrequency freq;
} sAF_t;
sAF_t sAF[N_ARCH];

typedef struct {
  boolean hasUpdate = false;
  CannonTrigger tr;
} sCT_t;
sCT_t sCT;

void setup() {
  // for local output
  delay(500);
  Serial.begin(115200);
  Serial.setTimeout(5);

  Serial << endl << endl << endl << F("Startup begins.") << endl;

  // configure comms
  comms.begin(myRole);
  myRole = comms.getRole();

  // subscribe
  for ( byte i = 0; i < N_ARCH; i++ ) {
    comms.subscribe(&sAD[i].dist, &sAD[i].hasUpdate, i);
    comms.subscribe(&sAF[i].freq, &sAF[i].hasUpdate, i);
  }
  comms.subscribe(&sCT.tr, &sCT.hasUpdate);

  // for random numbers
  randomSeed(analogRead(0));

  // print out
  Serial << F("playerSmoothing=") << String(playerSmoothing,8) << endl;


  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 64x48)
  // init done

  // update speed
//  Wire.setClock(400000L);
  Wire.setClock(800000L);

  // fonts
  display.setFont(&Picopixel);
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // 90 degree rotation
//  display.setRotation(1);

  // Clear the buffer.
  display.clearDisplay();
  display.display();

  Serial << F("LCD setup complete.") << endl;

  Serial << F("Startup complete.") << endl;
}

// check for serial input
void lookForSerialCommands() {
  if ( ! Serial.available() ) return;

  static boolean trigger = false;

  stateLock = true;

  char c = Serial.read();
  switch (c) {
    case 's': stateMachine.transitionTo(Startup); break;
    case 'l': stateMachine.transitionTo(Lonely); break;
    case 'o': stateMachine.transitionTo(Ohai); break;
    case 'n': stateMachine.transitionTo(Goodnuf); break;
    case 'g': stateMachine.transitionTo(Goodjob); break;
    case 'w': stateMachine.transitionTo(Winning); break;
    case 'f': stateMachine.transitionTo(Fanfare); break;
    case 'r': stateMachine.transitionTo(Reboot); break;
    case 'L':
      stateLock = false;
      Serial << "State lock=" << stateLock << endl;

      break;
    case 'T': {
        trigger = !trigger;
        CannonTrigger trig;
        trig.left = trigger ? TRIGGER_ON : TRIGGER_OFF;
        trig.right = trigger ? TRIGGER_ON : TRIGGER_OFF;

        comms.publish(&trig);
        Serial << "Trigger=" << trigger << endl;
        break;
      }
    case 'P': {
        int foo = Serial.parseInt();
        byte player = constrain(foo, 0, 2);
        sC.isPlayer[player] = !sC.isPlayer[player];
        sendSettings();
        break;
      }
    case 'C': {
        int foo = Serial.parseInt();
        byte pair = constrain(foo, 0, 2);
        sC.areCoordinated[pair] = !sC.areCoordinated[pair];
        // must have players if coordinated
        if ( sC.areCoordinated[0] ) sC.isPlayer[0] = sC.isPlayer[1] = true;
        if ( sC.areCoordinated[1] ) sC.isPlayer[1] = sC.isPlayer[2] = true;
        if ( sC.areCoordinated[2] ) sC.isPlayer[2] = sC.isPlayer[0] = true;
        sendSettings();
        break;
      }
    case '\n': break;
    case '\r': break;
    default:
      Serial << F("(s)tartup, (l)onely, (o)hai, good(n)uf, (g)oodjob, (w)inning, (f)anfare, (r)eboot.") << endl;
      Serial << F("un(L)ock game state. (T)rigger cannon.") << endl;
      Serial << F("(P)(0-2) toggle player state: A0, A1, A2.") << endl;
      Serial << F("(C)(0-2) toggle player state: A0:A1, A1:A2, A2:A0.") << endl;
      sendSettings();
      break;
  }
}

void loop() {
  // comms handling
  comms.update();

  // dump distance data?
  if ( DUMP_DISTANCE_DATA ) {

    const char sep = ',';
    static String msg = "NOP";

    if (Serial.available()) {
      delay(100);
      msg = Serial.readString();
    }

    for ( byte a = 0; a < N_ARCH; a++ ) {
      if ( sAD[a].hasUpdate ) {
        Serial << msg << sep << millis() << sep << a;
        // loop across each sensor
        for ( byte s = 0; s < N_SENSOR; s++ ) {
          Serial << sep << sAD[a].dist.prox[s];

        }
        Serial << endl;
        sAD[a].hasUpdate = false;
      }
    }
  } else {
    lookForSerialCommands();
  }

  // is the state locked out?
  if ( stateLock ) {
    resetTransitionLockout();
  } else {
    if ( sAD[0].hasUpdate ) decidePlayerState(0);
    if ( sAD[1].hasUpdate ) decidePlayerState(1);
    if ( sAD[2].hasUpdate ) decidePlayerState(2);

    if ( sAD[0].hasUpdate || sAD[1].hasUpdate ) decideCoordination(0,1); // 0,1 -> pair 0
    if ( sAD[1].hasUpdate || sAD[2].hasUpdate ) decideCoordination(1,2); // 1,2 -> pair 1
    if ( sAD[2].hasUpdate || sAD[0].hasUpdate ) decideCoordination(2,0); // 2,0 -> pair 2

    sAD[0].hasUpdate = sAD[1].hasUpdate = sAD[2].hasUpdate = false;

    if( sCT.hasUpdate ) decideTrigger();
    sCT.hasUpdate = false;
  }

  // update LCD
  showSettings();

  // state machine
  stateMachine.update();
}


/*
  P = # players = 0,1,2,3
  C = # coordinated pairs = 0,1,2,3
  PC = P+C = 0..6

  PC  State       Water
  0   Lonely
  1   Ohai  
  2   Goodnuff    Fountain + 30 %Duty
  3     Goodnuff
  4   Goodjob     60% Duty
  5   Winning     100% Duty
  6   Fanfare     100% Duty
*/

// I enumerated these, as I can use symmetric ">=" and "<" below to get the logic right for transition counts
const byte pc_Ohai = 1;
const byte pc_Goodnuf = 2;
const byte pc_Goodjob = 4;
const byte pc_Winning = 5;
const byte pc_Fanfare = 6;

// STARTUP state
void startupEnter() {
  genericStateEnter(STARTUP);
}
void startupUpdate() {
  if ( ! transitionLockoutExpired() ) return;

  stateMachine.transitionTo(Lonely);
}

// LONELY state
void lonelyEnter() {
  genericStateEnter(LONELY);
}
void lonelyUpdate() {
  if ( ! transitionLockoutExpired() ) return;

  // no demotion possible

  // promote
//  if ( ( totalPlayersGE(playerCount_Ohai) || coordPlayersGE(coordCount_Ohai) ) ) stateMachine.transitionTo(Ohai);
  if ( playerCoordinationGE(pc_Ohai) ) stateMachine.transitionTo(Ohai);

}

// OHAI state
void ohaiEnter() {
  genericStateEnter(OHAI);
}
void ohaiUpdate() {
  if ( ! transitionLockoutExpired() ) return;

  // demote
//  if ( ! ( totalPlayersGE(playerCount_Ohai) || coordPlayersGE(coordCount_Ohai) ) ) stateMachine.transitionTo(Lonely);
  if ( ! playerCoordinationGE(pc_Ohai) ) stateMachine.transitionTo(Lonely);

  // promote
//  if ( ( totalPlayersGE(playerCount_Goodnuf) || coordPlayersGE(coordCount_Goodnuf) ) ) stateMachine.transitionTo(Goodnuf);
  if ( playerCoordinationGE(pc_Goodnuf) ) stateMachine.transitionTo(Goodnuf);
}

// GOODNUF state
void goodnufEnter() {
  genericStateEnter(GOODNUF);
}
void goodnufUpdate() {
  if ( ! transitionLockoutExpired() ) return;

  // demote
//  if ( ! ( totalPlayersGE(playerCount_Goodnuf) || coordPlayersGE(coordCount_Goodnuf) ) ) stateMachine.transitionTo(Ohai);
  if ( ! playerCoordinationGE(pc_Goodnuf) ) stateMachine.transitionTo(Ohai);

  // promote
//  if ( ( totalPlayersGE(playerCount_Goodjob) || coordPlayersGE(coordCount_Goodjob) ) ) stateMachine.transitionTo(Goodjob);
  if ( playerCoordinationGE(pc_Goodjob) ) stateMachine.transitionTo(Goodjob);
}

// GOODJOB state
void goodjobEnter() {
  genericStateEnter(GOODJOB);
}
void goodjobUpdate() {
  if ( ! transitionLockoutExpired() ) return;

  // demote
//  if ( ! ( totalPlayersGE(playerCount_Goodjob) || coordPlayersGE(coordCount_Goodjob) ) ) stateMachine.transitionTo(Goodnuf);
  if ( ! playerCoordinationGE(pc_Goodjob) ) stateMachine.transitionTo(Goodnuf);

  // promote
//  if ( ( totalPlayersGE(playerCount_Winning) && coordPlayersGE(coordCount_Winning) ) ) stateMachine.transitionTo(Winning);
  if ( playerCoordinationGE(pc_Winning) ) stateMachine.transitionTo(Winning);
}

// WINNING state
void winningEnter() {
  genericStateEnter(WINNING);
}
void winningUpdate() {
  if ( ! transitionLockoutExpired() ) return;

  // demote
//  if ( ! ( totalPlayersGE(playerCount_Winning) && coordPlayersGE(coordCount_Winning) ) ) stateMachine.transitionTo(Goodjob);
  if ( ! playerCoordinationGE(pc_Winning) ) stateMachine.transitionTo(Goodjob);

  // promote
//  if ( ( totalPlayersGE(playerCount_Fanfare) && coordPlayersGE(coordCount_Fanfare) ) ) stateMachine.transitionTo(Fanfare);
  if ( playerCoordinationGE(pc_Fanfare) ) stateMachine.transitionTo(Fanfare);
}

// FANFARE state
Metro fanfareInterval(fanfareDuration);
void fanfareEnter() {
  genericStateEnter(FANFARE);
  fanfareInterval.reset();
}
void fanfareUpdate() {
  if ( fanfareInterval.check() ) stateMachine.transitionTo(Lonely);
}

// algorithm for player detection
double meanSensorProx(uint16_t (&x)[N_SENSOR]) {
  // compute mean
  double m = 0;
  for ( byte s = 0; s < N_SENSOR; s++ ) m += x[s];
  m /= (double)N_SENSOR;
  return(m);
}
double sdSensorProx(uint16_t (&x)[N_SENSOR]) {
  // compute sd
  double m = meanSensorProx(x);
  double sd = 0;
  for ( byte s = 0; s < N_SENSOR; s++ ) sd += pow((double)x[s] - m, 2.0);
  sd = pow(sd/(double)(N_SENSOR-1), 0.5);
  return( sd );
}
// watch the player state and also count the number of times in that same state (persistence)
void decidePlayerState(byte arch) {
  // current, immediate value
  double m = meanSensorProx(sAD[arch].dist.prox);
  
  // smooth it
  performSmoothing(meanProxAvg[arch], meanProxVar[arch], m, playerSmoothing);

  // check for player
  boolean isPlayer = meanProxAvg[arch] > isPlayerThreshold[arch];

  //  if( arch==2 ) Serial << "decidePlayerState. arch=" << arch << " power=" << power << " thresh=" << smoothedPower[arch] * isPlayerThreshold[arch] << " state=" << sC.isPlayer[arch] << endl;

  // check for state change
  if ( isPlayer != sC.isPlayer[arch] ) {
    sC.isPlayer[arch] = isPlayer;
    Serial << "decidePlayerState.  isPlayer? A" << arch << "=" << isPlayer << endl;
    sendSettings();
  }
  
  if( SHOW_PLAYER_DEBUG ) {
    Serial << "decidePlayer. ";
    Serial << " player0=" << meanProxAvg[0] << " (" << meanProxVar[0] << ") =" << sC.isPlayer[0];
    Serial << " player1=" << meanProxAvg[1] << " (" << meanProxVar[1] << ") =" << sC.isPlayer[1];
    Serial << " player2=" << meanProxAvg[2] << " (" << meanProxVar[2] << ") =" << sC.isPlayer[2];
    Serial << endl;
  }
  
}

// algorithm for coordination detection
void decideCoordination(byte arch1, byte arch2) {
  // which pair?
  byte pair = coordIndex(arch1, arch2);
 
  // compute sd from variance for each player
  meanProxSD[arch1] = meanProxVar[arch1]>0.0 ? sqrt(meanProxVar[arch1]) : 0.0;
  meanProxSD[arch2] = meanProxVar[arch2]>0.0 ? sqrt(meanProxVar[arch2]) : 0.0;

  // compute cv = sd/mean; 1.0 means 100% CV.
  meanProxCV[arch1] = meanProxSD[arch1]/meanProxAvg[arch1]*100.0;
  meanProxCV[arch2] = meanProxSD[arch2]/meanProxAvg[arch2]*100.0;

  // check coordination
  boolean areCoordinated = false;

  // if there are players in these arches, then they may be coordinated
  if ( 
    sC.isPlayer[arch1] == true && meanProxSD[arch1] > areCoordinatedThreshold[arch1] &&
    sC.isPlayer[arch2] == true && meanProxSD[arch2] > areCoordinatedThreshold[arch2] ) {
//    sC.isPlayer[arch1] == true && meanProxCV[arch1] > areCoordinatedThreshold[arch1] &&
//    sC.isPlayer[arch2] == true && meanProxCV[arch2] > areCoordinatedThreshold[arch2] ) {
    areCoordinated = true;
  }

  // set state
  if ( areCoordinated != sC.areCoordinated[pair] ) {
    Serial << "decideCoordination.  areCoordinated? P" << pair << "=" << areCoordinated << endl;
    sC.areCoordinated[pair] = areCoordinated;
    sendSettings();
  }  

  if( SHOW_COORD_DEBUG ) {
    Serial << "decideCoordination.";
    Serial << " pair0=" << meanProxSD[0];
    Serial << " pair1=" << meanProxSD[1];
    Serial << " pair2=" << meanProxSD[2];
    Serial << endl;
  }

}

void decideTrigger() {
  static CannonTrigger tr;
  if( tr.left != sCT.tr.left || tr.right != sCT.tr.right )
    Serial << "decideTrigger. left=" << sCT.tr.left << " right=" << sCT.tr.right << endl;
  tr = sCT.tr;
}

double calculateSmoothing(double updateInterval, double halfTime) {
  // smooth
  // updateInterval [=] ms; delta time between update to this function
  // halfTime [=] ms; delta time for smoothed signal to transition halfway to new value
  double samples = halfTime / updateInterval / log(2.0);
  double alpha = 1.0-(samples-1.0)/samples;
  return( alpha );
}
// http://people.ds.cam.ac.uk/fanf2/hermes/doc/antiforgery/stats.pdf
void performSmoothing(double &mean, double &var, double x, double alpha) {
  double diff = x - mean;
  double incr = alpha * diff;
  mean = mean + incr;
  var = (1.0-alpha)*(var + diff*incr);
}

byte coordIndex(byte arch1, byte arch2) {
  switch ( arch1 + arch2 ) {
    case 1: return (0); break; // A0:A1 or A1:A0
    case 3: return (1); break; // A1:A2 or A2:A1
    case 2: return (2); break; // A2:A0 or A0:A2
    default:
      Serial << F("coordIndex.  ERROR. arch1=") << arch1 << F(" arch2=") << arch2 << endl;
      return ( 0 );
      break;
  }
}

byte totalPlayersGE(byte th) {
  return ( (sC.isPlayer[0] + sC.isPlayer[1] + sC.isPlayer[2]) >= th );
}
byte coordPlayersGE(byte th) {
  return ( (sC.areCoordinated[0] + sC.areCoordinated[1] + sC.areCoordinated[2]) >= th );
}

byte playerCoordinationGE(byte th) {
  return(
    sC.isPlayer[0] + sC.isPlayer[1] + sC.isPlayer[2] +
    sC.areCoordinated[0] + sC.areCoordinated[1] + sC.areCoordinated[2]
    
    >= th
  );
}

// don't allow wild oscillations in state transitions
uint32_t nextTransitionAllowedAt;
uint32_t transitionLockoutTime;
void resetTransitionLockout() {
  nextTransitionAllowedAt = millis() + transitionLockoutTime;
}
boolean transitionLockoutExpired() {
  return ( millis() > nextTransitionAllowedAt );
}

void genericStateEnter(systemState state) {
  switch (state) {
    case STARTUP: transitionLockoutTime = startupMinTime; break;  //  all roles start here
    case LONELY: transitionLockoutTime = lonelyMinTime; break;   // 0 players
    case OHAI: transitionLockoutTime = ohaiMinTime; break;   // 1 players
    case GOODNUF: transitionLockoutTime = goodnufMinTime; break;  // 2 players
    case GOODJOB: transitionLockoutTime = goodjobMinTime; break;  // 3 players or 2 players coordinated
    case WINNING: transitionLockoutTime = winningMinTime; break;  // 3 players and 2 players coordinated
    case FANFARE: transitionLockoutTime = fanfareMinTime; break;  // 3 players and 3 players coordinated
  }
  
  resetTransitionLockout();

  sC.state = state;

  sendSettings();
}

void sendSettings() {
  Serial << "Settings. state=";
  switch (sC.state) {
    case STARTUP: Serial << "STARTUP"; break;  //  all roles start here

    case LONELY: Serial << "LONELY"; break;   // 0 players
    case OHAI: Serial << "OHAI"; break;   // 1 players
    case GOODNUF: Serial << "GOODNUF"; break;  // 2 players
    case GOODJOB: Serial << "GOODJOB"; break;  // 3 players or 2 players coordinated
    case WINNING: Serial << "WINNING"; break;  // 3 players and 2 players coordinated
    case FANFARE: Serial << "FANFARE"; break;  // 3 players and 3 players coordinated

    case REBOOT: Serial << "REBOOT"; break;   //  trigger to reboot
  }

  Serial << ". isPlayer? A0=" << sC.isPlayer[0];
  Serial << " A1=" << sC.isPlayer[1];
  Serial << " A2=" << sC.isPlayer[2];

  Serial << ". areCoord? A01=" << sC.areCoordinated[0];
  Serial << " A12=" << sC.areCoordinated[1];
  Serial << " A20=" << sC.areCoordinated[2];
  Serial << endl;

  comms.publish(&sC);
}

void showSettings() {
  static Metro updateInterval(333UL);
  if( !updateInterval.check() ) return;

  // show status; about 40 ms to show this update.
  // ~8ms with 800Khz
  display.clearDisplay();
  
  // shift down
  display.setCursor(0,FH);
  
  switch (sC.state) {
    case STARTUP: display.print("STARTUP"); break;  //  all roles start here
    case LONELY:  display.print("LONELY "); break;   // 0 players
    case OHAI:    display.print("OHAI   "); break;   // 1 players
    case GOODNUF: display.print("GOODNUF"); break;  // 2 players
    case GOODJOB: display.print("GOODJOB"); break;  // 3 players or 2 players coordinated
    case WINNING: display.print("WINNING"); break;  // 3 players and 2 players coordinated
    case FANFARE: display.print("FANFARE"); break;  // 3 players and 3 players coordinated
    case REBOOT:  display.print("REBOOT "); break;   //  trigger to reboot
  }
  if( sCT.tr.left ) display.print(" L");
  else              display.print("  ");
  if( sCT.tr.right ) display.println(" R");
  else               display.println("  ");

  for(byte i=0; i<N_ARCH; i++) {
    String p = String("A") + String(i,10) + " ";
    p += String(meanProxAvg[i], 0);
    p += String(sC.isPlayer[i] ? " Y " : " n "); 
    p += String(" (") + String(meanProxSD[i], 0) + String(")") ;
//    p += String(" (") + String(meanProxCV[i], 0) + String(")") ;
    p += String(meanProxSD[i] > areCoordinatedThreshold[i] ? " Y " : " n ");
//    p += String(meanProxCV[i] > areCoordinatedThreshold[i] ? " Y " : " n ");
    display.println(p);
  }

  String p = String("Coord: ");
  p += String(sC.areCoordinated[0] ? " Y " : " n ");
  p += String(sC.areCoordinated[1] ? " Y " : " n ");
  p += String(sC.areCoordinated[2] ? " Y " : " n ");
  display.println(p);

  // push
  yield(); comms.update(); 
  display.display();
  yield(); comms.update(); 

  updateInterval.reset();
}



/*
  static String msg = "NOP";
  if (Serial.available()) {
   delay(100);
   msg = Serial.readString();
  }

  const char sep = ',';
  // loop across each arch's information
  if( sAF[0].hasUpdate && sAF[1].hasUpdate && sAF[2].hasUpdate ){
   uint32_t now = millis();

   if( SHOW_FREQ_DEBUG ) {
     // loop across each arch's information
     for( byte a=0; a<N_ARCH; a++ ) {
       // loop across each sensor
       for( byte s=0; s<N_SENSOR; s++ ) {
         Serial << msg << sep << now << sep << a << sep << s;
         Serial << sep << sAF[a].freq.peakFreq[s];
         Serial << sep << sAF[a].freq.avgPower[s];
         for( byte b=0; b<N_FREQ_BINS; b++ ) {
           Serial << sep << sAF[a].freq.power[s][b];
         }
         Serial << endl;
       }
     }
   }

   if( SHOW_COORD_DEBUG ) {
     Serial << msg << sep << now;
  //      concordanceByPower();
     Serial << endl;
   }

   sAF[0].hasUpdate = sAF[1].hasUpdate = sAF[2].hasUpdate = false;
  }
*/

double correlation_fast(uint16_t (&x)[N_SENSOR], uint16_t (&y)[N_SENSOR]) {

  // sample size
  const double n = N_SENSOR;

  // loop over bins
  double sumXY = 0;
  double sumX = 0;
  double sumY = 0;
  double sumX2 = 0;
  double sumY2 = 0;
  for ( byte b = 0; b < (byte)n; b++ ) {
    sumXY += x[b] * y[b];
    sumX += x[b];
    sumY += y[b];
    sumX2 += x[b] * x[b];
    sumY2 += y[b] * y[b];
  }

  double n1 = n * sumXY;
  //  Serial << "n1 " << n1 << endl;
  double n2 = sumX * sumY;
  //  Serial << "n2 " << n2 << endl;
  double d1 = sqrt(n * sumX2 - sumX * sumX);
  //  Serial << "d1 " << d1 << endl;
  double d2 = sqrt(n * sumY2 - sumY * sumY);
  //  Serial << "d2 " << d2 << endl;
  return ( (n1 - n2) / d1 / d2 );
}


double correlation_fast(double (&x)[N_FREQ_BINS], double (&y)[N_FREQ_BINS]) {

  // sample size
  const double n = N_FREQ_BINS;

  // loop over bins
  double sumXY = 0;
  double sumX = 0;
  double sumY = 0;
  double sumX2 = 0;
  double sumY2 = 0;
  for ( byte b = 0; b < N_FREQ_BINS; b++ ) {
    sumXY += x[b] * y[b];
    sumX += x[b];
    sumY += y[b];
    sumX2 += x[b] * x[b];
    sumY2 += y[b] * y[b];
  }

  double n1 = n * sumXY;
  //  Serial << "n1 " << n1 << endl;
  double n2 = sumX * sumY;
  //  Serial << "n2 " << n2 << endl;
  double d1 = sqrt(n * sumX2 - sumX * sumX);
  //  Serial << "d1 " << d1 << endl;
  double d2 = sqrt(n * sumY2 - sumY * sumY);
  //  Serial << "d2 " << d2 << endl;
  return ( (n1 - n2) / d1 / d2 );
}

double correlation_slow(double (&x)[N_FREQ_BINS], double (&y)[N_FREQ_BINS]) {

  // sample size
  const double n = N_FREQ_BINS;

  //  Serial << endl;

  // average
  double xbar = 0;
  double ybar = 0;
  for ( byte b = 0; b < N_FREQ_BINS; b++ ) {
    //    Serial << x[b] << ", " << y[b] << endl;
    xbar += x[b];
    ybar += y[b];
  }
  //  Serial << endl;
  xbar /= n;
  //  Serial << "xbar " << xbar << endl;
  ybar /= n;
  //  Serial << "ybar " << ybar << endl;

  double num = 0;
  double den1 = 0;
  double den2 = 0;
  for ( byte b = 0; b < N_FREQ_BINS; b++ ) {
    double xdel = x[b] - xbar;
    double ydel = y[b] - ybar;
    num += xdel * ydel;
    den1 += pow(xdel, 2.0);
    den2 += pow(ydel, 2.0);
  }
  //  Serial << "num " << num << endl;

  //  Serial << "den1 " << den1 << endl;
  den1 = sqrt(den1);
  //  Serial << "den1 " << den1 << endl;

  //  Serial << "den2 " << den2 << endl;
  den2 = sqrt(den2);
  //  Serial << "den2 " << den2 << endl;

  double ret = num / den1 / den2;
  //  Serial << "ret " << ret << endl;

  return ( ret );
}

void concordanceByPower() {

  // sum the power information across each sensor for each arch
  double power[N_ARCH][N_FREQ_BINS] = {{0}};
  double avgPower[N_ARCH] = {0};
  for ( byte a = 0; a < N_ARCH; a++ ) {
    for ( byte b = 0; b < N_FREQ_BINS; b++ ) {
      for ( byte s = 0; s < N_SENSOR; s++ ) {
        power[a][b] += sAF[a].freq.power[s][b];
      }
      //      Serial << power[a][b] << " ";
      power[a][b] = log(power[a][b]); // weight the high frequency stuff more
    }
    for ( byte s = 0; s < N_SENSOR; s++ ) {
      avgPower[a] += sAF[a].freq.avgPower[s];
    }
    //    Serial << endl;
  }

  // compute
  double corr01 = correlation_fast(power[0], power[1]); yield();
  double corr12 = correlation_fast(power[1], power[2]); yield();
  double corr20 = correlation_fast(power[2], power[0]); yield();

  corr01 = pow(corr01, 2.0);
  corr12 = pow(corr12, 2.0);
  corr20 = pow(corr20, 2.0);

  char sep = ',';
  Serial << sep << corr01 << sep << corr12 << sep << corr20;
  Serial << sep << avgPower[0] << sep << avgPower[1] << sep << avgPower[2];

  uint16_t thresh = 40000;
  double thresh2 = 0.6;
  if ( avgPower[0] > thresh && avgPower[2] > thresh && corr20 > 0.6 ) {
    Serial << sep << "C02";
  } else {
    Serial << sep << "c02";
  }

  /*
    corr01 = correlation_slow(power[0], power[1]);
    corr12 = correlation_slow(power[1], power[2]);
    corr20 = correlation_slow(power[2], power[0]);

    Serial << endl << corr01 << sep << corr12 << sep << corr20 << endl;
  */
}


/*
  void getConcordance() {

  // this is all wrong.  need 2d correlation analysis
  // https://en.wikipedia.org/wiki/Two-dimensional_correlation_analysis

  // maybe we want to push this to a separate microcontroller, Sepal_Concordance
  // alternately, each Arch can calculate their concordance clockwise (or ccw) and publish
  // that

  // normalize weights to sum to 1.0
  static float vecWeight[N_FREQ_BINS] = {0.0};
  // scale covariance terms
  static float covTerm = 0.0;

  // we can compute a few things once.
  static float isInitialized = false;
  if ( ! isInitialized ) {
    float sumWeight = 0;
    for ( byte b = 0; b < N_FREQ_BINS; b++ ) {
      sumWeight += WeightFrequency[b];
    }
    for ( byte b = 0; b < N_FREQ_BINS; b++ ) {
      vecWeight[b] = (float)WeightFrequency[b] / sumWeight;
      covTerm += vecWeight[b] * vecWeight[b];
    }
    covTerm = 1.0 / (1.0 - covTerm);
    isInitialized = true;
  }

  // weighted mean vector
  float xBar[N_ARCH] = {0.0};
  for ( byte j = 0; j < N_ARCH; j++ ) {
    for ( byte i = 0; i < N_FREQ_BINS; i++ ) {
      // this doesn't look quite right:
      xBar[j] += vecWeight[j] * (float)sAF[myArch].freq.power[i][j];
    }
  }

  // weighted covariance matrix
  float Cov[N_ARCH][N_ARCH] = {0.0};
  for ( byte a1 = 0; a1 < N_ARCH; a1++ ) {
    for ( byte a2 = a1; a2 < N_ARCH; a2++ ) { // symmetry
      float sum = 0.0;
      for ( byte b = 0; b < N_FREQ_BINS; b++ ) {
        sum += vecWeight[b] *
               ((float)sAF[a1].freq[b] - xBar[a1]) *
               ((float)sAF[a2].freq[b] - xBar[a2]);
      }
      Cov[a1][a2] = covTerm * sum;
    }
  }

  // weighted correlation matrix
  float Cor[N_ARCH][N_ARCH] = {0.0};
  float Is[N_ARCH];
  for ( byte a = 0; a < N_ARCH; a++ ) {
    Is[a] = 1.0 / pow(Cov[a][a], 0.5);
  }
  for ( byte a1 = 0; a1 < N_ARCH; a1++ ) {
    for ( byte a2 = a1; a2 < N_ARCH; a2++ ) { // symmetry
      if ( a1 != a2 ) Cor[a1, a2] = Is[a1] * Cov[a1][a2] * Is[a2];
    }
  }

  // save
  switch ( myArch ) {
    case 0: // A. B is next. C is prev.
      concordNext = Cor[0, 1]; // corr.AB = mat.cor[1,2]
      concordPrev = Cor[0, 2]; // corr.CA = mat.cor[1,3]
      break;
    case 1: // B. C is next. A is prev.
      concordNext = Cor[1, 2]; // corr.BC = mat.cor[2,3]
      concordPrev = Cor[0, 1]; // corr.AB = mat.cor[1,2]
      break;
    case 2: // C. A is next. B is prev.
      concordNext = Cor[0, 2]; // corr.CA = mat.cor[1,3]
      concordPrev = Cor[1, 2]; // corr.BC = mat.cor[2,3]
      break;
  }
  concordTotal = (Cor[0, 1] + Cor[1, 2] + Cor[0, 2]) / 3.0;

  }
*/







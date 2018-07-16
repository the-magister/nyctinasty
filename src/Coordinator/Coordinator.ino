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
const uint32_t ohaiMinTime = 5UL * 1000UL;
const uint32_t goodnufMinTime = 5UL * 1000UL;
const uint32_t goodjobMinTime = 5UL * 1000UL;
const uint32_t winningMinTime = 5UL * 1000UL;
const uint32_t fanfareMinTime = 30UL * 1000UL;

// storage for running metrics
double meanProxAvg[N_ARCH] = {30, 30, 30};
double meanProxSD[N_ARCH] = {5, 5, 5};
double meanProxCV[N_ARCH] = {5/30*100, 5/30*100, 5/30*100};
double meanProxVar[N_ARCH] = {5*5, 5*5, 5*5};
double ratioCVAvg[N_ARCH] = {1.0, 1.0, 1.0};
double ratioCVVar[N_ARCH] = {1.0, 1.0, 1.0};

// See decidePlayerState()
// We require that the average power from the sensors exceed a threshold to score a player
//   as present.  
const double playerSmoothing = 1000.0; // ms. Half-time to new reading (smoother).
double isPlayerThreshold[N_ARCH] = {45, 45, 45}; // compare: meanProxAvg

// See decideCoordination()
// We require that a pair of players have difference in CV (ratio of SD/Mean sensor readings) that is 
//    lower than a theshold (coordination) AND both players have a SD of sensor readings greater than
//    a threshold (activity) to score a coordinated pair.
const double coordSmoothing = 1000.0; // ms. Half-time to new reading (smoother).
double areCoordinatedThreshold[N_ARCH] = {25, 25, 25}; // compare: ratioCVAvg
double areActiveThreshold[N_ARCH] = {20, 20, 20}; // compare: meanProxSD

// too hard? 
//double areCoordinatedThreshold[N_ARCH] = {10, 10, 10}; // compare: ratioCVAvg
//double areActiveThreshold[N_ARCH] = {25, 25, 25}; // compare: meanProxSD

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
  Serial.flush();
  
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
        char w = Serial.read();
        if( w=='l' ) sCT.tr.left = sCT.tr.left == TRIGGER_ON ? TRIGGER_OFF : TRIGGER_ON;
        if( w=='r' ) sCT.tr.right = sCT.tr.right == TRIGGER_ON ? TRIGGER_OFF : TRIGGER_ON;
       
        comms.publish(&sCT.tr);
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
    case 'p': {
        int foo = Serial.parseInt();
        isPlayerThreshold[0] = constrain(foo, 0, 1000);
        isPlayerThreshold[1] = isPlayerThreshold[2] = isPlayerThreshold[0];
        Serial << F("Setting player threshold=") << isPlayerThreshold[1] << endl;
        stateLock = false;
        break;
      }
    case 'a': {
        int foo = Serial.parseInt();
        areActiveThreshold[0] = constrain(foo, 0, 1000);
        areActiveThreshold[1] = areActiveThreshold[2] = areActiveThreshold[0];
        Serial << F("Setting activity threshold=") << areActiveThreshold[1] << endl;
        stateLock = false;
        break;
      }    
    case 'c': {
        int foo = Serial.parseInt();
        areCoordinatedThreshold[0] = constrain(foo, 0, 1000);
        areCoordinatedThreshold[1] = areCoordinatedThreshold[2] = areCoordinatedThreshold[0];
        Serial << F("Setting coordination threshold=") << areCoordinatedThreshold[1] << endl;
        stateLock = false;
        break;
      }
    case '\n': break;
    case '\r': break;
    default:
      Serial << F("(s)tartup, (l)onely, (o)hai, good(n)uf, (g)oodjob, (w)inning, (f)anfare, (r)eboot.") << endl;
      Serial << F("un(L)ock game state. (T)rigger (l)eft or (r)ight cannon.") << endl;
      Serial << F("(p)(0-1000) set player threshold.") << endl;
      Serial << F("(a)(0-1000) set activity threshold.") << endl;
      Serial << F("(c)(0-1000) set coord threshold.") << endl;
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
//const byte pc_Goodjob = 3;
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
void fanfareEnter() {
  genericStateEnter(FANFARE);
}
void fanfareUpdate() {
  if ( ! transitionLockoutExpired() ) return;

  stateMachine.transitionTo(Lonely);
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
  const double alpha = calculateSmoothing( DISTANCE_SAMPLING_RATE, playerSmoothing );
  performSmoothing(meanProxAvg[arch], meanProxVar[arch], m, alpha);

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

  // record CV difference
  double rCV = abs(meanProxCV[arch1]-meanProxCV[arch2]);
  
  // smooth it
  const double alpha = calculateSmoothing( DISTANCE_SAMPLING_RATE, coordSmoothing );
  performSmoothing(ratioCVAvg[pair], ratioCVVar[pair], rCV, alpha);
  
  // check coordination
  boolean areCoordinated = false;

  // if there are players in these arches, then they may be coordinated
  if ( sC.isPlayer[arch1] &&  sC.isPlayer[arch2] ) {
//    if( meanProxSD[arch1] > areCoordinatedThreshold[arch1] &&
//        meanProxSD[arch2] > areCoordinatedThreshold[arch2] ) 
//    meanProxCV[arch1] > areCoordinatedThreshold[arch1] &&
//    meanProxCV[arch2] > areCoordinatedThreshold[arch2] ) 
    if( ratioCVAvg[pair] < areCoordinatedThreshold[pair] &&
        meanProxSD[arch1] > areActiveThreshold[arch1] && 
        meanProxSD[arch2] > areActiveThreshold[arch2] )
    
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
    Serial << " pair0=" << ratioCVAvg[0];
    Serial << " pair1=" << ratioCVAvg[1];
    Serial << " pair2=" << ratioCVAvg[2];
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
  mean = mean + incr; // also = (1-alpha)*mean + alpha*x
  var = (1.0-alpha)*(var + diff*incr); // also = (1-alpha)*var + (alpha-alpha^2)*diff^2
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

byte playerCoordinationGE(byte threshold) {
  return(
    sC.isPlayer[0] + sC.isPlayer[1] + sC.isPlayer[2] +
    sC.areCoordinated[0] + sC.areCoordinated[1] + sC.areCoordinated[2]
    
    >= threshold
  );
}

// don't allow wild oscillations in state transitions
uint32_t nextTransitionAllowedAt;
uint32_t transitionLockoutTime;
void resetTransitionLockout() {
  nextTransitionAllowedAt = millis() + transitionLockoutTime;
}
double deltaTransition() {
  return( ( (double)millis() - (double)nextTransitionAllowedAt )/1000.0 );
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
  static Metro updateInterval(1000UL);
  if( !updateInterval.check() ) return;

  // show status; about 40 ms to show this update.
  // ~8ms with 800Khz
  display.clearDisplay();
  
  // shift down
  display.setCursor(0,FH);
  
  switch (sC.state) {
    case STARTUP: display.print("STARTUP"); break;  //  all roles start here
    case LONELY:  display.print("LONELY"); break;   // 0 players
    case OHAI:    display.print("OHAI"); break;   // 1 players
    case GOODNUF: display.print("GOODNUF"); break;  // 2 players
    case GOODJOB: display.print("GOODJOB"); break;  // 3 players or 2 players coordinated
    case WINNING: display.print("WINNING"); break;  // 3 players and 2 players coordinated
    case FANFARE: display.print("FANFARE"); break;  // 3 players and 3 players coordinated
    case REBOOT:  display.print("REBOOT"); break;   //  trigger to reboot
  }
  String p = String("    ") + String(deltaTransition(), 1);
  display.println(p);

  p = String("C") + String(playerCoordination(), 10) + ":";
  for(byte i=0; i<N_ARCH; i++)  p += String(sC.isPlayer[i] ? "P" : "p");
  for(byte i=0; i<N_ARCH; i++)  p += String(sC.areCoordinated[i] ? "C" : "c");
  display.println(p);

  for(byte i=0; i<N_ARCH; i++) {
    p = String(i,10) + " ";
    p += "P" + String(meanProxAvg[i], 0);
    p += " A" + String(meanProxSD[i], 0); 
    p += " C" + String(ratioCVAvg[i], 0); 
    display.println(p);
  }

  display.print("Tr:");
  if( sCT.tr.left ) display.print(" L");
  else              display.print(" l");
  if( sCT.tr.right ) display.println(" R");
  else               display.println(" r");

  // push
  yield(); comms.update(); 
  display.display();
  yield(); comms.update(); 

  updateInterval.reset();
}


// total coordination
byte playerCoordination() {
  return(
    sC.isPlayer[0] + sC.isPlayer[1] + sC.isPlayer[2] +
    sC.areCoordinated[0] + sC.areCoordinated[1] + sC.areCoordinated[2]
  );
}




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

// spammy?
#define SHOW_FREQ_DEBUG false
#define SHOW_COORD_DEBUG false
#define DUMP_DISTANCE_DATA false

// ##################
// Master settings:

// don't allow wild oscillations in state transitions. See resetTransitionLockout()
const uint32_t transitionLockoutTime = 6000UL;

// fold-increase from basal sensor readings for "yes, have a player".
// See decidePlayerState()
const double isPlayerThreshold[N_ARCH] = {1.25, 1.25, 1.25};

// threshold on coordination for "yes, they're coordinated".  See decideCoordinationState()
const double areCoordinatedThreshold[N_ARCH] = {0.9, 0.9, 0.9};
// want the coordination to be smoothed;  See decideCoordination()
const byte smoothing = 10;

// if we "win", how long do we do stuff for?
const uint32_t fanfareDuration = 10UL * 1000UL; // seconds

// ##################

// My role and arch number
NyctRole myRole = Coordinator; // see Nyctinasty_Comms.h; set N_ROLES to pull from EEPROM

// wire it up
// also used
// D4, GPIO2, BUILTIN_LED

// LED handling

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

void setup() {
  // for local output
  Serial.begin(115200);

  Serial << endl << endl << endl << F("Startup begins.") << endl;

  // configure comms
  comms.begin(myRole);
  myRole = comms.getRole();

  // subscribe
  for ( byte i = 0; i < N_ARCH; i++ ) {
    comms.subscribe(&sAD[i].dist, &sAD[i].hasUpdate, i);
    comms.subscribe(&sAF[i].freq, &sAF[i].hasUpdate, i);
  }

  // for random numbers
  randomSeed(analogRead(0));

  Serial << F("Startup complete.") << endl;
}

// check for serial input
void lookForSerialCommands() {
  if ( ! Serial.available() ) return;

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
      stateLock = ! stateLock; 
      Serial << "State lock=" << stateLock << endl; 
      break;
    case 'P': {
      byte player = constrain(Serial.parseInt(), 0, 2);
      sC.isPlayer[player] = !sC.isPlayer[player];
      sendSettings();
      break;
    }
    case 'C': {
      byte pair = constrain(Serial.parseInt(), 0, 2);
      sC.areCoordinated[pair] = !sC.areCoordinated[pair];
      // must have players if coordinated
      if( sC.areCoordinated[0] ) sC.isPlayer[0] = sC.isPlayer[1] = true;
      if( sC.areCoordinated[1] ) sC.isPlayer[1] = sC.isPlayer[2] = true;
      if( sC.areCoordinated[2] ) sC.isPlayer[2] = sC.isPlayer[0] = true;
      sendSettings();
      break;
    }    
    case '\n': break;
    case '\r': break;
    default:
      Serial << F("(s)tartup, (l)onely, (o)hai, good(n)uf, (g)oodjob, (w)inning, (f)anfare, (r)eboot.") << endl;
      Serial << F("(L)ock and unlock states.") << endl;
      Serial << F("(P)(0-2) toggle player state: A0, A1, A2.") << endl;
      Serial << F("(C)(0-2) toggle player state: A0:A1, A1:A2, A2:A0.") << endl;
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
  if( stateLock ) {
    resetTransitionLockout();
    stateMachine.update();
    return;
  }

  // otherwise, check stuff
  if ( sAF[0].hasUpdate ) {
    decidePlayerState(0);
    decideCoordination(0); // 0,1
    decideCoordination(2); // 2,0

    sAF[0].hasUpdate = false;
  }
  if ( sAF[1].hasUpdate ) {
    decidePlayerState(1);
    decideCoordination(0); // 0,1
    decideCoordination(1); // 1,2

    sAF[1].hasUpdate = false;
  }
  if ( sAF[2].hasUpdate ) {
    decidePlayerState(2);
    decideCoordination(2); // 2,0
    decideCoordination(0); // 0,1

    sAF[2].hasUpdate = false;
  }

  // state machine
  stateMachine.update();
}


/*
   Lonely    0 players
   Ohai      1 players
   Goodnuf   2 players
   Goodjob   3 players or 2 players coordinated
   Winning   3 players and 3 players coordinated
*/

// I enumerated these, as I can use symmetric ">=" and "<" below to get the logic right for transition counts
const byte playerCount_Ohai = 1;
const byte playerCount_Goodnuf = 2;
const byte playerCount_Goodjob = 3;
const byte playerCount_Winning = 3;
const byte playerCount_Fanfare = 3;

const byte coordCount_Ohai = 1;
const byte coordCount_Goodnuf = 1;
const byte coordCount_Goodjob = 1;
const byte coordCount_Winning = 2;
const byte coordCount_Fanfare = 3;

// STARTUP state
void startupEnter() {
  genericStateEnter(STARTUP);
}
void startupUpdate() {
  if ( transitionLockoutExpired() ) stateMachine.transitionTo(Lonely);
}

// LONELY state
void lonelyEnter() {
  genericStateEnter(LONELY);
}
void lonelyUpdate() {
  if ( ! transitionLockoutExpired() ) return;

  // no demotion possible

  // promote
  if ( ( totalPlayersGE(playerCount_Ohai) || coordPlayersGE(coordCount_Ohai) ) ) stateMachine.transitionTo(Ohai);
}

// OHAI state
void ohaiEnter() {
  genericStateEnter(OHAI);
}
void ohaiUpdate() {
  if ( ! transitionLockoutExpired() ) return;

  // demote
  if ( ! ( totalPlayersGE(playerCount_Ohai) || coordPlayersGE(coordCount_Ohai) ) ) stateMachine.transitionTo(Lonely);

  // promote
  if ( ( totalPlayersGE(playerCount_Goodnuf) || coordPlayersGE(coordCount_Goodnuf) ) ) stateMachine.transitionTo(Goodnuf);
}

// GOODNUF state
void goodnufEnter() {
  genericStateEnter(GOODNUF);
}
void goodnufUpdate() {
  if ( ! transitionLockoutExpired() ) return;

  // demote
  if ( ! ( totalPlayersGE(playerCount_Goodnuf) || coordPlayersGE(coordCount_Goodnuf) ) ) stateMachine.transitionTo(Ohai);

  // promote
  if ( ( totalPlayersGE(playerCount_Goodjob) || coordPlayersGE(coordCount_Goodjob) ) ) stateMachine.transitionTo(Goodjob);
}

// GOODJOB state
void goodjobEnter() {
  genericStateEnter(GOODJOB);
}
void goodjobUpdate() {
  if ( ! transitionLockoutExpired() ) return;

  // demote
  if ( ! ( totalPlayersGE(playerCount_Goodjob) || coordPlayersGE(coordCount_Goodjob) ) ) stateMachine.transitionTo(Goodnuf);

  // promote
  if ( ( totalPlayersGE(playerCount_Winning) || coordPlayersGE(coordCount_Winning) ) ) stateMachine.transitionTo(Winning);
}

// WINNING state
void winningEnter() {
  genericStateEnter(WINNING);
}
void winningUpdate() {
  if ( ! transitionLockoutExpired() ) return;

  // demote
  if ( ! ( totalPlayersGE(playerCount_Winning) || coordPlayersGE(coordCount_Winning) ) ) stateMachine.transitionTo(Goodjob);

  // promote
  if ( ( totalPlayersGE(playerCount_Fanfare) || coordPlayersGE(coordCount_Fanfare) ) ) stateMachine.transitionTo(Fanfare);
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
double detectPlayer(byte arch) {
  // compute mean
  double meanAvgPower = 0;
  for ( byte s = 0; s < N_SENSOR; s++ ) meanAvgPower += sAF[arch].freq.avgPower[s];
  meanAvgPower /= (double)N_SENSOR;

  return ( meanAvgPower );
}

// watch the player state and also count the number of times in that same state (persistence)
void decidePlayerState(byte arch) {
  static double smoothedPower[N_ARCH] = {6528, 6528, 6528};
  // should be 6528.
  const double s = 10;

  // current, immediate value
  double power = detectPlayer(arch);

  boolean state = detectPlayer(arch) > smoothedPower[arch] * isPlayerThreshold[arch];
  if ( state != sC.isPlayer[arch] ) {
    sC.isPlayer[arch] = state;
    Serial << "decidePlayerState.  isPlayer? A" << arch << "=" << state << endl;
    sendSettings();
  }

  if ( !sC.isPlayer[arch] && power < smoothedPower[arch] ) {
    smoothedPower[arch] = (smoothedPower[arch] * (s - 1.0) + power) / s;
    //    Serial << "decidePlayerState.  smoothed power? A0=" << smoothedPower[0];
    //    Serial << " A1=" << smoothedPower[1];
    //    Serial << " A2=" << smoothedPower[2] << endl;
  }
}

// algorithm for coordination detection
double isCoordinated(byte arch1, byte arch2) {

  // if there aren't players in these arches, then they can't be coordinated.
  if ( sC.isPlayer[arch1] == false || sC.isPlayer[arch2] == false ) return (0.0);

  // I didn't say it was an awesome algorithm; watch this space.
  return ( 1.0 );
}

void decideCoordination(byte pair) {
  // current, immediate value
  double coord = 0;
  switch (pair) {
    case 0: coord = isCoordinated(0, 1); break;
    case 1: coord = isCoordinated(1, 2); break;
    case 2: coord = isCoordinated(2, 0); break;
  }

  // want coordination to be smoothed
  static double smoothCoord[N_ARCH] = {0.0};
  smoothCoord[pair] = ( smoothCoord[pair] * ((double)smoothing - 1) + coord ) / ((double)smoothing);

  //  if( smoothCoord[pair] > 0 ) {
  //    Serial << "decideCoordination. P" << pair << "=" << smoothCoord[pair] << endl;
  //  }

  boolean areCoordinated = smoothCoord[pair] > areCoordinatedThreshold[pair];
  if ( areCoordinated != sC.areCoordinated[pair] ) {
    Serial << "decideCoordination.  areCoordinated? P" << pair << "=" << areCoordinated << endl;
    sC.areCoordinated[pair] = areCoordinated;
    sendSettings();
  }

}

byte totalPlayersGE(byte th) {
  return ( (sC.isPlayer[0] + sC.isPlayer[1] + sC.isPlayer[2]) >= th );
}
byte coordPlayersGE(byte th) {
  return ( (sC.areCoordinated[0] + sC.areCoordinated[1] + sC.areCoordinated[2]) >= th );
}

// don't allow wild oscillations in state transitions
uint32_t nextTransitionAllowedAt;
void resetTransitionLockout() {
  nextTransitionAllowedAt = millis() + transitionLockoutTime;
}
boolean transitionLockoutExpired() {
  return ( millis() > nextTransitionAllowedAt );
}

void genericStateEnter(systemState state) {
  resetTransitionLockout();

  Serial << F("State change.");
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




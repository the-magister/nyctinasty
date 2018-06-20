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
#define SHOW_SERIAL_DEBUG true

// my role and arch number
NyctRole myRole = Coordinator; // see Nyctinasty_Comms.h; set N_ROLES to pull from EEPROM

// wire it up
// also used
// D4, GPIO2, BUILTIN_LED

// LED handling

// comms
NyctComms comms;

// define a state for every systemState
void startup(); State Startup = State(startup);
void offline(); State Offline = State(offline);
void online(); State Online = State(online);
void slaved(); State Slaved = State(slaved);
void reboot() { comms.reboot(); }; State Reboot = State(reboot);
void reprogram() { comms.reprogram("Sepal_Arch.ino.bin"); }; State Reprogram = State(reprogram);
FSM stateMachine = FSM(Startup); // initialize state machine

// incoming message storage and flag for update
struct sC_t {
  boolean hasUpdate = false;
  SystemCommand settings;
} sC;

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

struct pC_t {
  boolean hasUpdate = false;
  WaterWorks water;
} pC;

// don't allow pump state switching too rapidly.
const uint32_t pumpChangeInterval = 1000UL;
byte targetPumpLevel = 0;

/*
Level PrimePumps  BoostPumps
0     0           0
1     1           0
2     1           1
3     2           1
4     2           2
*/

/*
 * clockwise: Arch0, Cannon, Arch1, steps, Arch2, steps
 */

void setup() {
  // for local output
  Serial.begin(115200);

  Serial << endl << endl << endl << F("Startup begins.") << endl;

  // configure comms
  comms.begin(myRole);
  myRole = comms.getRole();
  
  // subscribe
  comms.subscribe(&sC.settings, &sC.hasUpdate);
  for ( byte i = 0; i < N_ARCH; i++ ) {
    comms.subscribe(&sAF[i].freq, &sAF[i].hasUpdate, i);
  }

  // for random numbers
  randomSeed(analogRead(0));

  Serial << F("Startup complete.") << endl;
  Serial << F("[0-4] to set pump level.  'r' to toggle route.") << endl;
}

void loop() {
  // comms handling
  comms.update();

  // check for settings update
  if ( sC.hasUpdate ) {
    switchState(sC.settings.state);
    sC.hasUpdate = false;
  }
/*
  // check for serial input
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'r': pC.water.route = (pC.water.route == CANNON) ? FOUNTAIN : CANNON; pC.hasUpdate = true; break;
      case '0': targetPumpLevel = 0; break;
      case '1': targetPumpLevel = 1; break;
      case '2': targetPumpLevel = 2; break;
      case '3': targetPumpLevel = 3; break;
      case '4': targetPumpLevel = 4; break;
      case '\n': break;
      case '\r': break;
      default:
          Serial << F("[0-4] to set pump level.  'r' to toggle route.") << endl;

        break;
    }
  }
*/
  static String msg = "NOP";
  if (Serial.available()) {
    delay(100);
    msg = Serial.readString();
  }
  
  const char sep = ',';
  // loop across each arch's information
  if( sAF[0].hasUpdate && sAF[1].hasUpdate && sAF[2].hasUpdate ){
    uint32_t now = millis();

    /*
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
    }*/
    Serial << msg << sep << now;
    concordanceByPower();
    Serial << endl;
    
    sAF[0].hasUpdate = sAF[1].hasUpdate = sAF[2].hasUpdate = false;
  }
  // do stuff
  stateMachine.update();
}

void switchState(systemState state) {
  Serial << F("State.  Changing to ") << state << endl;
  switch ( state ) {
    case STARTUP: stateMachine.transitionTo(Startup); break;
    case OFFLINE: stateMachine.transitionTo(Offline); break;
    case ONLINE: stateMachine.transitionTo(Online); break;
    case SLAVED: stateMachine.transitionTo(Slaved); break;
    case REBOOT: stateMachine.transitionTo(Reboot); break;
    case REPROGRAM: stateMachine.transitionTo(Reprogram); break;
    default:
      Serial << F("ERROR!  unknown state.") << endl;
  }
}

void startup() {
   // after 5 seconds, transition to Offline, but we could easily get directed to Online before that.
  static Metro startupTimeout(5000UL);
  if( startupTimeout.check() ) stateMachine.transitionTo(Offline);
}
 
void offline() {
  if( comms.isConnected() ) {
    Serial << F("GOOD.  online!") << endl;
    stateMachine.transitionTo(Online);
  } else {
    normal(false);
  }
}

void online() {
  if( ! comms.isConnected() ) {
    Serial << F("WARNING.  offline!") << endl;
    stateMachine.transitionTo(Offline);
  } else {
    normal(true);
  }

}

void normal(boolean isOnline) {

  // queue up changes for online
  if( ! isOnline ) return;
  
  static Metro pumpChange(pumpChangeInterval);
  if( getPumpLevel() != targetPumpLevel ) {
    if( pumpChange.check() ) {
      
      changePumpLevel(); // update
      comms.publish(&pC.water); // push it
      
      pumpChange.reset();
    }
  }
}

double correlation(double (&x)[N_FREQ_BINS], double (&y)[N_FREQ_BINS]) {
  
  // sample size
  const double n = N_FREQ_BINS;

  // loop over bins
  double sumXY, sumX, sumY, sumX2, sumY2 = 0;
  for( byte b=0; b<N_FREQ_BINS; b++ ) {
    sumXY += x[b]*y[b];
    sumX += x[b];
    sumY += y[b];
    sumX2 += x[b]*x[b];
    sumY2 += y[b]*y[b];
  }

  double n1 = n * sumXY;
  double n2 = sumX * sumY;
  double d1 = sqrt(n*sumX2 - sumX*sumX);
  double d2 = sqrt(n*sumY2 - sumY*sumY);
  return( (n1-n2)/d1/d2 );
}

void concordanceByPower() {

  // sum the power information across each sensor for each arch
  double power[N_ARCH][N_FREQ_BINS] = {{0}};
  for( byte a=0; a<N_ARCH; a++ ) {
    for( byte b=0; b<N_FREQ_BINS; b++ ) {
      for( byte s=0; s<N_SENSOR; s++ ) {
        power[a][b] += sAF[a].freq.power[s][b];
      }
      power[a][b] = log(power[a][b]); // weight the high frequency stuff more
    }
  }
  
  // compute
  double corr01 = correlation(power[0], power[1]);
  double corr12 = correlation(power[1], power[2]);
  double corr20 = correlation(power[2], power[0]);

  char sep = ',';
  Serial << sep << corr01 << sep << corr12 << sep << corr20;
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

byte getPumpLevel() {
  byte ret = 0;
  for( byte p=0; p<2; p++ ) ret += (pC.water.primePump[p] == ON) + (pC.water.boostPump[p] == ON);
  return( ret );
}

void changePumpLevel() {
  byte currentPumpLevel = getPumpLevel();
  
  Serial << "Pump level: " << currentPumpLevel << " -> " << targetPumpLevel << endl;
  
  if( currentPumpLevel < targetPumpLevel ) {
    switch( currentPumpLevel ) {
      case 0: addPrimePump(); break;
      case 1: addBoostPump(); break;
      case 2: addPrimePump(); break;
      case 3: addBoostPump(); break;
    }
  } else {
    switch( currentPumpLevel ) {
      case 1: subPrimePump(); break;
      case 2: subBoostPump(); break;
      case 3: subPrimePump(); break;
      case 4: subBoostPump(); break;
    }
  }

  Serial << "\tRoute: ";
  Serial << (pC.water.route == FOUNTAIN ? "Fountain" : "Cannon") << endl;
  Serial << "\tPrime 1: ";
  Serial << (pC.water.primePump[0] == ON ? "ON" : "off") << endl;
  Serial << "\tPrime 2: ";
  Serial << (pC.water.primePump[1] == ON ? "ON" : "off") << endl;
  Serial << "\tBoost 1: ";
  Serial << (pC.water.boostPump[0] == ON ? "ON" : "off") << endl;
  Serial << "\tBoost 2: ";
  Serial << (pC.water.boostPump[1] == ON ? "ON" : "off") << endl;

}

// start with random value
static byte nextBoostPump = random(2); 
// start with random value
static byte nextPrimePump = random(2); 

void addBoostPump() {
  // odd case but
  if( pC.water.boostPump[0] == ON && pC.water.boostPump[1] == ON ) return;

  // set next in line on
  pC.water.boostPump[nextBoostPump] = ON;

  // roll to next
  nextBoostPump = !nextBoostPump;
}
void addPrimePump() {
  // odd case but
  if( pC.water.primePump[0] == ON && pC.water.primePump[1] == ON ) return;

  // set next in line on
  pC.water.primePump[nextPrimePump] = ON;

  // roll to next
  nextPrimePump = !nextPrimePump;
}

void subBoostPump() {
  // odd case but
  if( pC.water.boostPump[0] == OFF && pC.water.boostPump[1] == OFF ) return;

  // might need to toggle the same one
  if( (pC.water.boostPump[0] == ON) + (pC.water.boostPump[1] == ON) == 1 ) nextBoostPump != nextBoostPump;
  
  // set next in line on
  pC.water.boostPump[nextBoostPump] = OFF;

  // roll to next
  nextBoostPump = !nextBoostPump;
}
void subPrimePump() {
  // odd case but
  if( pC.water.primePump[0] == OFF && pC.water.primePump[1] == OFF ) return;

  // might need to toggle the same one
  if( (pC.water.primePump[0] == ON) + (pC.water.primePump[1] == ON) == 1 ) nextPrimePump != nextPrimePump;
  
  // set next in line on
  pC.water.primePump[nextPrimePump] = OFF;

  // roll to next
  nextPrimePump = !nextPrimePump;
}

void slaved() {
  // NOP, currently.  Will look for lighting directions, either through UDP (direct) or MQTT (procedural).
}



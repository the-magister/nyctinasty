#include <RFM69.h>
#include <SPI.h>

#define RF69_IRQ_PIN  D2
#define RF69_IRQ_NUM  digitalPinToInterrupt(RF69_IRQ_PIN)
#define RF69_SPI_CS   D8
#define RF69_RST_PIN  D1

#define LED           BUILTIN_LED   // GPIO02/D4, onboard blinky for Adafruit Huzzah

#define NODEID      99
#define NETWORKID   188
#define GATEWAYID   1
#define FREQUENCY   RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define LED         BUILTIN_LED
#define SERIAL_BAUD 115200
#define ACK_TIME    30  // # of ms to wait for an ack

int TRANSMITPERIOD = 1000; //transmit a packet to gateway so often (in ms)
byte sendSize=0;
boolean requestACK = false;
RFM69 radio;

typedef struct {		
  uint32_t           nodeId; //store this nodeId
  uint32_t uptime; //uptime in ms
  uint32_t         temp;   //temperature maybe?
} Payload;
Payload theData;

void radioReset() {
  // Hard Reset the RFM module
  pinMode(RF69_RST_PIN, OUTPUT);
  digitalWrite(RF69_RST_PIN, HIGH);
  delay(100);
  digitalWrite(RF69_RST_PIN, LOW);
  delay(100);  
}

void radioInitialize() {
  radio = RFM69(RF69_SPI_CS, RF69_IRQ_PIN, true, RF69_IRQ_NUM);

  radioReset();
  
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  radio.setHighPower(); //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  radioInitialize();
  
  char buff[50];
  sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);

  Serial.print("Size of Payload: ");
  Serial.println(sizeof(Payload));
}

long lastPeriod = -1;
void loop() {
  //process any serial input
  if (Serial.available() > 0)
  {
    char input = Serial.read();
    if (input >= 48 && input <= 57) //[0,9]
    {
      TRANSMITPERIOD = 100 * (input-48);
      if (TRANSMITPERIOD == 0) TRANSMITPERIOD = 1000;
      Serial.print("\nChanging delay to ");
      Serial.print(TRANSMITPERIOD);
      Serial.println("ms\n");
    }
    
    if (input == 'r') //d=dump register values
      radio.readAllRegs();
  }

  //check for any received packets
  if (radio.receiveDone())
  {
    Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print((char)radio.DATA[i]);
    Serial.print("   [RX_RSSI:");Serial.print(radio.readRSSI());Serial.print("]");

    if (radio.ACKRequested())
    {
      radio.sendACK();
      Serial.print(" - ACK sent");
      delay(10);
    }
    Blink(LED,5);
    Serial.println();
  }
  
  int currPeriod = millis()/TRANSMITPERIOD;
  if (currPeriod != lastPeriod)
  {
    //fill in the struct with new values
    theData.nodeId = NODEID;
    theData.uptime = millis();
    theData.temp = random(100,1000); //it's hot!
    
    Serial.print("Sending struct (");
    Serial.print(sizeof(theData));
    Serial.print(" bytes) ... ");
    if (radio.sendWithRetry(GATEWAYID, (const void*)(&theData), sizeof(theData)))
      Serial.print(" ok!");
    else Serial.print(" nothing...");
    Serial.println();
    Blink(LED,3);
    lastPeriod=currPeriod;
  }
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

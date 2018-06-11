/*
    Serial control:   bit for each sepal, and the number of second you want to mist...
    
    "bool,bool,bool,seconds"  example: 1,0,1,5;
    

    Direct line control:  rising edge of "GO" pin starts the process, and the sepal enable pins can change at any time
    Max fire time of 5 minutes is built in, no matter how long the "GO" line is held.
*/

#define ARRAYSIZE(x) (sizeof(x)/sizeof(x[0]))

class Actor{
  public:
  virtual void Update(long absTimeMs, long deltaTimeMs) = 0;
};

class Relay : public Actor{
public:
  Relay(int pin){
    Pin = pin;
    Open = false;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }

  void Update(long absTimeMs, long deltaTimeMs)
  {
    digitalWrite(Pin, Open ? LOW : HIGH);
  }

  void SetState(bool open)
  {
    Open = open;
  }
  
private:
  int Pin;
  bool Open;
};

class PressureSensor : public Actor{
public:
  PressureSensor (int pin, int timeAverageMs, int sensorRange)
  {
    Pin = pin;
    pinMode(Pin, INPUT);
    Readings[0] = analogRead(Pin);
    for (int i = 1; i < ARRAYSIZE(Readings); i++)
    {
      Readings[i] = Readings[0];
    }
    LastReadingMs = 0;
    ReadIntervalMs = max(1, timeAverageMs / ARRAYSIZE(Readings));
    SensorRange = sensorRange;
  }
  
  void Update(long absTimeMs, long deltaTimeMs)
  {
    if (absTimeMs - LastReadingMs > ReadIntervalMs)
    {
      LastReadingIndex++;
      LastReadingIndex%=10;
      Readings[LastReadingIndex] = analogRead(Pin);
      LastReadingMs = absTimeMs;
    }
  } 

  int GetRunningAverage()
  {
    int total = 0;
    for (int i = 0; i < ARRAYSIZE(Readings); i++) total += Readings[i];
    return total / ARRAYSIZE(Readings);
  }

  int GetLatestValue()
  {
    return Readings[LastReadingIndex];
  }

  int GetSensorDeviation()
  {
    long average = GetRunningAverage();
    long deviation = 0;
    for (int i = 0; i < ARRAYSIZE(Readings); i++) 
    {
      long delta = abs(Readings[i] - average);
      deviation += delta*delta;
    }
    return (deviation / ARRAYSIZE(Readings)) / average;
  }

  int GetRunningAveragePressure()
  {
    return ((long)GetRunningAverage()) * SensorRange / 1023;
  }

// for finding pin mapping on non-standard devices.
//  void NextPin()
//  {
//    Pin++;
//    pinMode(Pin, INPUT);
//  }

  int SensorRange;
  int ReadIntervalMs;
  long LastReadingMs;
  char LastReadingIndex;
  int Pin;
  int Readings[10];
};

class FlowSensorDevice : public Actor{
public:
  FlowSensorDevice (int pin, int timeAverageMs)
  {
    Pin = pin;
    pinMode(Pin, INPUT);
    Readings[0] = analogRead(Pin);
    for (int i = 1; i < ARRAYSIZE(Readings); i++)
    {
      Readings[i] = Readings[0];
    }
    LastReadingMs = 0;
    ReadIntervalMs = max(1, timeAverageMs / ARRAYSIZE(Readings));
  }
  
  void Update(long absTimeMs, long deltaTimeMs)
  {
    if (absTimeMs - LastReadingMs > ReadIntervalMs)
    {
      LastReadingIndex++;
      LastReadingIndex%=10;
      Readings[LastReadingIndex] = analogRead(Pin);
      LastReadingMs = absTimeMs;
    }
  } 

  int GetRunningAverage()
  {
    int total = 0;
    for (int i = 0; i < ARRAYSIZE(Readings); i++) total += Readings[i];
    return total / ARRAYSIZE(Readings);
  }

  int GetLatestValue()
  {
    return Readings[LastReadingIndex];
  }

  int ReadIntervalMs;
  long LastReadingMs;
  char LastReadingIndex;
  int Pin;
  int Readings[10];
};

class InputControl : public Actor {
public:
  bool SepalEnable[3];
  bool GoControl;  
};

class PinInputControl : public InputControl
{
public:
  PinInputControl (int sepalEnableControlPins[3], int goControlPin)
  {
    for (int i = 0; i < 3; i++) 
    {
      SepalEnableControlPins[i] = sepalEnableControlPins[i];
      pinMode(SepalEnableControlPins[i], INPUT);
    }
    GoControlPin = goControlPin;
    pinMode(GoControlPin, INPUT);
    GoControlPriorReading = digitalRead(GoControlPin);
    GoControl = false;
  }
  
  void Update(long absTimeMs, long deltaTimeMs)
  {
    char message[64];
    
    // first, disable if a prior "Go" is expired.
    if (GoControl && absTimeMs >= GoTimeout)
    {
      sprintf(message, "direct EXPIRE");
      Serial.println(message);  
      GoControl = false;
    }

    for (int i = 0; i < 3; i++) 
    {
      SepalEnable[i] = digitalRead(SepalEnableControlPins[i]);
    }

    bool manualGoControl = digitalRead(GoControlPin);
    if (GoControlPriorReading == false && manualGoControl == true)
    {
      // rising edge of manual control pin, that's the only time we enable      
      sprintf(message, "direct GO (%d->%d): %d, %d, %d", GoControlPriorReading, manualGoControl, SepalEnable[0], SepalEnable[1], SepalEnable[2]);
      Serial.println(message);  
      GoTimeout = absTimeMs + (5 * 60 * (long)1000); // 5 minute max duration
      GoControl = true;
    }
    else if (GoControlPriorReading == true && manualGoControl == false)
    {
      // falling edge of manual control pin, typical "done condition"
      sprintf(message, "direct STOP (%d->%d)", GoControlPriorReading, manualGoControl);
      Serial.println(message);  
      GoControl = false;
    }
    GoControlPriorReading = manualGoControl;
  } 

private:

  int SepalEnableControlPins[3];
  int GoControlPin;
  bool GoControlPriorReading;
  long GoTimeout;
};

class SerialInputControl : public InputControl
{
public:
  SerialInputControl ()
  {
    GoControl = false;
  }
  
  void Update(long absTimeMs, long deltaTimeMs)
  {
    char message[100];
    // first, disable if a prior "Go" is expired.
    if (GoControl && (absTimeMs > GoTimeout))
    {
      sprintf(message, "serial EXPIRE, %ld > %ld", absTimeMs, GoTimeout);
      Serial.println(message);  
      GoControl = false;
    }

    if (Serial.available() > 0)
    {
      int sepal[3];
      sepal[0] = Serial.parseInt();
      sepal[1] = Serial.parseInt();
      sepal[2] = Serial.parseInt();
      int duration = Serial.parseInt();
      long timeout = absTimeMs + ( duration * 1000 );
      bool isValid = (Serial.read() == ';');
      sprintf(message, "serial GO: %d, %d, %d, %ds (%ld->%ld)  %s", 
        sepal[0], sepal[1], sepal[2], duration, absTimeMs, timeout, isValid? "valid" : "invalid");
      Serial.println(message);  
      if (isValid) 
      {
        SepalEnable[0] = sepal[0];
        SepalEnable[1] = sepal[1];
        SepalEnable[2] = sepal[2];
        GoTimeout = timeout;
        GoControl = true;
      }
    }
  } 

private:

  long GoTimeout;
};


class TestScript : public Actor{
  public:
    TestScript(Relay** relays, int relayCount, PressureSensor** sensors, int sensorCount)
    {
      Relays = relays;
      RelayCount = relayCount;
      Sensors = sensors;
      SensorCount = sensorCount;
      ActiveRelay = -1;
    }

  void Update(long absTimeMs, long deltaTimeMs)
  {
    char message[64];
    int newActive = ((absTimeMs / 4000) % RelayCount);
    if (newActive != ActiveRelay)
    {
      if (ActiveRelay != -1)
      {
        Relays[ActiveRelay]->SetState(false);        
      }
      Relays[newActive]->SetState(true);
      sprintf(message, "Sol %d -> %d (%ld)", ActiveRelay, newActive, absTimeMs);
      Serial.println(message);   
      ActiveRelay = newActive;
      for (int i = 0; i < SensorCount; i++)
      {
        sprintf(message, "Pressure: %d  %dpsi (pin %d) dev:%d", Sensors[i]->GetRunningAverage(), Sensors[i]->GetRunningAveragePressure(), Sensors[i]->Pin, Sensors[i]->GetSensorDeviation());
        Serial.println(message);  
      }
    }
  }
    
  private:
    int ActiveRelay;
    Relay** Relays;
    int RelayCount;
    PressureSensor** Sensors;
    int SensorCount;
};


#define S_LOW_PRESSURE_PUMP 0
#define S_HIGH_PRESSURE_PUMP 1
#define S_SEPAL_0 2
#define S_SEPAL_1 3
#define S_SEPAL_2 4

enum SystemState{
  SystemInit = 0,
  Off,
  LowPressurePumpEnable,
  HighPressurePumpEnable,
  Error,
};

const char* SystemStateName[] = {
  "SystemInit",
  "Off",
  "LowPressurePumpEnable",
  "HighPressurePumpEnable",
  "Error",
};

class PumpScript : public Actor{
  public:
    PumpScript(Relay** relays, int relayCount, PressureSensor** sensors, int sensorCount, FlowSensorDevice* flowSensor, InputControl* inputSystem)
    {
      Relays = relays;
      RelayCount = relayCount;
      Sensors = sensors;
      SensorCount = sensorCount;
      FlowSensor = flowSensor;
      InputSystem = inputSystem;
      State = SystemInit;
    }

  void Update(long absTimeMs, long deltaTimeMs)
  {
    char message[64];
    switch(State)
    {
      case SystemInit:
        // wait for sensors to collect\buffer data before continuing
        if (absTimeMs > 1000)
        {
          sprintf(message, "Startup Sensor[0] health: %d", Sensors[0]->GetSensorDeviation());
          Serial.println(message);
          sprintf(message, "Startup Sensor[1] health: %d", Sensors[1]->GetSensorDeviation());
          Serial.println(message);
          ChangeState(Off, absTimeMs, "startup delay complete");
        }
        for (int i = 0; i < RelayCount; i++)
        {
          Relays[i]->SetState(false);
        }
        
        break;
        
      case Off:
        for (int i = 0; i < RelayCount; i++)
        {
          Relays[i]->SetState(false);
        }

        // TODO: check health
        if (InputSystem->GoControl && absTimeMs - LastStateTransisionTime > 500)
        {
          ChangeState(LowPressurePumpEnable, absTimeMs, "go requested");
        }
        break;
        
      case LowPressurePumpEnable:
          // check back condition first...
          if (!InputSystem->GoControl)
          {
            for (int i = 0; i < RelayCount; i++) Relays[i]->SetState(false);
            ChangeState(Off, absTimeMs, "go cancelled");
          }
          else
          {
            Relays[S_LOW_PRESSURE_PUMP]->SetState(true);
            for (int i = 1; i < RelayCount; i++) Relays[i]->SetState(false);
            
            if (absTimeMs - LastStateTransisionTime > 500 && Sensors[0]->GetRunningAveragePressure() > 30)
            {
              ChangeState(HighPressurePumpEnable, absTimeMs, "all good, misting");
            }
            else if (absTimeMs - LastStateTransisionTime > 5000 && Sensors[0]->GetRunningAveragePressure() <= 30)
            {
              // if LP pump is on for 5 seconds, without target pressure, abort.
              // TODO: worry about the plumbing priming condition...
              ChangeState(Error, absTimeMs, "low pressure error");
            }
          }
        break;
        
      case HighPressurePumpEnable:
          // check back condition first...
          if (!InputSystem->GoControl)
          {
            for (int i = 0; i < RelayCount; i++) Relays[i]->SetState(false);
            ChangeState(Off, absTimeMs, "go cancelled");
          }
          else
          {
            Relays[S_LOW_PRESSURE_PUMP]->SetState(true);
            Relays[S_HIGH_PRESSURE_PUMP]->SetState(true);
            for (int i = 0; i < 3; i++) Relays[i+2]->SetState(InputSystem->SepalEnable[i]);

            if (absTimeMs - LastStateTransisionTime > 2000)
            {
              int enabledSepals = 0;
              for (int i = 0; i < 3; i++) 
              {
                if (InputSystem->SepalEnable[i]) enabledSepals++;
              }

              // if HP pump is on for 2 seconds, without target pressure, or if flow is too high, abort
              if (Sensors[1]->GetRunningAveragePressure() < 60)
              {
                ChangeState(Error, absTimeMs, "high pressure loss");
              }
              else if (FlowSensor->GetRunningAverage() > 40 * enabledSepals)
              {
                ChangeState(Error, absTimeMs, "flow limit exceeded");
              }
            }
          }
        break;
        
      case Error:
        break;

    }
  }
    
  private:
    void ChangeState(SystemState newState, long absTimeMs, const char* additionalInfo)
    {
      char message[100];
      sprintf(message, "state %s -> %s @ %ld %s", SystemStateName[State], SystemStateName[newState], absTimeMs, 
        additionalInfo ? additionalInfo : "");
      Serial.println(message); 
      
      State = newState;
      LastStateTransisionTime = absTimeMs;
    }
  
    SystemState State;
    Relay** Relays;
    int RelayCount;
    PressureSensor** Sensors;
    int SensorCount;
    FlowSensorDevice* FlowSensor;
    InputControl* InputSystem;
    long LastStateTransisionTime;
};

template <class T>
class List {
    T* Items;
    int CurrentCount;
    int MaxCount;
    
  public:
    List (int maxSize)
    {
      Items = (T*)malloc(sizeof(void*) * maxSize);
      CurrentCount = 0;
      MaxCount = maxSize;
    }

    void Add(T newItem)
    {
      if (CurrentCount < MaxCount)
      {
        Items[CurrentCount++] = newItem;
      }
    }

  T Get(int index)
  {
    if (index < CurrentCount)
    {
      return Items[index];
    }
  }

  int Size()
  {
    return CurrentCount;
  }

  ~List()
  {
    free(Items);
  }
    
    
};


Relay* Relays[5];
PressureSensor* Sensors[2];
FlowSensorDevice* FlowSensor;
InputControl* InputSystem;
List<Actor*> Actors(15);
long lastUpdateMs = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  int actor = 0;
  // actors are processed in sequence, placing inputs, script, and output objects in that order...
  Actors.Add(Sensors[0] = new PressureSensor(18, 500, 175));  // 18 is A0 on the mini board
  Actors.Add(Sensors[1] = new PressureSensor(19, 500, 175));  // 19 is A1 on the mini board
  Actors.Add(FlowSensor = new FlowSensorDevice(20, 500));     // unknown pin????
  //Actors.Add(InputSystem = new SerialInputControl());
  int directSepalControlPins[] = {41,42,43};
  Actors.Add(InputSystem = new PinInputControl(directSepalControlPins , 44));
  
  Actors.Add(new PumpScript(Relays, ARRAYSIZE(Relays), Sensors, ARRAYSIZE(Sensors), FlowSensor, InputSystem));
  //Actors.Add(new TestScript(Relays, ARRAYSIZE(Relays), Sensors, ARRAYSIZE(Sensors)));
  
  Actors.Add(Relays[0] = new Relay(2)); // LP pump
  Actors.Add(Relays[1] = new Relay(3)); // HP pump
  Actors.Add(Relays[2] = new Relay(4)); // Sepal 0
  Actors.Add(Relays[3] = new Relay(5)); // Sepal 1
  Actors.Add(Relays[4] = new Relay(6)); // Sepal 2
  delay(100); // When programming, it appears the unit is reset twice, this prevents the loop from starting on that initial reset...
}

// the loop function runs over and over again forever
void loop() {
  long absTimeMs = millis();
  long deltaTimeMs = absTimeMs - lastUpdateMs;
  for (int i = 0; i < Actors.Size(); i++)
  {
    Actors.Get(i)->Update(absTimeMs, deltaTimeMs);
  }
  lastUpdateMs = absTimeMs;
}

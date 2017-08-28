
import mqtt.*;

MQTTClient client;

public static final float MM = 0.001f;
public static final float IN = 0.0254f;

ArrayList<Target> targets = new ArrayList<Target>();
ArrayList<Sensor> sensors = new ArrayList<Sensor>();

long startTime;
long currTime;
long lastUpdateTime;

float span = 6*12*IN;

float sensorX0 = 10*IN;
float sensorY0 = 3*IN;
float spacing = 9*IN;
float sensorFov = 20;
int numSensors = (int) Math.ceil(span / spacing);
boolean paused = false;

void setup() {
  size(640, 640,FX2D);
  background(0,0,0);

/*
  client = new MQTTClient(this);
  client.connect("mqtt://try:try@broker.shiftr.io", "processing");
  client.subscribe("/example");
  // client.unsubscribe("/example");
  */

  
  int[][] colors = new int[][] {{255,0,0},{0,255,0},{50,50,255},{255,255,255},
      {255,0,0},{0,255,0},{50,50,255},{255,255,255},
      {255,0,0},{0,255,0},{50,50,255},{255,255,255}};
  for(int i=0; i < numSensors; i++) {
    sensors.add(new Sensor(new PVector(sensorX0 + i * spacing,sensorY0),sensorFov,colors[i]));
  }
  
  targets.add(new Target(0));
  targets.add(new Target(1));
  
  startTime = System.currentTimeMillis();
  lastUpdateTime = System.currentTimeMillis();
  
}

PVector target1 = new PVector(0,sensorY0 + 3*12*IN);
PVector target1Vel = new PVector(4*IN,0);  // in m per second

PVector target2 = new PVector(3.25*12*IN,sensorY0 + 3*12*IN);
PVector target2Vel = new PVector(4*IN,0);  // in m per second

void draw() {
  clear();
  currTime = System.currentTimeMillis();
  
  updateTarget();
  
  for(Target target : targets) {
    target.draw();
  }
  
  for(Sensor sensor : sensors) {
    sensor.draw();
  }
}

void updateTarget() { 
  if (paused) {
    lastUpdateTime = currTime;
  }
  
  long delta = currTime - lastUpdateTime;
  if (delta < 100) return;
  
  //if ((currTime - startTime) > 1000) return;
    
  // calculate new position
  target1.x += delta/1000.0f * target1Vel.x; //<>//
  target1.y += delta/1000.0f * target1Vel.y;
  
  targets.get(0).setPos(target1);
  
  PVector tv = new PVector();
  
  // Calculate position from sensors, set a reading if < distance
  int len = sensors.size(); //<>//
  for(int i=0; i < len; i++) {
    Sensor sensor = sensors.get(i);
    PVector sorigin = sensor.getOrigin();
    
    tv.set(sorigin);
    
    float dist = tv.dist(target1);
    //System.out.printf("time: %d  target: %f %f  dist: %f\n",currTime-startTime,target1.x,target1.y,dist);
    if (dist < sensor.getMaxDist()) {
      
      float ydist = target1.y - sorigin.y;
      
      float beamWidth = sensor.getFov() * 0.018f * ydist;
      float lcx = ((sorigin.x - beamWidth / 2)); 
      float rcx = ((sorigin.x + beamWidth / 2));

      if (target1.x >= lcx && target1.x <= rcx) {
        sensor.addReading((int)(dist * 1000.0));
      } else {
        sensor.addReading(8192);  // Out of range      
      //if (i < 4 && (currTime - startTime < 1000)) System.out.printf("Sensor: %d dist: %d\n",i,(int)(dist*1000));
      }
    } else {
      sensor.addReading(8192);  // Out of range
    }
  }
  
    // calculate new position
  target2.x += delta/1000.0f * target2Vel.x;
  target2.y += delta/1000.0f * target2Vel.y;
  
  targets.get(1).setPos(target2);
  
  // Calculate position from sensors, set a reading if < distance
  len = sensors.size();
  for(int i=0; i < len; i++) {
    Sensor sensor = sensors.get(i);
    PVector sorigin = sensor.getOrigin();
    
    tv.set(sorigin);
    
    float dist = tv.dist(target2);
    //System.out.printf("time: %d  target: %f %f  dist: %f\n",currTime-startTime,target1.x,target1.y,dist);
    if (dist < sensor.getMaxDist()) {
      
      float ydist = target2.y - sorigin.y;
      
      float beamWidth = sensor.getFov() * 0.018f * ydist;
      float lcx = ((sorigin.x - beamWidth / 2)); 
      float rcx = ((sorigin.x + beamWidth / 2));

      if (target2.x >= lcx && target2.x <= rcx) {
        sensor.addReading((int)(dist * 1000.0));
      }
    }
  }

  lastUpdateTime = currTime;  
}

void messageReceived(String topic, byte[] payload) {
  println("new message: " + topic + " - " + new String(payload));
}

void keyPressed() {
  paused = !paused;
}
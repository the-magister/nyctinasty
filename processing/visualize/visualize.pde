import mqtt.*;
//import gifAnimation.*;

//GifMaker gifExport;
//int frames = 0;
//int totalFrames = 360;

MQTTClient client;

float pxPerFt = 36;
//float pxPerFt = 36/3;
float perimRadius = 32/2*pxPerFt;
float sepalRadius = 8/2*pxPerFt;
float archLength = 4*pxPerFt;
float archHeight = archLength;
float flowerRadius = 6*pxPerFt;
int N_SENSOR = 8;
int N_ARCH = 3;
int N_SEPAL = 3;

float[][][] dist = new float[N_SEPAL][N_ARCH][N_SENSOR];

void setup() {
  // at 36 px/ft, we want to display a 45'x45' area
  size(1620, 1620);
//  size(540, 540);

  // mqtt 
  client = new MQTTClient(this);
  //  client.connect("mqtt://192.168.4.1", "visualize");
  //  client.subscribe("nyc/Distance/#");

  // startup
  //  draw();

  // set frame rate
  //  frameRate(50);

  // save it
  //  save("Plan view.png");

  // setup gif
//  gifExport = new GifMaker(this, "export.gif", 10);
//  gifExport.setRepeat(0); // make it an "endless" animation
}

void draw() {
  // simulate messages
  simMessageReceived();

  // background
  background(102);

  // set (0,0) in the center
  translate(width/2, height/2);

  // denote extents and boundaries with dark lines
  noFill();
  stroke(64);

  // show outlines
  drawOutlines();

  // denote physical shapes by white fill
  fill(255);
  stroke(0);

  // show sepal platforms
  drawSepals();

  // show central flower
  drawFlower();

  // save it
//  export();
}

//void export() {
//  gifExport.setDelay(1);
//  gifExport.addFrame();
//  frames++;
//}

void messageReceived(String topic, byte[] payload) {
  println("new message: " + topic + " - " + new String(payload));
}

void simMessageReceived() {
//  boolean allDone = true;
  for ( int s=0; s<N_SEPAL; s++) {
    for ( int a=0; a<N_ARCH; a++) {
      for ( int n=0; n<N_SENSOR; n++) {
        dist[a][s][n] = abs( cos(radians(millis()/10 *(a+1)*(s+1)*(n+1)/(N_SEPAL*N_ARCH*N_SENSOR) % 360)) ) * archHeight;
//        dist[a][s][n] = abs( cos(radians((float)(frames+1) *(a+1)*(s+1)*(n+1) % 360)) ) * archHeight;
//        if ( dist[a][s][n] != archHeight ) allDone=false;
      }
    }
  }
//  if ( allDone ) {
//    gifExport.finish();
//    println("gif saved");
//    exit();
//  }
}

void drawFlower() {
  pushMatrix();

  // drop a triangle
  rotate(radians(-30));
  polygon(flowerRadius, 3);  // Triangle

  popMatrix();
}

void drawOutlines() {
  pushMatrix();

  // trace the 32' radius
  ellipse(0, 0, perimRadius*2, perimRadius*2);  

  // drop a triangle
  rotate(radians(30));
  polygon(perimRadius, 3);  // Triangle

  popMatrix();
}

void drawSepals() {

  pushMatrix();
  rotate(radians(360*0/3));
  translate(0, -perimRadius);
  polygon(sepalRadius, 6);  // Hexagon 
  drawArchSensor(dist[0][0]);
  rotate(radians(360/3));
  drawArchSensor(dist[0][1]);
  rotate(radians(360/3));
  drawArchSensor(dist[0][2]);
  popMatrix();

  pushMatrix();
  rotate(radians(360*1/3));
  translate(0, -perimRadius);
  polygon(sepalRadius, 6);  // Hexagon
  drawArchSensor(dist[1][0]);
  rotate(radians(360/3));
  drawArchSensor(dist[1][1]);
  rotate(radians(360/3));
  drawArchSensor(dist[1][2]);
  popMatrix();

  pushMatrix();
  rotate(radians(360*2/3));
  translate(0, -perimRadius);
  polygon(sepalRadius, 6);  // Hexagon  
  drawArchSensor(dist[2][0]);
  rotate(radians(360/3));
  drawArchSensor(dist[2][1]);
  rotate(radians(360/3));
  drawArchSensor(dist[2][2]);
  popMatrix();
}

void drawArchSensor(float dist[]) {

  // sensors
  pushMatrix();
  translate(-archLength/2, +sepalRadius);
  beginShape(TRIANGLES);
  for ( int i=0; i<N_SENSOR; i++ ) {
    vertex(archLength/8 * i, dist[i]); 
    vertex(archLength/8 * (i+0.5), 0); 
    vertex(archLength/8 * (i+1), dist[i]);
  }
  endShape();  
  popMatrix();
}

void polygon(float radius, int npoints) {
  float angle = TWO_PI / npoints;
  beginShape();
  for (float a = 0; a < TWO_PI; a += angle) {
    float sx = cos(a) * radius;
    float sy = sin(a) * radius;
    vertex(sx, sy);
  }
  endShape(CLOSE);
}
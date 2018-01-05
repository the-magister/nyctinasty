// dimensional information

static final int pxPerFt = 24; // pixel/foot
static final float mmPerFt = 304.8; // mm/foot
static final float pxPermm = (float)pxPerFt/mmPerFt; // mm/foot

static final int canvasSize = 50*pxPerFt; // show a 55'x55' area as canvas

static final float perimRadius = 32/2*pxPerFt; // center of sepal placement on circle with this radius
static final float flowerRadius = 6*pxPerFt; // footprint of the centerpiece

// thickness for extrusions
static final float deckThick = 5.5/12.0*pxPerFt; // 2x6 is 5.5" deep
static final float legThick = 3.5/12.0*pxPerFt; // 4x4 is 3.5" thick

// deck dimensions
static final float deckHeight = 8*pxPerFt;
static final float deckWidth = 4*pxPerFt;
static final float deckLength = sqrt(3.0)*deckWidth; 
static final float deckRadius = deckWidth;
static final float deckLegExtension = sqrt(pow(deckWidth,2.0)-pow(deckWidth/2.0,2.0));

// vertical/post leg dimensions
static final float plegS1 = deckWidth/2.0;
static final float plegDisp = 0.0;
static final float plegAng = atan(plegDisp/deckHeight);
static final float plegS2 = deckHeight/cos(plegAng);
static final float plegH = sqrt(pow(plegS1,2.0)+pow(plegS2,2.0));
static final float plegAngPoint = acos(plegS2/plegH)*2;
static final float plegAngWide = (radians(180.0)-plegAngPoint)/2.0;

// outboard/splayed leg and upper leg
static final float olegS1 = deckWidth/2.0;
static final float olegDisp = sqrt(pow(deckWidth,2.0)-pow(deckWidth/2.0,2.0)); // complete a triangle from plan view
static final float olegAng = atan(olegDisp/deckHeight);
static final float olegS2 = deckHeight/cos(olegAng);
static final float olegH = sqrt(pow(olegS1,2.0)+pow(olegS2,2.0));
static final float olegAngPoint = acos(olegS2/olegH)*2;
static final float olegAngWide = (radians(180.0)-olegAngPoint)/2.0;

// footprint of the sepal
static final float sepalRadius = deckLength/2.0+olegDisp;

// count of the things
static final int N_SENSOR = 8;
static final int N_ARCH = 3;
static final int N_SEPAL = 3;
static final int N_LED = 16;

// animations?
boolean showAnimations = false;

// packages lets us access the project traffic
import mqtt.*;
MQTTClient client;

// accumulate distance information vai mqtt
float[][][] dist = new float[N_SEPAL][N_ARCH][N_SENSOR];

// a human for reference
PShape david;

// center sails
PShape sails;

// camera control
float viewRot = -20.0;
float viewAng = 60.0;
float viewZoom = 0.9;

import peasy.*;
PeasyCam camera;

void settings() {
  // size based on object
  size(canvasSize, canvasSize, P3D);
}

void setup() {
  print("deck: ");
  print(" height=", deckHeight/pxPerFt);
  print(" width=", deckWidth/pxPerFt);
  print(" length=", deckLength/pxPerFt);
  print(" diameter=", deckRadius*2.0/pxPerFt);
  print(" leg extension=", deckLegExtension/pxPerFt);
  println();
  
  print("inboard/post/non-splayed legs: ");
  print(" S1=", plegS1/pxPerFt);
  print(" S2=", plegS2/pxPerFt);
  print(" H=", plegH/pxPerFt);
  print(" displacement=", plegDisp/pxPerFt);
  print(" point angle=", degrees(plegAngPoint));
  print(" wide angle=", degrees(plegAngWide));
  print(" splay angle=", degrees(plegAng));
  println();

  print("outboard/upright/splayed legs: ");
  print(" S1=", olegS1/pxPerFt);
  print(" S2=", olegS2/pxPerFt);
  print(" H=", olegH/pxPerFt);
  print(" displacement=", olegDisp/pxPerFt);
  print(" point angle=", degrees(olegAngPoint));
  print(" wide angle=", degrees(olegAngWide));
  print(" splay angle=", degrees(olegAng));
  println();
  
  print("sepal: ");
  print(" diameter=",sepalRadius*2.0/pxPerFt);
  float triSide = (deckWidth + 2.0*sqrt(pow(deckWidth/2.0,2.0)+pow(olegDisp,2.0)));
  print(" triangle side=", triSide/pxPerFt);
  print(" triangle altitude=", sqrt(3.0)/2.0*triSide/pxPerFt);
  println();
  
  david = loadShape("scan-the-world-michelangelo-s-david - reduced.obj");
//  david.scale(0.5);
  david.rotateZ(radians(180));
  david.scale(1.0/388.0*(69.7/12.0)*pxPerFt); // object is ~388 mm, and we want 67.7 in, average male height in NA
//  david.scale(1.0/2.9);
  david.setFill(color(255));
  david.setStroke(color(255));
//  println("david height=", david.height()/pxPerFt);

  sails = loadShape("center - reduced.obj");
  sails.scale(pxPermm); // looks like it's in mm scale
  sails.setStroke(color(255));
  sails.setFill(color(255));

  // mqtt 
  client = new MQTTClient(this);
  //  client.connect("mqtt://192.168.4.1", "visualize");
  //  client.subscribe("nyc/Distance/#");
  
  // set frame rate
  frameRate(33);
  
  camera = new PeasyCam(this, 0, 0, 0, canvasSize);
}

// https://processing.org/tutorials/p3d/

void draw() {
  // simulate messages
  simMessageReceived();
  
  // background
  background(102);

  // camera settings
  lights();
  
//  ortho(-width/2.0, width/2.0, -height/2.0, height/2.0);
//  float cameraZ = ((height/2/2.0) / tan(PI*60.0/360.0)); // defaults
//  perspective(PI/3.0, width/height, cameraZ/10.0, cameraZ*10.0); // defaults
 
  // set (0,0) in the center
  //translate(width/2, height/2, 0);

//  angle += PI/300;
/*
  camera(
    sin(radians(viewRot))*canvasSize*viewZoom, cos(radians(viewRot))*canvasSize*viewZoom, (height/2.0) / tan(PI*viewAng/180.0),
    0, 0, 0,
    0, 0, -1
  );
*/  
  // denote extents and boundaries with dark lines
  noFill();
  stroke(64);

  // use David as a reference.  
  drawDavid();
  
  // show outlines
  drawOutlines();

  // denote physical shapes by white fill
  fill(255);
  stroke(0);
  
  drawSails();

  drawSepal(0);

  rotate(radians(360.0/3.0));
  drawSepal(1);
  
  rotate(radians(360.0/3.0));
  drawSepal(2);
  
  // save it
//  save("perspective - posts - eqTri degrees.png");
//  exit();
}

void messageReceived(String topic, byte[] payload) {
  println("new message: " + topic + " - " + new String(payload));
}

void simMessageReceived() {
//  boolean allDone = true;
  for ( int s=0; s<N_SEPAL; s++) {
    for ( int a=0; a<N_ARCH; a++) {
      for ( int n=0; n<N_SENSOR; n++) {
        dist[a][s][n] = abs( cos(radians(millis()/10 *(a+1)*(s+1)*(n+1)/(N_SEPAL*N_ARCH*N_SENSOR) % 360)) ) * deckHeight;
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

void keyPressed() {
  if (key == CODED) {
    if (keyCode == UP) {
      viewAng -= 5;
    } else if (keyCode == DOWN) {
      viewAng += 5;
    } else if (keyCode == LEFT) {
      viewRot += 5;
    } else if (keyCode == RIGHT) {
      viewRot -= 5;
    } 
  } else if( key == '-') {
      viewZoom += 0.1;
  } else if( key == '+') {
      viewZoom -= 0.1;
  } else if( key == 'a') {
      showAnimations = !showAnimations;
  }
  if( viewAng>120 ) viewAng=120;
  if( viewAng<30 ) viewAng=30;
  if( viewZoom > 5.0 ) viewZoom=5.0;
  if( viewZoom < 0.1 ) viewZoom=0.1;
  
  println("viewAng=", viewAng, " viewRot=", viewRot, "viewZoom=", viewZoom);
}

void drawSails() {
  pushMatrix();
  translate(0, 0, -deckHeight+1*pxPerFt);
  rotateZ(-PI/6);
  shape(sails);
  popMatrix();
}

void drawDavid() {
  pushMatrix();
  translate(0, +perimRadius, -deckHeight);
  shape(david);   
  popMatrix();
}

void drawLeg(boolean isUp, int sN) {
  
  float angle =  radians(90)-olegAng; 
  if( isUp ) angle = olegAng-radians(90);

  pushMatrix();

  translate(0, +deckLength/2, 0);
  
  rotateX(-angle);

  box(olegS1*2, legThick, legThick);

  pushMatrix();
  translate(+olegS1/2,+olegH/2,0);
  rotateZ(radians(90)+asin(olegS1/olegH));
  box(olegH, legThick, legThick);

  if( isUp ) translate(-olegH/2.0, 0, -legThick/2.0);
  else translate(-olegH/2.0, -legThick/2.0, 0);
  drawLightSegment(isUp, sN); 
  
  popMatrix();
  
  pushMatrix();
  translate(-olegS1/2,+olegH/2,0);
  rotateZ(radians(90)-asin(olegS1/olegH));
  box(olegH, legThick, legThick);
  
  if( isUp ) translate(-olegH/2.0, 0, -legThick/2.0);
  else translate(-olegH/2.0, +legThick/2.0, 0);
  drawLightSegment(isUp, sN);
  
  popMatrix();
    
  popMatrix();
  
  // add a post
  if( !isUp ) {
    angle = radians(90)+plegAng;
    
    pushMatrix();

    translate(0, +deckLength/2, 0);
    
    rotateX(-angle);
  
    box(plegS1*2, legThick, legThick);
  
    pushMatrix();
    translate(+plegS1/2,+plegH/2,0);
    rotateZ(radians(90)+asin(plegS1/plegH));
    box(plegH, legThick, legThick);
    popMatrix();
    
    pushMatrix();
    translate(-plegS1/2,+plegH/2,0);
    rotateZ(radians(90)-asin(plegS1/plegH));
    box(plegH, legThick, legThick);
    popMatrix();
    
    popMatrix();
    
    // add a brace to form a truss
    pushMatrix();

    translate(0, +deckLength/2.0+olegDisp/2.0, -deckHeight);
    rotateZ(radians(90));
    box(olegDisp, legThick, legThick);
    
    popMatrix();
  }
}

void drawSepal(int sN) {

  pushMatrix();
  translate(0, -perimRadius, 0);

  // deck
  pushMatrix();
  box(deckWidth, deckLength, deckThick);
  rotateZ(radians(360/6));
  box(deckWidth, deckLength, deckThick);
  rotateZ(radians(360/6));
  box(deckWidth, deckLength, deckThick);
  popMatrix();
  
  // distance sensors
  pushMatrix();
  drawArchSensor(dist[sN][0]);
  rotateZ(radians(360/3));
  drawArchSensor(dist[sN][1]);
  rotateZ(radians(360/3));
  drawArchSensor(dist[sN][2]);
  rotateZ(radians(360/3));
  popMatrix();

  // legs
  pushMatrix();
  drawLeg(true, sN);
  rotate(radians(360/6));
  drawLeg(false, sN);
  rotate(radians(360/6));
  drawLeg(true, sN);
  rotate(radians(360/6));
  drawLeg(false, sN);
  rotate(radians(360/6));
  drawLeg(true, sN);
  rotate(radians(360/6));
  drawLeg(false, sN);
  popMatrix();

  // roof
  pushMatrix();
  translate(0, 0, +deckHeight);
  rotateZ(radians(-30));
  polygon(sepalRadius, 3);  // Triangle

  // lines
  strokeWeight(1.0/12.0*pxPerFt);
  stroke(255);
  
  rotateZ(radians(+30));
  line(0, olegDisp+deckLength/2.0, 0, 0, olegDisp+deckLength/2.0, -2.0*deckHeight);
  rotateZ(radians(360/3));
  line(0, olegDisp+deckLength/2.0, 0, 0, olegDisp+deckLength/2.0, -2.0*deckHeight);
  rotateZ(radians(360/3));
  line(0, olegDisp+deckLength/2.0, 0, 0, olegDisp+deckLength/2.0, -2.0*deckHeight);
  
  strokeWeight(1.0);
  stroke(0);
  
  popMatrix();

  popMatrix();
}


int lightHue = 0;
void drawLightSegment(boolean isUp, int sN) {
  if( !showAnimations ) return;
  
  colorMode(HSB, 255, 255, 255);
  
  pushMatrix();
  for( int i=0; i<N_LED; i++ ) {
    lightHue = (lightHue+5) % 255;
    stroke(color(lightHue,255,255));
    fill(color(lightHue,255,255));
  
    translate(olegH/(float)(N_LED+1), 0, 0);
    sphere(legThick/2.0*0.9);
  }
  popMatrix();

  colorMode(RGB);
  fill(255);
  stroke(0);
}

void drawArchSensor(float dist[]) {
  if( !showAnimations ) return;
  
  colorMode(HSB, 255, 255, 255);
  stroke(color(64,255,255));
  fill(color(64,255,255));

//  fill(color(255*i/(N_SENSOR-1),128,255));
  // sensors
  pushMatrix();
  translate(-deckWidth/2.0, +deckLength/2.0,0);
  rotateX(radians(-90));
  beginShape(TRIANGLES);
  for ( int i=0; i<N_SENSOR; i++ ) {
//    fill(color(255*i/(N_SENSOR-1),128,255));
    vertex(deckWidth/8.0 * i, dist[i]); 
    vertex(deckWidth/8.0 * (i+0.5), 0); 
    vertex(deckWidth/8.0 * (i+1), dist[i]);
  }
  endShape();  
  popMatrix();
  
//  colorMode(RGB);
  fill(255);
  stroke(0);
}


void drawOutlines() {
  pushMatrix();

  translate(0, 0, -deckHeight);

  // trace the sepal radius
  ellipse(0, 0, perimRadius*2, perimRadius*2);  

  // trace the flower radius
  ellipse(0, 0, flowerRadius*2, flowerRadius*2);  

  // drop a triangle to reference the sepal centers in plan view
  pushMatrix();
  rotate(radians(30));
  polygon(perimRadius, 3);  // Triangle
  popMatrix();

  // drop three circles to describe the sepal footprint
  pushMatrix();
  translate(0, -perimRadius);
  // trace the sepal radius
  ellipse(0, 0, sepalRadius*2.0, sepalRadius*2.0);  
  polygon(deckRadius, 6);  // Hexagon
  rotate(radians(30));
  polygon(sepalRadius, 3);  // Triangle
  rotate(radians(60));
  polygon(sepalRadius, 3);  // Triangle
  popMatrix();

  rotate(radians(360.0/3.0));

  pushMatrix();
  translate(0, -perimRadius);
  // trace the sepal radius
  ellipse(0, 0, sepalRadius*2.0, sepalRadius*2.0);  
  polygon(deckRadius, 6);  // Hexagon
  rotate(radians(30));
  polygon(sepalRadius, 3);  // Triangle
  rotate(radians(60));
  polygon(sepalRadius, 3);  // Triangle
  popMatrix();

  rotate(radians(360.0/3.0));

  pushMatrix();
  translate(0, -perimRadius);
  // trace the sepal radius
  ellipse(0, 0, sepalRadius*2.0, sepalRadius*2.0);  
  polygon(deckRadius, 6);  // Hexagon
  rotate(radians(30));
  polygon(sepalRadius, 3);  // Triangle
  rotate(radians(60));
  polygon(sepalRadius, 3);  // Triangle
  popMatrix();

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
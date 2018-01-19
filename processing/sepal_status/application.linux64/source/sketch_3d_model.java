import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class sketch_3d_model extends PApplet {

// dimensional information

static final int pxPerFt = 24; // pixel/foot
static final float mmPerFt = 304.8f; // mm/foot
static final float pxPermm = (float)pxPerFt/mmPerFt; // mm/foot

static final int canvasSize = 50*pxPerFt; // show a 55'x55' area as canvas

static final float perimRadius = 32/2*pxPerFt; // center of sepal placement on circle with this radius
static final float flowerRadius = 6*pxPerFt; // footprint of the centerpiece

// deck dimensions
static final float deckHeight = 8*pxPerFt;
static final float deckThick = 0.5f*pxPerFt;
static final float deckWidth = 4*pxPerFt;
static final float deckLength = sqrt(3.0f)*deckWidth; 
static final float deckRadius = deckWidth;

// leg dimensions
static final float legThick = 4.0f/12.0f*pxPerFt;
static final float olegS1 = deckWidth/2.0f;
static final float olegAng = radians(20.0f);
static final float olegS2 = deckHeight/cos(olegAng);
static final float olegH = sqrt(pow(olegS1,2.0f)+pow(olegS2,2.0f));
static final float olegAng2 = acos(olegS2/olegH);
static final float olegDisp = deckHeight*tan(olegAng);

static final float plegS1 = deckWidth/2.0f;
static final float plegAng = radians(0);
static final float plegS2 = deckHeight/cos(plegAng);
static final float plegH = sqrt(pow(plegS1,2.0f)+pow(plegS2,2.0f));
static final float plegAng2 = acos(plegS2/plegH);


// count of the things
static final int N_SENSOR = 8;
static final int N_ARCH = 3;
static final int N_SEPAL = 3;

// a human for reference
PShape david;

// center sails
PShape sails;

// camera control
float viewRot = 0.0f;
float viewAng = 60.0f;
float viewZoom = 1.0f;

public void settings() {
  // size based on object
  size(canvasSize, canvasSize, P3D);
}

public void setup() {
  print("deck: ");
  print(" height=", deckHeight/pxPerFt);
  print(" width=", deckWidth/pxPerFt);
  print(" length=", deckLength/pxPerFt);
  print(" diameter=", deckRadius*2.0f/pxPerFt);
  println();
  
  print("outboard/upright/splayed legs: ");
  print(" olegS1=", olegS1/pxPerFt);
  print(" olegS2=", olegS2/pxPerFt);
  print(" olegAng=", degrees(olegAng));
  print(" olegH=", olegH/pxPerFt);
  print(" olegAng2=", degrees(olegAng2));
  print(" olegDisp=", olegDisp/pxPerFt);
  println();
  
  print("inboard/post/non-splayed legs: ");
  print(" plegS1=", plegS1/pxPerFt);
  print(" plegS2=", plegS2/pxPerFt);
  print(" plegAng=", degrees(plegAng));
  print(" plegH=", plegH/pxPerFt);
  print(" plegAng2=", degrees(plegAng2));
  println();
  
  print("sepal: ");
  print(" diameter=",(deckRadius+olegDisp)*2.0f/pxPerFt);
  println();
  
  david = loadShape("scan-the-world-michelangelo-s-david - reduced.obj");
//  david.scale(0.5);
  david.rotateZ(radians(180));
  david.scale(1.0f/388.0f*(69.7f/12.0f)*pxPerFt); // object is ~388 mm, and we want 67.7 in, average male height in NA
//  david.scale(1.0/2.9);
  david.setFill(color(255));
  david.setStroke(color(255));
//  println("david height=", david.height()/pxPerFt);

  sails = loadShape("center - reduced.obj");
  sails.scale(pxPermm); // looks like it's in mm scale
  sails.setStroke(color(255));
  sails.setFill(color(255));
}

// https://processing.org/tutorials/p3d/

public void draw() {
  // background
  background(102);

  // camera settings
  lights();
  
//  camera(width/2.0, height/2.0, (height/2.0) / tan(PI*30.0 / 180.0), width/2.0, height/2.0, 0, 0, 1, 0); // defaults
//  camera(width/2, height/2*3, (height/2.0) / tan(PI*60.0 / 180.0), width/2.0, height/2.0, 0, 0, 1, 0);
 
//  camera(width/2, height/2, (height/2.0) / tan(PI*60.0 / 180.0), width/2.0, height/2.0, 0, 0, 1, 0);

//  camera(width/2, height/2*4, 1, width/2.0, height/2.0, 0, 0, 1, 0);

//  camera(width/2, height/2*3, (height/2.0) / tan(PI*60.0 / 180.0), width/2.0, height/2.0, 0, 0, 1, 0);

//  camera(width/2, height/2*2.5, (height/2.0) / tan(PI*60.0 / 180.0), width/2.0, height/2.0, 0, 0, 1, 0);

//  camera(width/2, height/2*3, (height/2.0) / tan(PI*60.0 / 180.0), width/2.0, height/2.0, 0, 0, 1, 0);

//  ortho(-width/2.0, width/2.0, -height/2.0, height/2.0);
//  float cameraZ = ((height/2/2.0) / tan(PI*60.0/360.0)); // defaults
//  perspective(PI/3.0, width/height, cameraZ/10.0, cameraZ*10.0); // defaults
 
  // set (0,0) in the center
  translate(width/2, height/2, 0);

//  angle += PI/300;
  camera(
    sin(radians(viewRot))*canvasSize*viewZoom, cos(radians(viewRot))*canvasSize*viewZoom, (height/2.0f) / tan(PI*viewAng/180.0f),
    0, 0, 0,
    0, 0, -1
  );
  
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

  drawSepal();

  rotate(radians(360.0f/3.0f));
  drawSepal();
  
  rotate(radians(360.0f/3.0f));
  drawSepal();
  
  // save it
//  save("perspective - posts - 20 degrees.png");
//  exit();
}

public void keyPressed() {
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
      viewZoom += 0.1f;
  } else if( key == '+') {
      viewZoom -= 0.1f;
  }
  if( viewAng>120 ) viewAng=120;
  if( viewAng<30 ) viewAng=30;
  if( viewZoom > 5.0f ) viewZoom=5.0f;
  if( viewZoom < 0.1f ) viewZoom=0.1f;
  
  println("viewAng=", viewAng, " viewRot=", viewRot, "viewZoom=", viewZoom);
}

public void drawSails() {
  pushMatrix();
  translate(0, 0, -deckHeight+1*pxPerFt);
  rotateZ(-PI/6);
  shape(sails);
  popMatrix();
}

public void drawDavid() {
  pushMatrix();
  translate(0, +perimRadius, -deckHeight);
  shape(david);
  popMatrix();
}

public void drawLeg(boolean isUp) {
  
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
  popMatrix();
  
  pushMatrix();
  translate(-olegS1/2,+olegH/2,0);
  rotateZ(radians(90)-asin(olegS1/olegH));
  box(olegH, legThick, legThick);
  popMatrix();
    
  popMatrix();
  
  if( !isUp ) {
    angle = radians(90)+plegAng;
    
    // add symmetric triangle
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
  }
}

public void drawSepal() {

  pushMatrix();
  translate(0, -perimRadius);

//  polygon(sepalRadius, 6);  // Hexagon
  pushMatrix();
  box(deckWidth, deckLength, deckThick);
  rotateZ(radians(360/6));
  box(deckWidth, deckLength, deckThick);
  rotateZ(radians(360/6));
  box(deckWidth, deckLength, deckThick);
  popMatrix();
  
  drawLeg(true);

  rotate(radians(360/6));
  drawLeg(false);
  rotate(radians(360/6));
  drawLeg(true);
  rotate(radians(360/6));
  drawLeg(false);
  rotate(radians(360/6));
  drawLeg(true);
  rotate(radians(360/6));
  drawLeg(false);

  popMatrix();
}

public void drawOutlines() {
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
  // trace the flower radius
  ellipse(0, 0, (deckRadius+olegDisp)*2.0f, (deckRadius+olegDisp)*2.0f);  
  popMatrix();

  rotate(radians(360.0f/3.0f));

  pushMatrix();
  translate(0, -perimRadius);
  // trace the flower radius
  ellipse(0, 0, (deckRadius+olegDisp)*2.0f, (deckRadius+olegDisp)*2.0f);  
  popMatrix();

  rotate(radians(360.0f/3.0f));

  pushMatrix();
  translate(0, -perimRadius);
  // trace the flower radius
  ellipse(0, 0, (deckRadius+olegDisp)*2.0f, (deckRadius+olegDisp)*2.0f);  
  popMatrix();

  popMatrix();
}

public void polygon(float radius, int npoints) {
  float angle = TWO_PI / npoints;
  beginShape();
  for (float a = 0; a < TWO_PI; a += angle) {
    float sx = cos(a) * radius;
    float sy = sin(a) * radius;
    vertex(sx, sy);
  }
  endShape(CLOSE);
}
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "--present", "--window-color=#666666", "--stop-color=#cccccc", "sketch_3d_model" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}

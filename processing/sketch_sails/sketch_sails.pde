// dimensional information

static final int pxPerFt = 24; // pixel/foot
static final int canvasSize = 55*pxPerFt; // show a 55'x55' area as canvas

static final float perimRadius = 32/2*pxPerFt; // center of sepal placement on circle with this radius
static final float flowerRadius = 6*pxPerFt; // footprint of the centerpiece

// deck dimensions
static final float deckHeight = 8*pxPerFt;
static final float deckWidth = 4*pxPerFt;
static final float deckLength = 8*pxPerFt;
static final float deckThick = 0.5*pxPerFt;

// leg dimensions
static final float legH = 10*pxPerFt;
static final float legS1 = deckWidth/2;
static final float legS2 = sqrt(pow(legH,2)-pow(legS1,2));
static final float legAng = acos(deckHeight/legS2);
static final float legThick = 4.0/12.0*pxPerFt;

// count of the things
static final int N_SENSOR = 8;
static final int N_ARCH = 3;
static final int N_SEPAL = 3;

// a human for reference
PImage sailT;

void settings() {
  // size based on object
  size(canvasSize, canvasSize, P3D);
}

void setup() {
  print("deckHeight=", deckHeight/pxPerFt);
  print(" legS2=", legS2/pxPerFt);
  print(" legAng=", degrees(legAng));
  println();
  
  sailT = loadImage("sails_better.png");
  sailT.resize(0, pxPerFt*15);
}

// https://processing.org/tutorials/p3d/

void draw() {
  // background
  background(102);

  // camera settings
  lights();
  
//  camera(width/2.0, height/2.0, (height/2.0) / tan(PI*30.0 / 180.0), width/2.0, height/2.0, 0, 0, 1, 0); // defaults
//  camera(width/2, height/2*3, (height/2.0) / tan(PI*60.0 / 180.0), width/2.0, height/2.0, 0, 0, 1, 0);
 
//  camera(width/2, height/2, (height/2.0) / tan(PI*60.0 / 180.0), width/2.0, height/2.0, 0, 0, 1, 0);

//  camera(width/2, height/2*4, (height/2.0) / tan(PI*60.0 / 180.0), width/2.0, height/2.0, 0, 0, 1, 0);

//  ortho(-width/2.0, width/2.0, -height/2.0, height/2.0);
//  float cameraZ = ((height/2/2.0) / tan(PI*60.0/360.0)); // defaults
//  perspective(PI/3.0, width/height, cameraZ/10.0, cameraZ*10.0); // defaults
 
  // set (0,0) in the center
  translate(width/2, height/2, 0);

  // denote extents and boundaries with dark lines
  noFill();
  stroke(64);

  // show outlines
  drawOutlines();

  // denote physical shapes by white fill
  fill(255);
  stroke(0);

  imageMode(CENTER);
  image(sailT, 0, 0);


}

void drawSail() {
  // Uh.
}

void drawLeg(boolean isUp) {
  
  float angle =  radians(90)-legAng; 
  if( isUp ) angle = legAng-radians(90);

  pushMatrix();

  translate(0, +deckLength/2, 0);
  rotateX(-angle);

  box(legS1*2, legThick, legThick);

  pushMatrix();
  translate(+legS1/2,+legH/2,0);
  rotateZ(radians(90)+asin(legS1/legH));
  box(legH, legThick, legThick);
  popMatrix();
  
  pushMatrix();
  translate(-legS1/2,+legH/2,0);
  rotateZ(radians(90)-asin(legS1/legH));
  box(legH, legThick, legThick);
  popMatrix();
  
  popMatrix();
}

void drawSepal() {

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

void drawOutlines() {
  pushMatrix();

  translate(0, 0, -deckHeight);

  // trace the sepal radius
  ellipse(0, 0, perimRadius*2, perimRadius*2);  

  // trace the flower radius
  ellipse(0, 0, flowerRadius*2, flowerRadius*2);  

  // drop a triangle to reference the sepal centers in plan view
  rotate(radians(30));
  polygon(perimRadius, 3);  // Triangle

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
float pxPerFt = 36;
float pxPerIn = 3;
float perimRadius = 32/2*pxPerFt;
float sepalRadius = 8/2*pxPerFt;
float archLength = 4*pxPerFt;
float archHeight = archLength;
float flowerRadius = 6*pxPerFt;
int N_SENSOR = 8;
int N_ARCH = 3;
int N_SEPAL = 3;

void setup() {
  // at 36 px/ft, we want to display a 45'x45' area
  size(1620, 1620);
  background(102);

  // (0,0) is the center
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
  save("Plan view.png");
  
}

void draw() {
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
  float[] dist = {archHeight, archHeight/2, archHeight/3, archHeight/4, archHeight/5, archHeight/6, archHeight/7, archHeight/8};
  
  pushMatrix();
  rotate(radians(360*0/3));
  translate(0, -perimRadius);
  polygon(sepalRadius, 6);  // Hexagon 
  drawArchSensor(dist);
  rotate(radians(360/3));
  drawArchSensor(dist);
  rotate(radians(360/3));
  drawArchSensor(dist);
  popMatrix();
  
  pushMatrix();
  rotate(radians(360*1/3));
  translate(0, -perimRadius);
  polygon(sepalRadius, 6);  // Hexagon
  drawArchSensor(dist);
  rotate(radians(360/3));
  drawArchSensor(dist);
  rotate(radians(360/3));
  drawArchSensor(dist);
  popMatrix();
  
  pushMatrix();
  rotate(radians(360*2/3));
  translate(0, -perimRadius);
  polygon(sepalRadius, 6);  // Hexagon  
  drawArchSensor(dist);
  rotate(radians(360/3));
  drawArchSensor(dist);
  rotate(radians(360/3));
  drawArchSensor(dist);
  popMatrix();
}

void drawArchSensor(float dist[]) {

  // sensors
  pushMatrix();
  translate(-archLength/2, +sepalRadius);
  beginShape(TRIANGLES);
  for( int i=0; i<N_SENSOR; i++ ) {
    vertex(archLength/8 * i, dist[i]); 
    vertex(archLength/8 * (i+0.5),0); 
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
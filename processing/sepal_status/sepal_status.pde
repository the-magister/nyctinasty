// display size (tiny)
static final int pixelx = 64;
static final int pixely = 48;

// dimensional information
static final int H = pixely;

// vertex definitions
static final int[][][] legVert = new int[6][3][2];
static final int[][][] distVert = new int[3][3][2];

// packages lets us access the project traffic
import mqtt.*;
MQTTClient client;

// active component tracking
//                         up0    down1   up2    down3  up4    down5
boolean[] isLegActive = {  true,  false,  true,  false, true, false};
//                         arch0          arch1         arch2   
boolean[] isDistActive = { true,          true,         true};

// colors
static final int WHITE = 255;
static final int BLACK = 0;

void settings() {
  // size based on object
  size(pixelx, pixely);
}

void setup() {
  // mqtt 
  client = new MQTTClient(this);
  //  client.connect("mqtt://192.168.4.1", "visualize");
  //  client.subscribe("nyc/Distance/#");
  
  // set up dimensions
  initializeVertices();
  
  // set frame rate
  frameRate(33);
}

// https://processing.org/tutorials/p3d/

void draw() {
  // background
  background(BLACK);

  // show status
  for( int i=0; i<6; i++ ) {
    if( isLegActive[i] ) filledTriangle(legVert[i][0][0],legVert[i][0][1],legVert[i][1][0],legVert[i][1][1],legVert[i][2][0],legVert[i][2][1]);
    else emptyTriangle(legVert[i][0][0],legVert[i][0][1],legVert[i][1][0],legVert[i][1][1],legVert[i][2][0],legVert[i][2][1]);
  }
  for( int i=0; i<3; i++ ) {
    if( isDistActive[i] ) filledTriangle(distVert[i][0][0],distVert[i][0][1],distVert[i][1][0],distVert[i][1][1],distVert[i][2][0],distVert[i][2][1]);
    else emptyTriangle(distVert[i][0][0],distVert[i][0][1],distVert[i][1][0],distVert[i][1][1],distVert[i][2][0],distVert[i][2][1]);
  }

}

void emptyTriangle(int x0, int y0, int x1, int y1, int x2, int y2) {
  noFill();
  drawTriangle(x0, y0, x1, y1, x2, y2);
}
void filledTriangle(int x0, int y0, int x1, int y1, int x2, int y2) {
  fill(WHITE);
  drawTriangle(x0, y0, x1, y1, x2, y2);
}

void drawTriangle(int x0, int y0, int x1, int y1, int x2, int y2) {
  stroke(WHITE);
  beginShape(TRIANGLES);
  vertex(x0,y0);
  vertex(x1,y1);
  vertex(x2,y2);
  endShape(CLOSE);
}

void messageReceived(String topic, byte[] payload) {
  println("new message: " + topic + " - " + new String(payload));
}

void initializeVertices() {
  // legs
  float R = H/2;
  float r = R/3.0*sqrt(3.0);
  
  float gam = 3;
  
  // angles
  float angle = TWO_PI/6.0;
  float offset = TWO_PI/6.0/2.0;
  float rot = TWO_PI/6.0/2.0;
  
  for (int i=0; i<6; i++) {
    // outer perimeter
    legVert[i][0][0] = (int)(cos(angle*i+rot) * R + R);
    legVert[i][0][1] = (int)(sin(angle*i+rot) * R + R);
    // inner perimeter
    legVert[i][1][0] = (int)(cos(angle*i+offset+rot) * r + R);
    legVert[i][1][1] = (int)(sin(angle*i+offset+rot) * r + R);

    legVert[i][2][0] = (int)(cos(angle*i-offset+rot) * r + R);
    legVert[i][2][1] = (int)(sin(angle*i-offset+rot) * r + R);
    
    // distance sensors
    if( (i % 2) == 0 ) {
      distVert[i/2][0][0] = (int)(cos(angle*i+rot) * gam + R);
      distVert[i/2][0][1] = (int)(sin(angle*i+rot) * gam + R);
      
      distVert[i/2][1][0] = (int)(cos(angle*i+offset+rot) * (r-3.0) + R);
      distVert[i/2][1][1] = (int)(sin(angle*i+offset+rot) * (r-3.0) + R);
      
      distVert[i/2][2][0] = (int)(cos(angle*i-offset+rot) * (r-3.0) + R);
      distVert[i/2][2][1] = (int)(sin(angle*i-offset+rot) * (r-3.0) + R);
    }
  }
}
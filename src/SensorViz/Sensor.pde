import java.util.LinkedList;

public static final float M_TO_PIXELS = 256;  // 640 pixels = 2.5m
public class Sensor {
  float maxDist = 1.6;  // Maximum sensor distance in meters
  private static final int READ_BUFFER = 10;
  
  // Origin in world coordinates
  PVector origin;
  float fov = 15;  // Degrees fov of the sensor
  int maxSensorValue = 2000;
  
  LinkedList<Integer> readings = new LinkedList<Integer>();
  
  boolean displayCone = true;
  boolean displayHistory = true;
  boolean displayPossibleLocation = true;
  
  PVector leftCorner, rightCorner;
  int[] baseColor = new int[3]; 
  
  public Sensor(PVector origin) {
    this(origin,15,new int[] {255,0,0});
  }

  public Sensor(PVector origin,float fov,int[] bcolor) {
    this.origin = origin;
    this.fov = fov;
    
    float beamWidth = fov * 0.018 * maxDist;
    float beamHeight = maxDist;
    
    int lcx = (int) ((origin.x - beamWidth / 2) * M_TO_PIXELS); 
    int rcx = (int) ((origin.x + beamWidth / 2) * M_TO_PIXELS);
    int lcy = (int) ((origin.y + beamHeight) * M_TO_PIXELS);

    leftCorner = new PVector(lcx,lcy);
    rightCorner = new PVector(rcx,lcy); 
    
    baseColor[0] = bcolor[0];
    baseColor[1] = bcolor[1];
    baseColor[2] = bcolor[2];
  }

  public synchronized void addReading(int dist) {
    readings.addFirst(dist);
    
    if (readings.size() > READ_BUFFER) {
      readings.removeLast();
    }
  }
  
  public synchronized void draw() {
    stroke(255, 255, 255);
    point(origin.x * M_TO_PIXELS,origin.y * M_TO_PIXELS);
    
    if (displayCone) {
      stroke(baseColor[0]/2, baseColor[1]/2, baseColor[2]/2);
      line(origin.x * M_TO_PIXELS,origin.y * M_TO_PIXELS,leftCorner.x,leftCorner.y);
      line(origin.x * M_TO_PIXELS,origin.y * M_TO_PIXELS,rightCorner.x,rightCorner.y);   
    }
    
    if (readings.size() > 0) {
      float dist = readings.getFirst() / 1000.0;  // In mm's
      if (dist < maxDist) {
        int y = (int) (dist * M_TO_PIXELS);
        
        fill(baseColor[0],baseColor[1],baseColor[2]);
        stroke(baseColor[0],baseColor[1],baseColor[2]);
        
        int px = (int) (origin.x * M_TO_PIXELS);
        int py = (int) (origin.y * M_TO_PIXELS + y);
    
        rect(px,py,2,2);
        
        if (displayPossibleLocation) {
          noFill();
          stroke(baseColor[0]/3, baseColor[1]/3, baseColor[2]/3);
          int diam = 2*(int) ((dist) * M_TO_PIXELS);
          //ellipse(origin.x * M_TO_PIXELS,origin.y * M_TO_PIXELS,diam,diam);
          
          float beamWidth = fov * 0.018f * dist;
          float amt = (float) (Math.PI * beamWidth / dist / 2);

          amt = fov / 2 * PI / 180;
          float start = HALF_PI - amt;
          float end = HALF_PI  + amt;
          arc(origin.x * M_TO_PIXELS,origin.y * M_TO_PIXELS,diam,diam,start,end);
          // Change to an arc with this clipping?
          /*
                float ydist = target1.y - sorigin.y;
      
      float beamWidth = sensor.getFov() * 0.018f * ydist;
      float lcx = ((sorigin.x - beamWidth / 2)); 
      float rcx = ((sorigin.x + beamWidth / 2));

      if (target1.x >= lcx && target1.x <= rcx) {
*/
        }
      }
    }
  }
  
  public PVector getOrigin() {
    return origin;
  }
  
  public float getMaxDist() {
    return maxDist;
  }
  
  public float getFov() {
    return fov;
  }
}
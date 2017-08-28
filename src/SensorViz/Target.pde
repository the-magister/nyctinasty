/**
 * A tracked target
 */
public class Target {
  public static final float M_TO_PIXELS = 256;  // 640 pixels = 2.5m

  int targetId;
  PVector pos = new PVector();
  int[] baseColor = new int[] {255,255,255};
  int size = 4;
  
  public Target(int id) {
    targetId = id;
  }
  
  public void draw() {
      stroke(baseColor[0], baseColor[1], baseColor[2]);
      fill(baseColor[0], baseColor[1], baseColor[2]);

      ellipse(pos.x * M_TO_PIXELS,pos.y * M_TO_PIXELS,size,size);    
  }
  
  public void setPos(PVector pos) {
    this.pos.set(pos);
  }
}
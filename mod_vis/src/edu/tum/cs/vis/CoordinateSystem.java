package edu.tum.cs.vis;

public class CoordinateSystem implements Drawable {

	public float size;
	
	public CoordinateSystem(float size) {
		this.size = size;
	}
	
	public void draw(Canvas c) {
		c.pushMatrix();
		c.stroke(255, 0, 0);
		c.line(0, 0, 0, size, 0, 0);
		c.popMatrix();
		c.pushMatrix();
		c.stroke(0, 255, 0);
		c.line(0, 0, 0, 0, size, 0);
		c.popMatrix();
		c.pushMatrix();
		c.stroke(0, 0, 255);
		c.line(0, 0, 0, 0, 0, size);
		c.stroke(0, 0, 0);
		c.popMatrix();
	}
}

package edu.tum.cs.vis.items;

import edu.tum.cs.util.math.Vector3f;
import edu.tum.cs.vis.Canvas;
import edu.tum.cs.vis.Drawable;

public class Point implements Drawable {

	public Vector3f v;
	public float size;
	public int color;
	
	public Point(float x, float y, float z, int color, float size) {
		this.v = new Vector3f(x,y,z);
		this.color = color;
		this.size = size;
	}
	
	public Point(Vector3f v, int color, float size) {
		this(v.x, v.y, v.z, color, size);
	}
	
	public Point(Point p) {
		copyPos(p);
		color = p.color;
		size = p.size;
	}
	
	/**
	 * draws this point as a circle in the xy plane
	 */
	public void draw(Canvas c) {
	    c.pushMatrix();
		c.translate(0,0,v.z);
	    c.noStroke(); // do not draw outline
	    c.fill(color);	    
	    c.ellipse(v.x, v.y, size, size);
	    c.popMatrix();
	}
	
	public void drawAsSphere(Canvas c) {
		c.pushMatrix();
		c.fill(color);
		c.color(color);
		c.noStroke();
		c.translate(v.x,v.y,v.z);
		c.sphere(size);
		c.popMatrix();
	}
	
	@Override
	public String toString() {
		return String.format("(%f/%f/%f)", v.x, v.y, v.z);
	}

	/**
	 * copies the position of another point into this
	 * @param p2
	 */
	public void copyPos(Point p2) {
		v = new Vector3f(p2.v);
	}	
}

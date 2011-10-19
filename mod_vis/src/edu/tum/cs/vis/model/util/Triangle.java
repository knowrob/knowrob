package edu.tum.cs.vis.model.util;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;

import processing.core.PApplet;
import processing.core.PConstants;

public class Triangle {
	 /**
     * the 3 Vertices of the Triangle
     */
	public Point3f position[];
    /**
     * Texture-Points
     */
	public Point2f texPosition[];

	public Appearance appearance;
     
	public Triangle() {
		position = new Point3f[3];
	}

     /**
      * Draw the triangle onto the applet.
      * @param applet Applet to draw on
      */
     public void draw(PApplet applet)
     {
		if (!appearance.containsTexture)
         {
			applet.fill(appearance.colour.red * 255,
					appearance.colour.green * 255,
					appearance.colour.blue * 255,
					appearance.colour.transparency * 255);
             applet.beginShape(PConstants.TRIANGLES);

			applet.vertex(position[0].x, position[0].y, position[0].z);
			applet.vertex(position[1].x, position[1].y, position[1].z);
			applet.vertex(position[2].x, position[2].y, position[2].z);

             applet.endShape();

         } else
         {
        	 applet.beginShape(PConstants.TRIANGLES);
			applet.texture(appearance.imageReference);

			applet.vertex(position[0].x, position[0].y, position[0].z,
					texPosition[0].x, texPosition[0].y);
			applet.vertex(position[1].x, position[1].y, position[1].z,
					texPosition[1].x, texPosition[1].y);
			applet.vertex(position[2].x, position[2].y, position[2].z,
					texPosition[2].x, texPosition[2].y);

        	 applet.endShape();

         }
     }

	/**
	 * Moves the triangle in the given direction. Used to center model
	 * 
	 * @param x
	 *            direction in x
	 * @param y
	 *            direction in y
	 * @param z
	 *            direction in z
	 */
	public void translate(float x, float y, float z) {
		for (int v = 0; v < 3; v++) {
			position[v].x += x;
			position[v].y += y;
			position[v].z += z;
		}
	}
}

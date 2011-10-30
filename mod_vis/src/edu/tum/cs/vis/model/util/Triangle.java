package edu.tum.cs.vis.model.util;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;

import processing.core.PApplet;
import processing.core.PConstants;

/**
 * Triangle of a model. May have color or texture.
 * 
 * @author Stefan Profanter
 *
 */
public class Triangle extends DrawObject {

    /**
     * Texture-Points
     */
	public Point2f texPosition[];

	/**
	 * Color or texture of triangle
	 */
	public Appearance appearance;
     
	/**
	 * Default constructor
	 */
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
}

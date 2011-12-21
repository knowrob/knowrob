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
     public void draw(PApplet applet, int overrideColor)
     {

 		applet.noStroke();
		if (!appearance.containsTexture || overrideColor != 0)
         {
			if (overrideColor != 0)
				applet.fill(overrideColor);
			else
				applet.fill(appearance.colour.red * 255,
					appearance.colour.green * 255,
					appearance.colour.blue * 255,
					appearance.colour.transparency * 255);
             applet.beginShape(PConstants.TRIANGLES);

             for (int i=0; i<3; i++)
            	 applet.vertex(position[i].x, position[i].y, position[i].z);

             applet.endShape();

         } else
         {
        	 if (overrideColor != 0)
 				applet.fill(overrideColor);
 			else
 				applet.fill(1 * 255,
 					1 * 255,
 					1 * 255,
 					0 * 255);
			applet.beginShape(PConstants.TRIANGLES);
			applet.texture(appearance.imageReference);
			
			for (int i=0; i<3; i++)
			{
				applet.vertex(position[i].x, position[i].y, position[i].z,
						texPosition[i].x, texPosition[i].y);				
			}
			

			
			applet.endShape();

         }
     }
}

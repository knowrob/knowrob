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
	 * Default constructor
	 */
	public Triangle() {
		position = new Point3f[3];
	}

     /**
      * Draw the triangle onto the applet.
      * @param applet Applet to draw on
      * @param overrideColor Gray value from 0 to 255. 0 = don't override
      */
     public void draw(PApplet applet, int overrideColor)
     {
   		setColor(applet, overrideColor);
		if (appearance.imageReference==null || overrideColor != 0)
        {
			//no texture only color
             applet.beginShape(PConstants.TRIANGLES);

             for (int i=0; i<3; i++)
            	 applet.vertex(position[i].x, position[i].y, position[i].z);

             applet.endShape();

		} else
		{
			//has texture
			applet.beginShape(PConstants.TRIANGLES);
			applet.texture(appearance.imageReference);
			
			for (int i=0; i<3; i++)
				applet.vertex(position[i].x, position[i].y, position[i].z,
						texPosition[i].x, texPosition[i].y);	
						
			applet.endShape();

        }
    }
}

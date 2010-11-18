package de.tum.in.fipm.kipm.gui.visualisation.items;

import edu.tum.cs.vis.Canvas;

public class Chair extends Item {

	
	public Chair(float m00, float m01, float m02, float m03, float m10,
			float m11, float m12, float m13, float m20, float m21, float m22,
			float m23, float m30, float m31, float m32, float m33, float xdim,
			float ydim, float zdim) {
		
		super(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31,
				m32, m33, xdim, ydim, zdim);
	}

	@Override
	public void drawIt(Canvas c) {
		
		c.translate(0, 0, 45f);	  
		c.box(40f, 40f, -2f);
		c.translate(20.0f, 20.0f, -20.0f);	c.box(-5f, 5f, 45f);
		c.translate(-40.0f,0,22.5f);		c.box(-5f, 5f, 90f);
		// back rest
		c.translate(0.0f,-20f,30f);c.box(-2f, 40f, 20f);
		
		c.translate(0,-20.0f,-30);			c.box(-5f, 5f, 90f);
		c.translate(40f,0,-22.5f);			c.box(-5f, 5f, 45f);

	}

}

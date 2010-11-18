package de.tum.in.fipm.kipm.gui.visualisation.items;

import processing.core.PConstants;
import edu.tum.cs.vis.Canvas;

public class Tetrapak extends Item {

	public Tetrapak(float m00, float m01, float m02, float m03, float m10,
			float m11, float m12, float m13, float m20, float m21, float m22,
			float m23, float m30, float m31, float m32, float m33, float xdim,
			float ydim, float zdim) {
		
		super(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31,
				m32, m33, xdim, ydim, zdim);
	}

	@Override
	public void drawIt(Canvas c) {

		// translate it down by half its height
		c.translate(0.0f, 0.0f, -12.5f);

		// "wrapper"
		c.beginShape(PConstants.QUAD_STRIP);
		c.vertex(5,5,0); c.vertex(-5,5,0);
		c.vertex(5,5,20); c.vertex(-5,5,20);
		c.vertex(5,0,25); c.vertex(-5,0,25);
		c.vertex(5,-5,20); c.vertex(-5,-5,20);
		c.vertex(5,-5,0); c.vertex(-5,-5,0);
		c.endShape(PConstants.CLOSE);
		
		// front side
		c.beginShape(PConstants.TRIANGLE_STRIP);
		c.vertex(-5,5,0);
		c.vertex(-5,-5,0);
		c.vertex(-5,5,20);
		c.vertex(-5,-5,20);
		c.vertex(-5,0,25);
		c.endShape();
		
		// back side
		c.beginShape(PConstants.TRIANGLE_STRIP);
		c.vertex(5,5,0);
		c.vertex(5,-5,0);
		c.vertex(5,5,20);
		c.vertex(5,-5,20);
		c.vertex(5,0,25);
		c.endShape();

	}
}

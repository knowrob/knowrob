package de.tum.in.fipm.kipm.gui.visualisation.items;

import edu.tum.cs.vis.Canvas;
import edu.tum.cs.util.math.Vector3f;
import edu.tum.cs.vis.items.Cylinder;

public class PancakeMix extends Item {
	

	public PancakeMix(float m00, float m01, float m02, float m03, float m10,
			float m11, float m12, float m13, float m20, float m21, float m22,
			float m23, float m30, float m31, float m32, float m33, float xdim,
			float ydim, float zdim) {
		super(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31,
				m32, m33, xdim, ydim, zdim);
	}

	@Override
	public void drawIt(Canvas c) {
		
		(new Cylinder(new Vector3f(0f, 0f, 0f), new Vector3f(0f, 0f, 15f), 4f)).draw(c); // cup
		(new Cylinder(new Vector3f(0f, 0f, 15f), new Vector3f(0f, 0f, 18f), 2.5f)).draw(c); // nozzle
		
	}
}


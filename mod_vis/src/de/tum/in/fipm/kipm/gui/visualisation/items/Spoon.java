package de.tum.in.fipm.kipm.gui.visualisation.items;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;

import edu.tum.cs.util.math.Vector3f;
import edu.tum.cs.vis.Canvas;
import edu.tum.cs.vis.items.Cylinder;

public class Spoon extends Item {

	public Spoon(float m00, float m01, float m02, float m03, float m10,
			float m11, float m12, float m13, float m20, float m21, float m22,
			float m23, float m30, float m31, float m32, float m33, float xdim,
			float ydim, float zdim) {
		
		super(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31,
				m32, m33, xdim, ydim, zdim);
	}
	
	public Spoon(Matrix4d pose, Vector3d dim){
		super(pose, dim);
	}
	
	public void drawIt(Canvas c) {

		// main spree
		(new Cylinder(new Vector3f(-7.8f, 0f, -1.04f), new Vector3f(3.9f, 0f, -1.04f), 1.04f)).draw(c);
		
		// spoon
		(new Cylinder(new Vector3f(5.2f, 0f, -0.78f), new Vector3f(5.2f, 0f, -1.3f), 2.6f)).draw(c);
		
	}

}

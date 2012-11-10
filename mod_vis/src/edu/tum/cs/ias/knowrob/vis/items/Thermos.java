package edu.tum.cs.ias.knowrob.vis.items;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;

import edu.tum.cs.ias.knowrob.vis.Canvas;

public class Thermos extends Item {


	public Thermos(float m00, float m01, float m02, float m03, float m10,
			float m11, float m12, float m13, float m20, float m21, float m22,
			float m23, float m30, float m31, float m32, float m33, float xdim,
			float ydim, float zdim) {
		
		super(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31,
				m32, m33, xdim, ydim, zdim);
	}
	
	public Thermos(Matrix4d pose, Vector3d dim){
		super(pose, dim);
	}
	
	@Override
	public void drawIt(Canvas c) {
		
		// RSS: use box instead of jug for coffee
		c.translate(0, 0, 0.10f);
		c.box(-0.10f, 0.06f, 0.20f);
		
//		(new Cylinder(new Vector3f(0f, 0f, 0f), new Vector3f(0f, 0f, 27f), 7f)).draw(c); // body
//		(new Cylinder(new Vector3f(7f, 1f, 15f), new Vector3f(7f, -1f, 15f),7f)).draw(c); // handle
//		
	}
}

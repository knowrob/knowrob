package edu.tum.cs.ias.knowrob.vis.items;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;

import edu.tum.cs.ias.knowrob.vis.Canvas;

public class Table extends Item {

	
	public Table(float m00, float m01, float m02, float m03, float m10,
			float m11, float m12, float m13, float m20, float m21, float m22,
			float m23, float m30, float m31, float m32, float m33, float xdim,
			float ydim, float zdim) {
		
		super(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31,
				m32, m33, xdim, ydim, zdim);
	}
	public Table(Matrix4d pose, Vector3d dim){
		super(pose, dim);
	}

	
	@Override
	public void drawIt(Canvas c) {
		
		// draw legs
		float depth = xdim;
		float width = ydim;
		float height= zdim;

    	float legs  = 0.05f;
    	float ttop  = 0.02f;

	    c.translate(0, 0, 0.5f*height);
	    c.box(depth, width, -ttop);
	    
	    c.translate( 0.45f*depth, 0.45f*width, -0.5f*height); c.box(-legs, legs, height);
	    c.translate(-0.9f*depth,  0,            0);           c.box(-legs, legs, height);
	    c.translate( 0,          -0.9f*width,   0);           c.box(-legs, legs, height);
	    c.translate( 0.9f*depth,  0,            0);           c.box(-legs, legs, height);
		
	}

}

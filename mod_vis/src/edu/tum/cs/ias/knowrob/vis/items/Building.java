
package edu.tum.cs.ias.knowrob.vis.items;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;

import edu.tum.cs.ias.knowrob.vis.Canvas;


public class Building extends Item {

	public Building (float m00, float m01, float m02, float m03, float m10,
			float m11, float m12, float m13, float m20, float m21, float m22,
			float m23, float m30, float m31, float m32, float m33, float xdim,
			float ydim, float zdim) {
		
      super(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33, xdim, ydim, zdim);
	}
	public Building(Matrix4d pose, Vector3d dim){
		super(pose, dim);
	}

	@Override
	public void drawIt(Canvas c) {

      c.translate(0,0, -zdim/2);
      c.stroke(100,100,100);
      c.strokeWeight((float)1.5);
      c.noFill();
      c.box(this.xdim, this.ydim, 0.1f);
      
	}
}

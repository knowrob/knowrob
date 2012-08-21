package edu.tum.cs.ias.knowrob.vis.items;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;

import edu.tum.cs.ias.knowrob.vis.Canvas;

public class Chair extends Item {

    private final static float LEG_DIM  = 0.05f;
    private final static float SEAT_DIM = 0.02f;
    
	
	public Chair(float m00, float m01, float m02, float m03, float m10,
			float m11, float m12, float m13, float m20, float m21, float m22,
			float m23, float m30, float m31, float m32, float m33, float xdim,
			float ydim, float zdim) {
		
		super(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31,
				m32, m33, xdim, ydim, zdim);
	}
	
	public Chair(Matrix4d pose, Vector3d dim){
		super(pose, dim);
	}
	
	@Override
	public void drawIt(Canvas c) {
      
      c.box(xdim, ydim, SEAT_DIM);
		
      // frontside legs
      c.translate(xdim/2 - LEG_DIM, ydim/2-LEG_DIM, -zdim/4);   c.box(LEG_DIM, LEG_DIM, zdim/2);
      c.translate(0 ,-(ydim - 2*LEG_DIM),  0);   c.box(LEG_DIM, LEG_DIM, zdim/2);
		
      // backside legs
      c.translate(- (xdim - 2*LEG_DIM),0, zdim/4);     c.box(LEG_DIM, LEG_DIM, zdim);
      c.translate(0, ydim - 2*LEG_DIM, 0);            c.box(LEG_DIM, LEG_DIM, zdim);

      // back rest
      c.translate(0.0f, - (ydim/2 - LEG_DIM), 0.75f * zdim/2);   c.box(SEAT_DIM,ydim, zdim/4);
      
	}

}

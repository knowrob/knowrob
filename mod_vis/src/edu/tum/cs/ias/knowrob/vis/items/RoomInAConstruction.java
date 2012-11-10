package edu.tum.cs.ias.knowrob.vis.items;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;

import edu.tum.cs.ias.knowrob.vis.Canvas;


public class RoomInAConstruction extends Item {
    
	public RoomInAConstruction(float m00, float m01, float m02, float m03, float m10,
			float m11, float m12, float m13, float m20, float m21, float m22,
			float m23, float m30, float m31, float m32, float m33, 
                             float xdim, float ydim, float zdim){
		

		super(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31,
				m32, m33, xdim, ydim, zdim);
	}
	public RoomInAConstruction(Matrix4d pose, Vector3d dim){
		super(pose, dim);
	}

	@Override
	public void drawIt(Canvas c) {

      c.translate(0,0,-zdim/2 - 0.05f);
      //c.stroke(50,50,50);
      
      // ground plane
      c.box(this.xdim, this.ydim, 0.02f);
      c.fill(127, 127, 127, 100);
      c.translate(0,0,zdim/2);
      c.box(this.xdim, this.ydim, this.zdim);
      
      
      // center of room
      //c.fill(0,0,200);
//      c.box(xdim/15,ydim/15,0.1f);

      // float diff_x = - (this.trafoMatrix[3] - xtrans); 
      // float diff_y = - (this.trafoMatrix[7] - ytrans);
      // // rotation only works since it is a 180 degree rotation!?
      // float rot_x = trafoMatrix[0] * diff_x + trafoMatrix[1] * diff_y;
      // float rot_y = trafoMatrix[4] * diff_x + trafoMatrix[5] * diff_y;      
      // // origin
      // //c.translate(rot_x,rot_y,8f);
      // //c.stroke(250,200,0);
      // //c.noFill();
      // //c.strokeWeight(1f);
      // //c.box(1f, 8f, 16f);
      // c.translate(rot_x,rot_y,0f);
      // c.fill(250,200,0);
      // c.box(20f,20f,20f);
     
  }
}

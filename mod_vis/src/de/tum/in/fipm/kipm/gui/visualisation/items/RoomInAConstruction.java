package de.tum.in.fipm.kipm.gui.visualisation.items;

import edu.tum.cs.vis.Canvas;


public class RoomInAConstruction extends Item {
    
    float xtrans, ytrans, ztrans;
    
	public RoomInAConstruction(float m00, float m01, float m02, float m03, float m10,
			float m11, float m12, float m13, float m20, float m21, float m22,
			float m23, float m30, float m31, float m32, float m33, 
                             float xdim, float ydim, float zdim,
                             float xtrans, float ytrans, float ztrans) {
		

		super(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31,
				m32, m33, xdim, ydim, zdim);
    this.xtrans = xtrans;
    this.ytrans = ytrans;
    this.ztrans = ztrans;
	}

	@Override
	public void drawIt(Canvas c) {

      // ensure box size = image size?!?
      //c.stroke(200,200,200);
      //c.noFill();
      c.translate(0,0,-zdim/2);
      c.box(this.xdim, this.ydim, 1f);//this.zdim);
      
      // // center of room
      // //c.translate(0f,0f,0f);
      c.fill(0,0,200);
      //c.sphere(2f); // visualization very slow when using spheres!
      c.box(20f,20f,20f);

      
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

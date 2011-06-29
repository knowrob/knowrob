
package de.tum.in.fipm.kipm.gui.visualisation.items;

import edu.tum.cs.vis.Canvas;
import processing.core.PImage;
import processing.core.PConstants;


public class LevelOfAConstruction extends Item {

	public LevelOfAConstruction(float m00, float m01, float m02, float m03, float m10,
			float m11, float m12, float m13, float m20, float m21, float m22,
			float m23, float m30, float m31, float m32, float m33, float xdim,
			float ydim, float zdim) {
		
      
      //super(m00, m01, m02, m23, m10, m11, m12, m13, m20, m21, m22, (float) (m03  / 20f), m30, m31, m32, m33, xdim, ydim, zdim);
      super(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33, xdim, ydim, zdim);
	}

	@Override
	public void drawIt(Canvas c) {

      // ensure box size = image size?!?
      c.translate(0,0,-zdim/2);
      c.stroke(200,200,200);
      c.noFill();
      c.box(this.xdim, this.ydim, 1f);
      
      // PImage a = c.loadImage("/home/lars/tmp/eng2-7f.png");
      // a.filter(PConstants.OPAQUE);
      // c.textureMode(PConstants.NORMALIZED);
      // //c.image(a,-(xdim/2),-(xdim/2),1200,1200);
      // c.rotateX((float) Math.PI);
      // //c.rotateZ((float) (Math.PI + Math.PI/2));
      // c.beginShape();
      // //c.normal(0,0,-1);
      // c.texture(a);

      // float m03 = trafoMatrix[3];
      // float m13 = trafoMatrix[7];

      // c.vertex(-xdim/2, -xdim/2,  0, 0);
      // c.vertex(xdim/2, -xdim/2,  1,  0);
      // c.vertex(xdim/2, xdim/2,  1, 1);
      // c.vertex(-xdim/2, xdim/2, 0, 1);
      // //c.vertex(m03 - xdim/2, m13 - xdim/2,     0,    0);
      // // c.vertex(m03 + xdim/2, m13 - xdim/2,  1200,    0);
      // // c.vertex(m03 + xdim/2, m13 + xdim/2,  1200, 1200);
      // // c.vertex(m03 - xdim/2, m13 + xdim/2,     0, 1200);
      // c.endShape();
      
	}
}

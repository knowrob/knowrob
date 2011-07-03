
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

      c.translate(0,0,-0.2f);
      c.stroke(100,100,100);
      c.strokeWeight((float)1.5);
      c.fill(250,250,250,15);
      c.box(this.xdim, this.ydim, 0.02f);
      
      // PImage a = c.loadImage("/home/lars/tmp/eng2-7f.png");
      // a.filter(PConstants.OPAQUE);
      // c.noStroke();
      // c.textureMode(PConstants.NORMALIZED);
      // c.rotateX((float) Math.PI);
      // c.beginShape();
      // c.texture(a);
      // c.vertex(-xdim/2, -ydim/2, 0,  0, 0);
      // c.vertex( xdim/2, -ydim/2, 0,  1, 0);
      // c.vertex( xdim/2,  ydim/2, 0,  1, 1);
      // c.vertex(-xdim/2,  ydim/2, 0,  0, 1);
      // c.endShape();
      
	}
}

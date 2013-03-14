package edu.tum.cs.ias.knowrob.vis.items;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;

import edu.tum.cs.ias.knowrob.vis.Canvas;

/**
 * TODO This class should probably be rewritten to be based on edu.tum.cs.vis.items.GenericItem (sooner or later), since that class handles transformations applied to the renderer that may change over time much more generically  
 */
public abstract class Item extends ItemBase {
	
	 Item(Matrix4d pose, Vector3d dim){
		if(pose!=null) {
		this.trafoMatrix = new float[] {
				(float) pose.m00, (float) pose.m01, (float) pose.m02, (float) pose.m03,
				(float) pose.m10, (float) pose.m11, (float) pose.m12, (float) pose.m13, 
				(float) pose.m20, (float) pose.m21, (float) pose.m22, (float) pose.m23, 
				(float) pose.m30, (float) pose.m31, (float) pose.m32, (float) pose.m33	};
		}
		if(dim!=null) {
			this.xdim=(float) dim.x;
			this.ydim=(float) dim.y;
			this.zdim=(float) dim.z;
		}
		this.color=defaultColor;
	}
	
	
	public Item(float m00,float m01,float m02,float m03,
			float m10,float m11,float m12,float m13, 
			float m20,float m21,float m22,float m23, 
			float m30,float m31,float m32,float m33, 
			float xdim, float ydim, float zdim){
	
		this.trafoMatrix = new float[] {
			m00, m01, m02, m03,
			m10, m11, m12, m13, 
			m20, m21, m22, m23, 
			m30, m31, m32, m33	};
		this.xdim=xdim;
		this.ydim=ydim;
		this.zdim=zdim;
		
		this.color=defaultColor;
	}
	
	
	public Item(float m00,float m01,float m02,float m03,
				float m10,float m11,float m12,float m13, 
				float m20,float m21,float m22,float m23, 
				float m30,float m31,float m32,float m33, 
				float xdim, float ydim, float zdim, int color){
		
		this( m00,   m01,   m02,  m03,
			  m10,   m11,   m12,  m13, 
			  m20,   m21,   m22,  m23, 
			  m30,   m31,   m32,  m33, 
			  xdim,  ydim,  zdim);

		this.color=color;
	}

	
	
	@Override
	public void draw(Canvas c) {

		c.noStroke();		
		c.pushMatrix();
		
			if(trafoMatrix != null)
				c.applyMatrix(trafoMatrix[0],  trafoMatrix[1],  trafoMatrix[2],  trafoMatrix[3], 
							  trafoMatrix[4],  trafoMatrix[5],  trafoMatrix[6],  trafoMatrix[7], 
							  trafoMatrix[8],  trafoMatrix[9],  trafoMatrix[10], trafoMatrix[11], 
							  trafoMatrix[12], trafoMatrix[13], trafoMatrix[14], trafoMatrix[15]);
			if (drawSettings != null && drawSettings.getOverrideColor() != null)
				c.fill(drawSettings.getOverrideColor().getRed(), drawSettings.getOverrideColor().getGreen(), drawSettings.getOverrideColor().getBlue(), drawSettings.getOverrideColor().getAlpha());
			else
				c.fill(color);
			
			// draw object-local coordinate systems:
//			c.pushMatrix();
//				c.strokeWeight(1.5f);
//				c.stroke(255, 0, 0);      c.line(0, 0, 0,   1, 0, 0);
//				c.stroke(0, 255, 0);      c.line(0, 0, 0,   0, 1, 0); 
//				c.stroke(0, 0, 255);      c.line(0, 0, 0,   0, 0, 1);
//				c.stroke(0, 0, 0);
//			c.popMatrix();C
			
			drawIt(c);
			
		c.popMatrix();
	}
	
	
	public void draw(Canvas c, int step) {
		draw(c);
	}
	
	
	protected abstract void drawIt(Canvas c);
}

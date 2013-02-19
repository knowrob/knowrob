package edu.tum.cs.ias.knowrob.vis.items;

import javax.vecmath.Vector3f;

import edu.tum.cs.ias.knowrob.vis.Canvas;
import edu.tum.cs.ias.knowrob.vis.Drawable;


public class ConePrimitive implements Drawable {

	protected int cone_detail=0;
	protected float[] cylinderX,cylinderZ;
	protected float h, r, r2;
	protected edu.tum.cs.ias.knowrob.util.datastructures.Vector3f v1, v2;

	public ConePrimitive(){
		this(new Vector3f(),new Vector3f(),0,0);
	}

	public ConePrimitive(Vector3f v1, Vector3f v2, float r){
		this(v1,v2,r,r);
	}
	
	public ConePrimitive(Vector3f v1, Vector3f v2, float r, float r2){

		this.v1 = new edu.tum.cs.ias.knowrob.util.datastructures.Vector3f(v1);
		this.v2 = new edu.tum.cs.ias.knowrob.util.datastructures.Vector3f(v2);
		this.h = (float)this.v1.distance(this.v2);
		this.r = r;
		this.r2 = r2;
		coneDetail(30);
	}
	
	public void draw(Canvas c) { 
		
		c.pushMatrix();
		
			c.noStroke();		
			
			// translate to start position v1
			c.translate(v1.x, v1.y, v1.z);
			
			// compute longitudinal cylinder direction 
			Vector3f dir = new Vector3f(v2);
			dir.sub(v1);
			
			// compute rotation axis perpedicular to 'dir' and rotate around it
			edu.tum.cs.ias.knowrob.util.datastructures.Vector3f cp = new edu.tum.cs.ias.knowrob.util.datastructures.Vector3f();
			edu.tum.cs.ias.knowrob.util.datastructures.Vector3f yAxis = new edu.tum.cs.ias.knowrob.util.datastructures.Vector3f(0,1,0);
			cp.cross(dir, yAxis);
			
			if(cp.distance(new edu.tum.cs.ias.knowrob.util.datastructures.Vector3f(0,0,0)) != 0){
				double theta = dir.angle(yAxis);
				c.rotateAxis(theta,cp);
			}
			
			// translate to center
			c.translate(0,h/2,0);
			
			// draw cone
			cone(c,r,r2,h,true,true);
			
		c.popMatrix();
	}


	
	public void cone(Canvas c, Vector3f dir, float r1, float r2, float h, boolean topCap, boolean bottomCap) {

		c.pushMatrix();
			// compute rotation axis perpedicular to 'dir' and rotate around it
			edu.tum.cs.ias.knowrob.util.datastructures.Vector3f cp = new edu.tum.cs.ias.knowrob.util.datastructures.Vector3f();
			edu.tum.cs.ias.knowrob.util.datastructures.Vector3f yAxis = new edu.tum.cs.ias.knowrob.util.datastructures.Vector3f(0,1,0);
			cp.cross(dir, yAxis);
			
			if(cp.distance(new edu.tum.cs.ias.knowrob.util.datastructures.Vector3f(0,0,0)) != 0){
				double theta = dir.angle(yAxis);
				c.rotateAxis(theta,cp);
				
	
//				System.err.println("cp -- th: " + cp.x + ", " + cp.y + ", " + cp.z + " -- " + theta);
			}
			
			// translate to center
//			c.translate(0,h/2,0);
			
			// draw cone
			cone(c,r1,r2,h,true,true);
			
		c.popMatrix();
	}
	
	public void cone(Canvas c, float r1, float r2, float h, boolean topCap, boolean bottomCap) {
		
		if(cone_detail == 0) {
			coneDetail(30);
		}
		
		// draw top cap of the cone
		h *= 0.5;
		if(topCap) {
			c.beginShape(Canvas.TRIANGLE_STRIP);
			for(int i = 0; i < cone_detail; i++) {
				c.vertex(0, -h, 0);
				c.vertex(cylinderX[i] * r1, -h, cylinderZ[i] * r1);
			}
			c.vertex(0, -h, 0);
			c.vertex(cylinderX[0] * r1, -h, cylinderZ[0] * r1);
			c.endShape();
		}
		
		
		c.beginShape(Canvas.TRIANGLE_STRIP);
		for(int i = 0; i < cone_detail; i++) {
			c.vertex(cylinderX[i] * r1, -h, cylinderZ[i] * r1);
			c.vertex(cylinderX[i] * r2, h, cylinderZ[i] * r2);
		}
		c.vertex(cylinderX[0] * r1, -h, cylinderZ[0] * r1);
		c.vertex(cylinderX[0] * r2, h, cylinderZ[0] * r2);
		c.endShape();
		

		// draw bottom cap of the cone
		if(bottomCap) {
			c.beginShape(Canvas.TRIANGLE_STRIP);
			for(int i = 0; i < cone_detail; i++) {
				c.vertex(0, h, 0);
				c.vertex(cylinderX[i] * r2, h, cylinderZ[i] * r2);
			}
			c.vertex(0, h, 0);
			c.vertex(cylinderX[0] * r2, h, cylinderZ[0] * r2);
			c.endShape();
		}
	}

	
	
	protected void coneDetail(int res) {
		
		if(res < 3)
			res = 3; // force minimum res
		
		if(res != cone_detail) {
			
			float delta = (float) Math.PI * 2 / res;//g.SINCOS_LENGTH/res;
			cylinderX = new float[res];
			cylinderZ = new float[res];
			
			// calc unit circle in current resolution in XZ plane
			for(int i = 0; i < res; i++) {
				cylinderX[i] = (float) Math.cos(i * delta);//g.cosLUT[(int) (i*delta) % g.SINCOS_LENGTH];
				cylinderZ[i] = (float) Math.sin(i * delta);//g.sinLUT[(int) (i*delta) % g.SINCOS_LENGTH];
			}
			
			cone_detail = res;
		}
	}

}

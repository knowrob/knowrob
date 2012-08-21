/*
 * Created on Oct 19, 2009
 *
 * TODO To change the template for this generated file go to
 * Window - Preferences - Java - Code Style - Code Templates
 */
package edu.tum.cs.ias.knowrob.vis.items.transform;

import edu.tum.cs.ias.knowrob.vis.Canvas;

public class Pose implements Cloneable, DrawingTransformation {
	public float x, y, z, xRot, yRot, zRot, scale;
	
	public Pose(float x, float y, float z, float xRot, float yRot, float zRot, float scale) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.xRot = xRot;
		this.yRot = yRot;
		this.zRot = zRot;
		this.scale = scale;
	}

	public void applyTransformation(Canvas c) {
		c.translate(x, y, z);
		c.scale(scale);
		if(xRot != 0) c.rotateX(xRot);
		if(yRot != 0) c.rotateY(yRot); 
		if(zRot != 0) c.rotateZ(zRot);
	}
	
	public boolean equals(Pose o) {
		return x==o.x && y==o.y && z==o.z && xRot==o.xRot && yRot==o.yRot && zRot==o.zRot && scale==o.scale;
	}
	
	@Override
	public Pose clone() {
		return new Pose(x,y,z,xRot,yRot,zRot,scale);
	}
}

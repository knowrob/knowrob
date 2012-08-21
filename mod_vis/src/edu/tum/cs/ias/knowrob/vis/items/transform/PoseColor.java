/*
 * Created on Oct 19, 2009
 *
 * TODO To change the template for this generated file go to
 * Window - Preferences - Java - Code Style - Code Templates
 */
package edu.tum.cs.ias.knowrob.vis.items.transform;

import edu.tum.cs.ias.knowrob.vis.Canvas;

/**
 * 
 * @author jain
 */
public class PoseColor implements Cloneable, DrawingTransformation {
	
	public Pose pose;
	public int color;

	public PoseColor(float x, float y, float z, float rot, float rot2,
			float rot3, float scale, int color) {
		pose = new Pose(x, y, z, rot, rot2, rot3, scale);
		this.color = color;		
	}
	
	public PoseColor(Pose pose, int color) {
		this.pose = pose;
		this.color = color;
	}
	
	public void applyTransformation(Canvas c) {
		pose.applyTransformation(c);
		c.fill(color);
	}
	
	public boolean equals(PoseColor o) {
		return pose.equals(o.pose) && color==o.color; 
	}
	
	public PoseColor clone() {
		return new PoseColor(pose.clone(), color);
	}
}

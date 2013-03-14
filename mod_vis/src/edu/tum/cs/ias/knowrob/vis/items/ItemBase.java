package edu.tum.cs.ias.knowrob.vis.items;

import java.awt.Color;

import javax.vecmath.Matrix4d;

import edu.tum.cs.ias.knowrob.vis.Canvas;
import edu.tum.cs.ias.knowrob.vis.Drawable;
import edu.tum.cs.ias.knowrob.vis.DrawableAnimated;
import edu.tum.cs.ias.knowrob.vis.applets.SemanticMapVisApplet;
import edu.tum.cs.vis.model.util.DrawSettings;

/**
 * Abstract base class for items (animated or not) that have a default color and may have
 * a general transformation matrix applied to them   
 * 
 */
public abstract class ItemBase implements Drawable, DrawableAnimated {
	public String name;
	/**
	 * FIXME It is bad practice to use a static method of a class that is completely unrelated to this class. It destroys modularity. The static method convertColor should probably be moved here or some place else that is neutral.
	 */
	public int defaultColor = SemanticMapVisApplet.convertColor(220, 220, 220, 255);
	protected DrawSettings drawSettings = null;
	
	public void setOverrideColor(Color overrideColor) {
		if (drawSettings == null)
			drawSettings = new DrawSettings();
		drawSettings.setOverrideColor(overrideColor);
	}
	
	/**
	 * a transformation matrix to apply before drawing
	 * TODO should describe ordering (rows, columns).
	 */
	protected float[] trafoMatrix = null;
	
	protected float xdim;
	protected float ydim;
	protected float zdim;
	
	protected int color;
	
	public void draw(Canvas c) {
		draw(c,0);
	}
	
	public int getMaxStep() {
		return -1;
	}		
	public static int convertColor(int red, int green, int blue, int alpha) {
		return (((((alpha << 8) + red) << 8) + green) << 8) + blue;
	}
	
	public void setColor(int color){
		this.color = color;
	}

	public void setColor(int color,int start, int end){
		this.color=color;
	}

	public void setPose(Matrix4d pose) {
		if(pose!=null) {
			this.trafoMatrix = new float[] {
					(float)pose.m00, (float)pose.m01, (float)pose.m02, (float)pose.m03,
					(float)pose.m10, (float)pose.m11, (float)pose.m12, (float)pose.m13, 
					(float)pose.m20, (float)pose.m21, (float)pose.m22, (float)pose.m23, 
					(float)pose.m30, (float)pose.m31, (float)pose.m32, (float)pose.m33	};
		}
	}
	
	public void setPose(double[] pose) {
		this.setPose(new Matrix4d(pose));
	}
	
	public void setPose(float[] pose) {
		this.trafoMatrix = pose;
	}
	
	public void setPose(float m00,float m01,float m02,float m03,
						float m10,float m11,float m12,float m13, 
						float m20,float m21,float m22,float m23, 
						float m30,float m31,float m32,float m33) {
		this.trafoMatrix = new float[] {
				m00, m01, m02, m03,
				m10, m11, m12, m13, 
				m20, m21, m22, m23, 
				m30, m31, m32, m33	};
	}
	
	public void setPosition(float x, float y, float z) {
		this.trafoMatrix[3]=x;
		this.trafoMatrix[7]=y;
		this.trafoMatrix[11]=z;
	}
	
	public void setDimensions(float xdim, float ydim, float zdim) {
		this.xdim=xdim;
		this.ydim=ydim;
		this.zdim=zdim;
	}
}

/*
 * Created on Oct 19, 2009
 *
 * TODO To change the template for this generated file go to
 * Window - Preferences - Java - Code Style - Code Templates
 */
package edu.tum.cs.ias.knowrob.vis.items;

import edu.tum.cs.ias.knowrob.vis.items.transform.PoseColor;

/**
 * 
 * @author jain
 */
public abstract class GenericPoseColorItem extends GenericItem<PoseColor> {

	public GenericPoseColorItem(PoseColor transform) {
		super(transform);
	}
	
	public GenericPoseColorItem(float x, float y, float z, int color) {
		this(new PoseColor(x, y, z, 0.0f, 0.0f, 0.0f, 1.0f, color));
	}

	/**
	 * sets the position for one frame, leaving rotation, scale, color unchanged.
	 */
	public void setPosition(int frame, float x, float y, float z) {
		setPosition(frame, frame, x, y, z);
	}

	/**
	 * sets the position for several frames. leaves rotation, scale, color unchanged;
	 * @param lastFrame last frame to be changed; -1 for "until the end".
	 */
	public void setPosition(int firstFrame, int lastFrame, float x, float y, float z) {
		PoseColor replacement = new PoseColor(x,y,z,0,0,0,0,0); 
		setFrames(firstFrame, lastFrame, new PartialTransformationReplacer<PoseColor>(replacement) {
		
			@Override
			public PoseColor replace(PoseColor before) {
				return new PoseColor(replacement.pose.x, replacement.pose.y, replacement.pose.z, before.pose.xRot, before.pose.yRot, before.pose.zRot, before.pose.scale, before.color);				
			}
			
		});
	}
	
	/**
	 * sets position, rotation and scale (i.e. the entire pose) for one frame.
	 */
	public void setPose(int frame, float x, float y, float z, float rotX, float rotY, float rotZ, float scale) {
		setPose(frame, frame, x, y, z, rotX, rotY, rotZ, scale);		
	}
	
	
	/**
	 * sets the position, rotation and scale (i.e. the entire pose) for several frames
	 */
	public void setPose(int firstFrame, int lastFrame, float x, float y, float z, float rotX, float rotY, float rotZ, float scale) {
		PoseColor replacement = new PoseColor(x,y,z,rotX,rotY,rotZ,scale,0); 
		setFrames(firstFrame, lastFrame, new PartialTransformationReplacer<PoseColor>(replacement) {
		
			@Override
			public PoseColor replace(PoseColor before) {
				return new PoseColor(replacement.pose, before.color);
			}
			
		});
	}
	
	/**
	 * replaces the color (in all frames)
	 * @param color
	 */
	public void setColor(int color) {
		setColor(0, animation.size()-1, color);
	}
	
	/**
	 * replaces the color in the given sequence of frames
	 * @param color
	 */
	public void setColor(int from, int to, int color) {
		PoseColor replacement = new PoseColor(null, color); 
		setFrames(from, to, new PartialTransformationReplacer<PoseColor>(replacement) {
		
			@Override
			public PoseColor replace(PoseColor before) {
				return new PoseColor(before.pose, replacement.color);
			}
			
		});
	}
}

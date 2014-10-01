package org.knowrob.vis;

import org.knowrob.tfmemory.TFMemory;
import org.ros.message.Time;

import tfjava.StampedTransform;

/**
 * Represents a joint in the skeleton of a human.
 * 
 * @author danielb@cs.uni-bremen.de
 */
public class HumanJoint {
	// TODO: Make this configurable ?
	private static final String targetFrame = "/map";
	
	private final String sourceFrame;
	
	private final String connections[];
	
	private final double size;
	
	private StampedTransform lastTransform = null;
	
	private Time lastTime = new Time();
	
	public HumanJoint(String sourceFrame, String[] connections, double size) {
		super();
		this.sourceFrame = sourceFrame;
		this.connections = connections;
		this.size = size;
	}

	/**
	 * The joint name as used in the TF tree.
	 */
	public String getSourceFrame() {
		return sourceFrame;
	}

	/**
	 * List of joints with connections to this joint.
	 * Connections are represented unidirectional.
	 */
	public String[] getConnections() {
		return connections;
	}

	/**
	 * Size hint for rendering.
	 */
	public double getSize() {
		return size;
	}
	
	/**
	 * Lookup pose for user specified point in time.
	 */
	public StampedTransform lookupPose(Time time) {
		// Use last transform if time point didn't changed
		// since last call
		
		if(!lastTime.equals(time)) {
			lastTime = time;
			lastTransform = TFMemory.getInstance().lookupTransform(targetFrame, sourceFrame, time);
		}
		
		return lastTransform;
	}
}

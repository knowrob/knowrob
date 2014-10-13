package org.knowrob.vis;

import java.util.HashMap;

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
	
	/**
	 * Instantiates HumanJoint's based on XSens TF format
	 * (i.e. the skeletal structure captured by the XSens suite).
	 */
	public static HashMap<String,HumanJoint> initializeHumanJoints() {
		HashMap<String,HumanJoint> out = new HashMap<String,HumanJoint>();
		// TODO: Don't restrict to XSens data format.
		// Make the skeletal structure configurable.
		addHumanJoint(out,"/Head", "/Neck", 0.2);
		addHumanJoint(out,"/Neck", 0.1);
		addHumanJoint(out,"/RightShoulder", "/RightUpperArm", 0.1);
		addHumanJoint(out,"/LeftShoulder", "/LeftUpperArm", 0.1);
		addHumanJoint(out,"/RightUpperArm", "/RightForearm", 0.1);
		addHumanJoint(out,"/LeftUpperArm", "/LeftForearm", 0.1);
		addHumanJoint(out,"/RightForearm", "/RightHand", 0.1);
		addHumanJoint(out,"/LeftForearm", "/LeftHand", 0.1);
		addHumanJoint(out,"/RightHand", 0.15);
		addHumanJoint(out,"/LeftHand", 0.15);
		addHumanJoint(out,"/T8", new String[]{"/RightShoulder","/LeftShoulder"}, 0.1);
		addHumanJoint(out,"/T12", "/T8", 0.1);
		addHumanJoint(out,"/L3", "/T12", 0.1);
		addHumanJoint(out,"/L5", "/L3", 0.1);
		addHumanJoint(out,"/Pelvis", new String[]{"/RightUpperLeg","/LeftUpperLeg","/L5"}, 0.2);
		addHumanJoint(out,"/RightUpperLeg", "/RightLowerLeg", 0.1);
		addHumanJoint(out,"/LeftUpperLeg", "/LeftLowerLeg", 0.1);
		addHumanJoint(out,"/RightLowerLeg", "/RightFoot", 0.1);
		addHumanJoint(out,"/LeftLowerLeg", "/LeftFoot", 0.1);
		addHumanJoint(out,"/RightFoot", "/RightToe", 0.1);
		addHumanJoint(out,"/LeftFoot", "/LeftToe", 0.1);
		addHumanJoint(out,"/RightToe", 0.1);
		addHumanJoint(out,"/LeftToe", 0.1);
		return out;
	}
	private static void addHumanJoint(HashMap<String,HumanJoint> out, String frame, double size) {
		addHumanJoint(out,frame, new String[]{}, size);
	}
	private static void addHumanJoint(HashMap<String,HumanJoint> out, String frame, String connection, double size) {
		addHumanJoint(out,frame, new String[]{connection}, size);
	}
	private static void addHumanJoint(HashMap<String,HumanJoint> out, String frame, String connections[], double size) {
		out.put(frame, new HumanJoint(frame, connections, size));
	}
	
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
	 * @param prefix 
	 */
	public StampedTransform lookupPose(Time time, String prefix) {
		// Use last transform if time point didn't changed
		// since last call
		
		if(!lastTime.equals(time)) {
			lastTime = time;
			lastTransform = TFMemory.getInstance().lookupTransform(targetFrame, prefix+sourceFrame, time);
		}
		
		return lastTransform;
	}
}

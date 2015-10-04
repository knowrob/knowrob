package org.knowrob.vis;

import geometry_msgs.Pose;
import geometry_msgs.Vector3;

import java.util.LinkedList;
import java.util.List;

import org.ros.message.Duration;
import org.ros.message.Time;

import std_msgs.ColorRGBA;
import visualization_msgs.Marker;

public class MarkerObject {
	private final String identifier;
	private final Marker markerMsg;
	private final MarkerPublisher publisher;
	
	private final List<MarkerObject> children = new LinkedList<MarkerObject>();
	private boolean hasVisual = true;
	
	public MarkerObject(String identifier, Marker markerMsg, MarkerPublisher publisher) {
		this.markerMsg = markerMsg;
		this.publisher = publisher;
		this.identifier = identifier;
	}

	public String getIdentifier() {
		return identifier;
	}
	
	public void clear() {
		for(MarkerObject child : children) {
			publisher.eraseMarker(child);
		}
		children.clear();
	}
	
	public MarkerObject createMarker(String identifier) {
		MarkerObject obj = publisher.createMarker(identifier);
		obj.setType(getType());
		obj.setScale(getScale());
		obj.setColor(getColor());
		children.add(obj);
		return obj;
	}
	
	public void setHasVisual(boolean value) {
		hasVisual  = value;
	}

	public boolean hasVisual() {
		return this.hasVisual;
	}
	
	public String[] getChildrenNames() {
		String out[] = new String[children.size()];
		int i = 0;
		for(MarkerObject child : children) {
			out[i] = child.identifier;
			i += 1;
		}
		return out;
	}
	
	
	public int getType() {
		return markerMsg.getType();
	}
	
	public void setType(int value) {
		markerMsg.setType(value);
	}
	
	public String getMeshResource() {
		return markerMsg.getMeshResource();
	}
	
	public void setMeshResource(String value) {
		markerMsg.setMeshResource(value);
	}
	
	public String getText() {
		return markerMsg.getText();
	}
	
	public void setText(String value) {
		markerMsg.setText(value);
	}
	
	public Vector3 getScale() {
		return markerMsg.getScale();
	}
	
	public void setScale(Vector3 value) {
		markerMsg.getScale().setX(value.getX());
		markerMsg.getScale().setY(value.getY());
		markerMsg.getScale().setZ(value.getZ());
		for(MarkerObject child : children) child.setScale(value);
	}

	public ColorRGBA getColor() {
		return markerMsg.getColor();
	}
	
	public void setColor(ColorRGBA value) {
		markerMsg.getColor().setA(value.getA());
		markerMsg.getColor().setR(value.getR());
		markerMsg.getColor().setG(value.getG());
		markerMsg.getColor().setB(value.getB());
		for(MarkerObject child : children) child.setColor(value);
	}
	
	public Pose getPose() {
		return markerMsg.getPose();
	}

	public Marker getMessage() {
		return markerMsg;
	}
	
	public double getTimestamp() {
		return markerMsg.getHeader().getStamp().totalNsecs()/1000.0;
	}
	
	public void setTimestamp(double posix_ts) {
		final Time time = new Time();
		time.secs = (int)posix_ts;
		time.nsecs = (int) (1E9 * (posix_ts - ((int) posix_ts)));
		markerMsg.getHeader().setStamp(time);
	}
	
	public double getLifetime() {
		return markerMsg.getLifetime().totalNsecs()/1000.0;
	}
	
	public void setLifetime(double value) {
		markerMsg.setLifetime(new Duration(value));
		for(MarkerObject child : children) child.setLifetime(value);
	}
}

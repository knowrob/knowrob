package org.knowrob.vis;

import java.util.LinkedList;
import java.util.List;

import org.ros.message.Duration;
import org.ros.message.Time;

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
		hasVisual = value;
		if(value) {
			queueRepublish();
		}
		else {
			unqueueRepublish();
		}
	}
	
	public boolean getHasVisual() {
		return hasVisual;
	}

	public boolean hasVisual() {
		return this.hasVisual;
	}
	
	public MarkerObject[] getChildren() {
		return children.toArray(new MarkerObject[children.size()]);
	}
	
	
	public int getType() {
		return markerMsg.getType();
	}
	
	public void setType(int value) {
		markerMsg.setType(value);
		queueRepublish();
	}
	
	public String getMeshResource() {
		return markerMsg.getMeshResource();
	}
	
	public void setMeshResource(String value) {
		markerMsg.setMeshResource(value);
		queueRepublish();
	}
	
	public String getText() {
		return markerMsg.getText();
	}
	
	public void setText(String value) {
		markerMsg.setText(value);
		queueRepublish();
	}
	
	public double[] getScale() {
		return new double[] {
			markerMsg.getScale().getX(),
			markerMsg.getScale().getY(),
			markerMsg.getScale().getZ()
		};
	}
	
	public void setScale(float value[]) {
		markerMsg.getScale().setX((double)value[0]);
		markerMsg.getScale().setY((double)value[1]);
		markerMsg.getScale().setZ((double)value[2]);
		for(MarkerObject child : children) child.setScale(value);
		queueRepublish();
	}
	
	public void setScale(double value[]) {
		markerMsg.getScale().setX(value[0]);
		markerMsg.getScale().setY(value[1]);
		markerMsg.getScale().setZ(value[2]);
		for(MarkerObject child : children) child.setScale(value);
		queueRepublish();
	}

	public float[] getColor() {
		return new float[] {
				markerMsg.getColor().getR(),
				markerMsg.getColor().getG(),
				markerMsg.getColor().getB(),
				markerMsg.getColor().getA()
			};
	}
	
	public void setColor(float value[]) {
		markerMsg.getColor().setR(value[0]);
		markerMsg.getColor().setG(value[1]);
		markerMsg.getColor().setB(value[2]);
		markerMsg.getColor().setA(value[3]);
		for(MarkerObject child : children) child.setColor(value);
		queueRepublish();
	}
	
	public void setColor(double value[]) {
		markerMsg.getColor().setR((float)value[0]);
		markerMsg.getColor().setG((float)value[1]);
		markerMsg.getColor().setB((float)value[2]);
		markerMsg.getColor().setA((float)value[3]);
		for(MarkerObject child : children) child.setColor(value);
		queueRepublish();
	}

	public float getAlpha() {
		return markerMsg.getColor().getA();
	}
	
	public void setAlpha(float value) {
		markerMsg.getColor().setA(value);
		for(MarkerObject child : children) child.setAlpha(value);
		queueRepublish();
	}
	
	public void setAlpha(double value) {
		markerMsg.getColor().setA((float)value);
		for(MarkerObject child : children) child.setAlpha(value);
		queueRepublish();
	}
	
	public double[] getTranslation() {
		return new double[] {
			markerMsg.getPose().getPosition().getX(),
			markerMsg.getPose().getPosition().getY(),
			markerMsg.getPose().getPosition().getZ()
		};
	}
	
	public double[] getOrientation() {
		return new double[] {
			markerMsg.getPose().getOrientation().getW(),
			markerMsg.getPose().getOrientation().getX(),
			markerMsg.getPose().getOrientation().getY(),
			markerMsg.getPose().getOrientation().getZ()
		};
	}
	
	public void setTranslation(double []translation) {
		markerMsg.getPose().getPosition().setX(translation[0]);
		markerMsg.getPose().getPosition().setY(translation[1]);
		markerMsg.getPose().getPosition().setZ(translation[2]);
		queueRepublish();
	}
	
	public void setOrientation(double []orientation) {
		markerMsg.getPose().getOrientation().setW(orientation[0]);
		markerMsg.getPose().getOrientation().setX(orientation[1]);
		markerMsg.getPose().getOrientation().setY(orientation[2]);
		markerMsg.getPose().getOrientation().setZ(orientation[3]);
		queueRepublish();
	}
	
	public void setTranslation(float []translation) {
		markerMsg.getPose().getPosition().setX(translation[0]);
		markerMsg.getPose().getPosition().setY(translation[1]);
		markerMsg.getPose().getPosition().setZ(translation[2]);
		queueRepublish();
	}
	
	public void setOrientation(float []orientation) {
		markerMsg.getPose().getOrientation().setW(orientation[0]);
		markerMsg.getPose().getOrientation().setX(orientation[1]);
		markerMsg.getPose().getOrientation().setY(orientation[2]);
		markerMsg.getPose().getOrientation().setZ(orientation[3]);
		queueRepublish();
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
		queueRepublish();
	}

	private void queueRepublish() {
		publisher.queueRepublish(this);
	}

	private void unqueueRepublish() {
		publisher.unqueueRepublish(this);
	}
}

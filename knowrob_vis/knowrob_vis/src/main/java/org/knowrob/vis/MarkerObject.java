package org.knowrob.vis;

import java.util.LinkedList;
import java.util.List;

import org.ros.message.Duration;
import org.ros.message.Time;

import visualization_msgs.Marker;

/**
 * Wrapper class for marker messages.
 * MarkerObject's have an identifier, a list of child markers
 * and provide an interface to getter/setter methods of the message.
 * Note: JPL has a bug using the getter/setter methods of the message in Prolog
 * (see: https://groups.google.com/forum/#!topic/swi-prolog/0ZH-eTDadh8),
 * using this wrapper class this problem can be avoided.
 * 
 * @author Daniel Beßler
 */
public class MarkerObject {
	private final String identifier;
	private final Marker markerMsg;
	private final MarkerPublisher publisher;
	
	private final List<MarkerObject> children = new LinkedList<MarkerObject>();
	
	private float[] highlighted;
	private String tfPrefix = "/";
	
	boolean isHidden = false;
	boolean hasVisual = true;
	boolean isPublished = false;
	
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
		obj.setTfPrefix(tfPrefix);
		if(!children.contains(obj)) children.add(obj);
		return obj;
	}
	
	public void hide() {
		setHidden(true);
	}
	
	public void show() {
		setHidden(false);
	}
	
	public void setHidden(boolean value) {
		if(isHidden==value) return;
		
		isHidden  = value;
		if(value) {
			unqueueRepublish();
			
			if(isPublished) {
				markerMsg.setAction(Marker.DELETE);
				publisher.markers.put(identifier, this);
			}
		}
		else {
			queueRepublish();
		}
	}

	public boolean isHidden() {
		return isHidden;
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
	
	public String getTfPrefix() {
		return tfPrefix;
	}
	
	public void setTfPrefix(String value) {
		this.tfPrefix  = value;
		for(MarkerObject child : children) child.setTfPrefix(value);
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
		queueRepublish();
	}
	
	public void setScale(double value[]) {
		markerMsg.getScale().setX(value[0]);
		markerMsg.getScale().setY(value[1]);
		markerMsg.getScale().setZ(value[2]);
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
		queueRepublish();
	}
	
	public void setColor(double value[]) {
		markerMsg.getColor().setR((float)value[0]);
		markerMsg.getColor().setG((float)value[1]);
		markerMsg.getColor().setB((float)value[2]);
		markerMsg.getColor().setA((float)value[3]);
		queueRepublish();
	}

	public float getAlpha() {
		return markerMsg.getColor().getA();
	}
	
	public void setAlpha(float value) {
		markerMsg.getColor().setA(value);
		queueRepublish();
	}
	
	public void setAlpha(double value) {
		markerMsg.getColor().setA((float)value);
		queueRepublish();
	}
	
	public double[][] getPoints() {
		List<geometry_msgs.Point> points = markerMsg.getPoints();
		if(points == null) return null;
		double[][] out = new double[points.size()][3];
		int i = 0;
		for(geometry_msgs.Point p : points) {
			out[i] = new double[3];
			out[i][0] = p.getX();
			out[i][1] = p.getY();
			out[i][2] = p.getZ();
			i += 1;
		}
		return out;
	}
	
	public void setPoints(double[][] in) {
		List<geometry_msgs.Point> points = new LinkedList<geometry_msgs.Point>();
		for(int i=0; i<points.size(); ++i) {
			geometry_msgs.Point p = publisher.node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
			p.setX(in[i][0]);
			p.setY(in[i][1]);
			p.setZ(in[i][2]);
			points.add(p);
		}
		markerMsg.setPoints(points);
	}
	
	public float[][] getColors() {
		List<std_msgs.ColorRGBA> colors = markerMsg.getColors();
		if(colors == null) return null;
		float[][] out = new float[colors.size()][4];
		int i = 0;
		for(std_msgs.ColorRGBA c : colors) {
			out[i] = new float[4];
			out[i][0] = c.getR();
			out[i][1] = c.getG();
			out[i][2] = c.getB();
			out[i][3] = c.getA();
			i += 1;
		}
		return out;
	}
	
	public void setColors(float[][] in) {
		List<std_msgs.ColorRGBA> colors = new LinkedList<std_msgs.ColorRGBA>();
		for(int i=0; i<colors.size(); ++i) {
			std_msgs.ColorRGBA c = publisher.node.getTopicMessageFactory().newFromType(std_msgs.ColorRGBA._TYPE);
			c.setR(in[i][0]);
			c.setG(in[i][1]);
			c.setB(in[i][2]);
			c.setA(in[i][3]);
			colors.add(c);
		}
		markerMsg.setColors(colors);
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
			markerMsg.getPose().getOrientation().getX(),
			markerMsg.getPose().getOrientation().getY(),
			markerMsg.getPose().getOrientation().getZ(),
			markerMsg.getPose().getOrientation().getW()
		};
	}
	
	public void setTranslation(double []translation) {
		markerMsg.getPose().getPosition().setX(translation[0]);
		markerMsg.getPose().getPosition().setY(translation[1]);
		markerMsg.getPose().getPosition().setZ(translation[2]);
		queueRepublish();
	}
	
	public void setOrientation(double []orientation) {
		markerMsg.getPose().getOrientation().setX(orientation[0]);
		markerMsg.getPose().getOrientation().setY(orientation[1]);
		markerMsg.getPose().getOrientation().setZ(orientation[2]);
		markerMsg.getPose().getOrientation().setW(orientation[3]);
		queueRepublish();
	}
	
	public void setTranslation(float []translation) {
		markerMsg.getPose().getPosition().setX(translation[0]);
		markerMsg.getPose().getPosition().setY(translation[1]);
		markerMsg.getPose().getPosition().setZ(translation[2]);
		queueRepublish();
	}
	
	public void setOrientation(float []orientation) {
		markerMsg.getPose().getOrientation().setX(orientation[0]);
		markerMsg.getPose().getOrientation().setY(orientation[1]);
		markerMsg.getPose().getOrientation().setZ(orientation[2]);
		markerMsg.getPose().getOrientation().setW(orientation[3]);
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

	public void highlight(String color) {
		highlight(Integer.valueOf(color, 16));
	}

	public void highlight(int col) {
		// TODO: What about the alpha channel?
		int r = (col & 0xff0000) >> 16;
		int g = (col & 0x00ff00) >> 8;
		int b = (col & 0x0000ff);
		highlight(new float[] {
			r/255.0f, g/255.0f, b/255.0f, 1.0f
		});
	}
	
	public void highlight(float color[]) {
		if(highlighted==null) {
			highlighted = getColor();
		}
		markerMsg.getColor().setR(color[0]);
		markerMsg.getColor().setG(color[1]);
		markerMsg.getColor().setB(color[2]);
		markerMsg.getColor().setA(color[3]);
		queueRepublish();
	}
	
	public void removeHighlight() {
		if(highlighted != null) {
			setColor(highlighted);
			highlighted = null;
		}
	}
	
	public boolean hasHighlight() {
		return highlighted != null;
	}

	private void queueRepublish() {
		if(isHidden)
			markerMsg.setAction(Marker.DELETE);
		else if(isPublished)
			markerMsg.setAction(Marker.MODIFY);
		else
			markerMsg.setAction(Marker.ADD);
		
		publisher.queueRepublish(this);
	}

	private void unqueueRepublish() {
		publisher.unqueueRepublish(this);
	}
}

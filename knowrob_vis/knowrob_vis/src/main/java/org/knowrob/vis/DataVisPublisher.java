package org.knowrob.vis;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import org.apache.commons.logging.Log;
import org.ros.message.Duration;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import data_vis_msgs.DataVis;

/**
 * Visualization marker node. Used for generating and publishing marker messages
 * on the topic "/visualization_marker_array".
 * The messages can be displayed by visualization tools such as openEASE or RViz.
 * 
 * @author Daniel Beßler
 */
public class DataVisPublisher extends AbstractNodeMain {
	/**
	 * The ROS message publisher if connected, else null
	 */
	private Publisher<DataVis> pub = null;
	/**
	 * The ROS node if connected, else null
	 */
	protected ConnectedNode node = null;
	/**
	 * Logger of ROS node if connected, else null
	 */
	private Log log = null;
	
	/**
	 * Store the data vis messages to be published
	 */
	protected Map<String, DataVisObject> dirtyObjects;
	protected Map<String, DataVisObject> objects;
	
	/**
	 * Counter for IDs
	 */
	private static int id = 0;

	private static DataVisPublisher instance = null;
	
	public static DataVisPublisher get() {
		if(instance==null) instance = new DataVisPublisher();
		return instance;
	}

	private DataVisPublisher() {
		objects =  new ConcurrentHashMap<String, DataVisObject>(8, 0.9f, 1);
		dirtyObjects =  new ConcurrentHashMap<String, DataVisObject>(8, 0.9f, 1);
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		node = connectedNode;
		pub = connectedNode.newPublisher("/data_vis_msgs", data_vis_msgs.DataVis._TYPE);
		log = connectedNode.getLog();
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("data_vis_publisher");
	}
	
	public DataVisObject createDataVisObject(String identifier) {
		waitForNode();
		DataVisObject o = objects.get(identifier);
		if(o!=null) return o;

		DataVis data = node.getTopicMessageFactory().newFromType(data_vis_msgs.DataVis._TYPE);
		data.setId(identifier);
		data.setTitle(identifier);
		data.setFontsize("9px");
		data.setXlabel("");
		data.setYlabel("");
		data.setWidth(250);
		data.setHeight(250);
		data.setType(0);
		
		o = new DataVisObject(identifier, data, this);
		synchronized (objects) { objects.put(identifier, o); }
		synchronized (dirtyObjects) { dirtyObjects.put(identifier, o); }
		
		return o;
	}
	
	public void removeDataVisObject(String identifier) {
		DataVis data =  node.getTopicMessageFactory().newFromType(data_vis_msgs.DataVis._TYPE);
		data.setId(identifier);
		// publish empty message
		pub.publish(data);
		// remove the object from the list		
		synchronized (objects) { objects.remove(identifier); }
	}
	
	public void removeDataVisObjects() {
		for(String identifier : objects.keySet()) {
			removeDataVisObject(identifier);
		}
	}
	
	public void publish() {
		try {
			waitForNode();
			synchronized (dirtyObjects) {
				for(DataVisObject o : dirtyObjects.values()) {
					pub.publish(o.getMessage());
				}
				dirtyObjects.clear();
			}
		}
		catch (Exception exc) {
			log.error("Failed to publish marker.", exc);
		}
	}

	public void queueRepublish(DataVisObject o) {
		synchronized (objects) {
			dirtyObjects.put(o.getIdentifier(), o);
		}
	}

	public void unqueueRepublish(DataVisObject o) {
		synchronized (objects) {
			dirtyObjects.remove(o.getIdentifier());
		}
	}
	
	public ConnectedNode getNode(){
		waitForNode();
		return this.node;
	}

	private void waitForNode() {
		try {
			while(node == null || pub == null) Thread.sleep(200);
		}
		catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
}

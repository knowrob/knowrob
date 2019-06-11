package org.knowrob.interfaces.mongo;

import java.util.HashMap;
import java.util.Map;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

import com.mongodb.BasicDBObject;

import org.apache.log4j.Logger;

/**
 * Factory singleton that allows to generate and publish ROS messages from Mongo DB objects.
 * @author Daniel Beßler
 */
public class MongoMessages extends AbstractNodeMain {
	final static Logger logger = Logger.getLogger(MongoMessages.class);
	
	private ConnectedNode node = null;
	private Map<String,MongoPublisher<?>> publisher = new HashMap<String,MongoPublisher<?>>();
	private static MongoMessages instance = null;
	
	public static MongoMessages get() {
		if(instance==null) {
			instance = new MongoMessages();
		}
		return instance;
	}
	
	private MongoMessages() {}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		this.node  = connectedNode;
	}
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("knowrob_mongo/ros_messages");
	}
	
	/**
	 * Generate a ROS message for given BasicDBObject.
	 * @param mngObj The mongo DB object (result of a query)
	 * @param cls The ROS message class (e.g., sensor_msgs.Image.class)
	 * @param type The message type (e.g., sensor_msgs/Image)
	 * @return The ROS message created
	 * @throws InterruptedException
	 */
	public <T> T create(BasicDBObject mngObj, Class<T> cls, String type) {
		MongoPublisher<T> publisher = getPublisher(type+"_factory", type);
		return publisher.create(mngObj);
	}

	/**
	 * Generate and publish a ROS message for given BasicDBObject.
	 * @param mngObj The mongo DB object (result of a query)
	 * @param cls The ROS message class (e.g., sensor_msgs.Image.class)
	 * @param type The message type (e.g., sensor_msgs/Image)
	 * @param topic The topic that should be used for publishing the message
	 * @return The ROS message created
	 * @throws InterruptedException
	 */
	public <T> T publish(BasicDBObject mngObj, Class<T> cls, String type, String topic) {
		MongoPublisher<T> publisher = getPublisher(topic, type);
		return publisher.publish(mngObj);
	}
	
	@SuppressWarnings("unchecked")
	private <T> MongoPublisher<T> getPublisher(String topic, String type) {
		// wait for node to be ready
		try {
			while(node == null) Thread.sleep(200);
		}
		catch (InterruptedException e) {
			// TODO Auto-generated catch block
			logger.error("mongo getPublisher failed: " + e.getMessage());
			e.printStackTrace();
		}
		
		if(!publisher.containsKey(topic)) {
			publisher.put(topic, new MongoPublisher<T>(this.node, topic, type));
		}
		return (MongoPublisher<T>)publisher.get(topic);
	}
}

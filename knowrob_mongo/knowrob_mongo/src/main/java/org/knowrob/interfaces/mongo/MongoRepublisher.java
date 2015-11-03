package org.knowrob.interfaces.mongo;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

import com.mongodb.BasicDBObject;

public class MongoRepublisher extends AbstractNodeMain {
	private ConnectedNode node = null;

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		this.node  = connectedNode;
	}
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("knowrob_mongo/ros_messages");
	}
	
	public sensor_msgs.Image getImageMessage(BasicDBObject mngObj) {
		waitForNode();
		return new MongoRosMessage<sensor_msgs.Image>(this.node,
				"sensor_msgs/Image", "logged_images").createMessgae(mngObj);
	}
	
	private void waitForNode() {
		// wait for node to be ready
		try {
			while(node == null) Thread.sleep(200);
		}
		catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
}

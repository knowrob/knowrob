package org.knowrob.interfaces.mongo.types;

import geometry_msgs.Pose;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;


import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;


public class PoseFactory {
	
	/**
	 * Read from a MongoDB result object
	 * 
	 * @param row BasicDBObject, e.g. result of a MongoDB query
	 * @return Instance of Pose with values from row
	 */
	public static Pose readFromDBObject(DBObject row) {	
		
		NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
		MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
		
		Pose p = messageFactory.newFromType(geometry_msgs.Pose._TYPE);
		
		p.getPosition().setX(((BasicDBObject) row.get("position")).getDouble("x"));
		p.getPosition().setY(((BasicDBObject) row.get("position")).getDouble("y"));
		p.getPosition().setZ(((BasicDBObject) row.get("position")).getDouble("z"));
		
		p.getOrientation().setX(((BasicDBObject) row.get("orientation")).getDouble("x"));
		p.getOrientation().setY(((BasicDBObject) row.get("orientation")).getDouble("y"));
		p.getOrientation().setZ(((BasicDBObject) row.get("orientation")).getDouble("z"));
		p.getOrientation().setW(((BasicDBObject) row.get("orientation")).getDouble("w"));
		
		return p;
	}
	
	/**
	 * Transform pose to matrix representation 
	 * 
	 * @return Matrix4d representing this pose
	 */
	public static Matrix4d toMatrix4d(Pose p) {
		
		Quat4d q = new Quat4d(p.getOrientation().getX(), 
							  p.getOrientation().getY(),
							  p.getOrientation().getZ(),
							  p.getOrientation().getW());
		
		Vector3d t = new Vector3d(p.getPosition().getX(),
								  p.getPosition().getY(),
								  p.getPosition().getZ());
		
		return new Matrix4d(q, t, 1);
	}
	
}

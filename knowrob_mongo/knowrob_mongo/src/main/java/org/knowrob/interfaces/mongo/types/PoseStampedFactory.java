package org.knowrob.interfaces.mongo.types;

import geometry_msgs.PoseStamped;

import java.util.Date;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;


import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;


public class PoseStampedFactory {
	
	/**
	 * Read from a MongoDB result object
	 * 
	 * @param row BasicDBObject, e.g. result of a MongoDB query
	 * @return Instance of PoseStamped with values from row
	 */
	public static PoseStamped readFromDBObject(DBObject row) {

		NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
		MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
		
		PoseStamped p = messageFactory.newFromType(geometry_msgs.PoseStamped._TYPE);
		
		BasicDBObject header = (BasicDBObject) row.get("header");
		BasicDBObject pose   = (BasicDBObject) row.get("pose");

		// return null for empty poses
		if(header==null || pose == null)
			return null;
		
		p.getHeader().setFrameId(header.getString("frame_id"));
		p.getHeader().setSeq(header.getInt("seq"));
		p.getHeader().setStamp(new ISODate((Date) header.get("stamp")).toROSTime());
		
		p.getPose().getPosition().setX(((BasicDBObject) pose.get("position")).getDouble("x"));
		p.getPose().getPosition().setY(((BasicDBObject) pose.get("position")).getDouble("y"));
		p.getPose().getPosition().setZ(((BasicDBObject) pose.get("position")).getDouble("z"));
		
		p.getPose().getOrientation().setX(((BasicDBObject) pose.get("orientation")).getDouble("x"));
		p.getPose().getOrientation().setY(((BasicDBObject) pose.get("orientation")).getDouble("y"));
		p.getPose().getOrientation().setZ(((BasicDBObject) pose.get("orientation")).getDouble("z"));
		p.getPose().getOrientation().setW(((BasicDBObject) pose.get("orientation")).getDouble("w"));
		
		return p;
	}
	
	/**
	 * Transform pose to matrix representation 
	 * 
	 * @return Matrix4d representing this pose
	 */
	public static Matrix4d toMatrix4d(PoseStamped p) {
		
		Quat4d q = new Quat4d(p.getPose().getOrientation().getX(), 
							  p.getPose().getOrientation().getY(),
							  p.getPose().getOrientation().getZ(),
							  p.getPose().getOrientation().getW());
		
		Vector3d t = new Vector3d(p.getPose().getPosition().getX(),
								  p.getPose().getPosition().getY(),
								  p.getPose().getPosition().getZ());
		
		return new Matrix4d(q, t, 1);
	}
	
}

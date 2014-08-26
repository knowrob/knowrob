package org.knowrob.interfaces.mongo.types;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;


public class PoseFactory {
	
	/**
	 * Read from a MongoDB result object
	 * 
	 * @param row BasicDBObject, e.g. result of a MongoDB query
	 * @return Instance of Pose with values from row
	 */
	public static Matrix4d readFromDBObject(DBObject row) {	
		
		BasicDBObject pose   = (BasicDBObject) row.get("pose");

		Vector3d v = new Vector3d();
		v.x = ((BasicDBObject) pose.get("position")).getDouble("x");
		v.y = ((BasicDBObject) pose.get("position")).getDouble("y");
		v.z = ((BasicDBObject) pose.get("position")).getDouble("z");
		
		Quat4d q = new Quat4d();
		q.x = ((BasicDBObject) pose.get("orientation")).getDouble("x");
		q.y = ((BasicDBObject) pose.get("orientation")).getDouble("y");
		q.z = ((BasicDBObject) pose.get("orientation")).getDouble("z");
		q.w = ((BasicDBObject) pose.get("orientation")).getDouble("w");
		
		Matrix4d mat = new Matrix4d(q, v, 1.0);				
		return mat;
	}
	
}

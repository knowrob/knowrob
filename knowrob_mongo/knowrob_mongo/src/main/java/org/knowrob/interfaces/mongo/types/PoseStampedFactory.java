/*
 * Copyright (c) 2013 Moritz Tenorth
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Technische Universiteit Eindhoven nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
*/
package org.knowrob.interfaces.mongo.types;

import java.util.Date;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import tfjava.Stamped;


import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;


public class PoseStampedFactory {
	
	/**
	 * Read from a MongoDB result object
	 * 
	 * @param row BasicDBObject, e.g. result of a MongoDB query
	 * @return Instance of PoseStamped with values from row
	 */
	public static Stamped<Matrix4d> readFromDBObject(DBObject row) {

		Stamped<Matrix4d> p = new Stamped<Matrix4d>();
		
		BasicDBObject header = (BasicDBObject) row.get("header");
		BasicDBObject pose   = (BasicDBObject) row.get("pose");

		// return null for empty poses
		if(header==null || pose == null)
			return null;
		
		p.frameID = header.getString("frame_id");
		
		Object stamp = header.get("stamp");
		if(stamp instanceof Date) {
			p.timeStamp = new ISODate((Date)stamp).toROSTime();
		}
		else if(stamp instanceof Double) {
			Double miliSeconds = ((Double)stamp)*1000.0;
			p.timeStamp = new ISODate(miliSeconds.longValue()).toROSTime();
			
		}
		else if(stamp instanceof Long) {
			p.timeStamp = new ISODate(((Long)stamp).longValue()).toROSTime();
		}
		else {
			System.err.println("Unknown date type: " + stamp.getClass().getName());
			p.timeStamp = new ISODate(0l).toROSTime();
		}
		

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
		p.setData(mat);
		
		return p;
	}
	
}

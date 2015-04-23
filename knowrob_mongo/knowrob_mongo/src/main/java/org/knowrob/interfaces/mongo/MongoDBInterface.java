/*
 * Copyright (c) 2013 Moritz Tenorth, 2015 Daniel Beßler
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

package org.knowrob.interfaces.mongo;

import java.sql.Timestamp;
import java.text.DecimalFormat;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.TimeZone;
import java.util.regex.Pattern;

import javax.vecmath.Matrix4d;

import org.knowrob.interfaces.mongo.types.Designator;
import org.knowrob.interfaces.mongo.types.ISODate;
import org.knowrob.tfmemory.TFMemory;
import org.ros.message.Time;

import tfjava.Stamped;
import tfjava.StampedTransform;

import com.mongodb.BasicDBList;
import com.mongodb.BasicDBObject;
import com.mongodb.DB;
import com.mongodb.DBCollection;
import com.mongodb.DBCursor;
import com.mongodb.DBObject;
import com.mongodb.QueryBuilder;


public class MongoDBInterface {

	TFMemory mem;

	final SimpleDateFormat mongoDateFormat;
	
	/**
	 * Constructor
	 *
	 * Initialize DB client and connect to database.
	 *
	 */
	public MongoDBInterface() {
		// Format of dates as saved in mongo
		mongoDateFormat = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'");
		mongoDateFormat.setTimeZone(TimeZone.getTimeZone("GMT"));
        
		mem = TFMemory.getInstance();
	}
	
	/**
	 * @return DB handle of currently active DB
	 */
	public DB getDatabase() {
		return mem.getDatabase();
	}
	
	/**
	 * Set the DB name that is used for mongo queries.
	 * @param name The DB name
	 * @return DB handle
	 */
	public DB setDatabase(String name) {
		mem.setDatabase(name);
		return getDatabase();
	}


	/**
	 * Wrapper around the lookupTransform method of the TFMemory class
	 *
	 * @param sourceFrameId ID of the source frame of the transformation
	 * @param targetFrameId ID of the target frame of the transformation
	 * @param posix_ts POSIX timestamp (seconds since 1.1.1970)
	 * @return
	 */
	public StampedTransform lookupTransform(String targetFrameId, String sourceFrameId, double posix_ts) {
		Time t = new Time();
		t.secs = (int)posix_ts;
		t.nsecs = (int) (1E9 * (posix_ts - ((int) posix_ts)));
		return(mem.lookupTransform(targetFrameId, sourceFrameId, t));
	}

	/**
	 * Wrapper around the transformPose method of the TFMemory class
	 *
	 * @param targetFrameID  ID of the target frame of the transformation
	 * @param stampedIn      Stamped<Matrix4d> with the pose in the original coordinates
	 * @param stampedOut     Stamped<Matrix4d> that will hold the resulting pose
	 * @return               true if transform succeeded
	 */
	public boolean transformPose(String targetFrameID, Stamped<Matrix4d> stampedIn, Stamped<Matrix4d> stampedOut) {
		return mem.transformPose(targetFrameID, stampedIn, stampedOut);
	}
	
	public Designator getDecisionTree() {
		DBCollection coll = getDatabase().getCollection("resulting_decision_tree");
		DBObject query = new QueryBuilder().get();
		DBObject cols  = new BasicDBObject();
		cols.put("tree", 1 );
		
		DBCursor cursor = coll.find(query, cols);
		while(cursor.hasNext()) {
			DBObject row = cursor.next();
			Designator desig = new Designator().readFromDBList((BasicDBList) row.get("tree"));
			if(cursor.hasNext()) cursor.close();
			return desig;
		}
		
		cursor.close();
		
		return null;
	}
	
	/**
	 * Read designator value from either the uima_uima_results collection
	 * or the logged_designators collection.
	 *
	 * @param designator Designator ID to be read
	 * @return Instance of a Designator
	 */
	public Designator getDesignatorByID(Object designator) {
	
		return getDesignatorByID(designator, "designator._id");
	}
	
	/**
	 * Read designator value from either the uima_uima_results collection
	 * or the logged_designators collection.
	 *
	 * @param designator Designator ID to be read
	 * @param idKey The ID key in mongo DB
	 * @return Instance of a Designator
	 */
	 public Designator getDesignatorByID(Object designator, String idKey) {
		DBCollection coll = getDatabase().getCollection("logged_designators");
		DBObject query = QueryBuilder.start(idKey).is(designator).get();
		
		DBObject cols  = new BasicDBObject();
		cols.put("__recorded", 1 );
		cols.put("designator", 1 );
		
		DBCursor cursor = coll.find(query, cols);
		while(cursor.hasNext()) {
			DBObject row = cursor.next();
			Designator desig = new Designator().readFromDBObject((BasicDBObject) row.get("designator"));
			if(cursor.hasNext()) cursor.close();
			return desig;
		}
		
		cursor.close();
		
		return null;
    }
	
	/**
	 * Read designators based on the given filter pattern. The strings in the
	 * keys and values lists are AND-joined to form query expressions for MongoDB.
	 * 
	 * K = ['designator.TYPE','designator.GOAL.TO','designator.GOAL.OBJ.TYPE']
	 * V = [NAVIGATION,SEE,PANCAKEMIX].
	 * 
	 * @param keys Strings describing fields in a document using the dot notation 
	 * @param values Strings of values that these fields need to have
	 * @return List of @Designator data structures that match the query expressions
	 */
	public Designator[] getDesignatorsByPattern(String[] keys, String[] values) {
		
		DBCollection coll = getDatabase().getCollection("logged_designators");
		
		QueryBuilder qb = QueryBuilder.start("designator").exists("_id");
		for(int i=0; i<keys.length; i++) {
			qb = qb.and(keys[i]).is(Pattern.compile(values[i],Pattern.CASE_INSENSITIVE)); // pattern for case insensitive matching
		}
		
		DBObject query = qb.get();
		
		DBObject cols  = new BasicDBObject();
		cols.put("__recorded", 1 );		
		cols.put("designator", 1 );

		DBCursor cursor = coll.find(query, cols);

		Designator[] res = new Designator[cursor.size()];
		int r=0;
		
		while(cursor.hasNext()) {
			DBObject row = cursor.next();
			Designator desig = new Designator().readFromDBObject((BasicDBObject) row.get("designator"));
			res[r++]=desig;
		}
		cursor.close();
		
		return res;
	}


	/**
	 * Read the latest perception before the time point identified by posix_ts
	 *
	 * @param posix_ts Time stamp in POSIX format (seconds since 1.1.1970)
	 * @return Designator object returned by the last perception before that time
	 */
	public Designator latestUIMAPerceptionBefore(int posix_ts) {

		Designator desig = null;
		DBCollection coll = getDatabase().getCollection("logged_designators");

		// read all events up to one minute before the time
		Date start = new ISODate((long) 1000 * (posix_ts - 60) ).getDate();
		Date end   = new ISODate((long) 1000 * posix_ts ).getDate();

		DBObject query = QueryBuilder
				.start("__recorded").greaterThanEquals( start )
				.and("__recorded").lessThan( end )
				.and("designator.POSE").notEquals(null).get();

		DBObject cols  = new BasicDBObject();
		cols.put("designator", 1 );

		DBCursor cursor = coll.find(query, cols);
		cursor.sort(new BasicDBObject("__recorded", -1));
		try {
			while(cursor.hasNext()) {
				DBObject row = cursor.next();
				desig = new Designator().readFromDBObject((BasicDBObject) row.get("designator"));
				break;
			}
		} catch(Exception e){
			e.printStackTrace();
		} finally {
			cursor.close();
		}
		return desig;
	}

	/**
	 * Find latest designator before given timepoint.
	 * @param posix_ts Time stamp in POSIX format (seconds since 1.1.1970)
	 * @return The matching designator or null
	 */
	public Designator getLatestDesignatorBefore(long posix_ts) {
		return getLatestDesignatorBefore((double)posix_ts, null, null, null);
	}

	/**
	 * Find latest designator before given timepoint.
	 * @param posix_ts Time stamp in POSIX format (seconds since 1.1.1970)
	 * @return The matching designator or null
	 */
	public Designator getLatestDesignatorBefore(double posix_ts) {
		return getLatestDesignatorBefore(posix_ts, null, null, null);
	}

	/**
	 * Find latest designator that matches given key-value pairs.
	 * @param posix_ts Time stamp in POSIX format (seconds since 1.1.1970)
	 * @param keys DB keys
	 * @param relations Pointwise relations between key and value
	 * @param values DB values
	 * @return The matching designator or null
	 */
	public Designator getLatestDesignatorBefore(long posix_ts, String[] keys, String[] relations, Object[] values) {
		return getLatestDesignatorBefore((double)posix_ts, keys, relations, values);
	}
	
	/**
	 * Find latest designator that matches given key-value pairs.
	 * @param posix_ts Time stamp in POSIX format (seconds since 1.1.1970)
	 * @param keys DB keys
	 * @param relations Pointwise relations between key and value
	 * @param values DB values
	 * @return The matching designator or null
	 */
	// TODO: merge with latestUIMAPerceptionBefore & getDesignatorsByPattern
	public Designator getLatestDesignatorBefore(double posix_ts, String[] keys, String[] relations, Object[] values) {
		Designator desig = null;
		DBCollection coll = getDatabase().getCollection("logged_designators");

		// read all events up to one minute before the time
		Object t = new ISODate((long) (1000.0 * posix_ts) ).getDate();

		QueryBuilder query = QueryBuilder.start("__recorded").lessThan( t );
		
		if(relations!=null&&keys!=null&&values!=null) {
			for(int i=0; i<relations.length && i<keys.length && i<values.length; ++i) {
				String rel = relations[i];
				String key = keys[i];
				Object val = values[i];
				
				if("==".equals(rel) || "=".equals(rel) || "is".equals(rel))
					query = query.and(key).is(val);
				else if("!=".equals(rel))
					query = query.and(key).notEquals(val);
				else if("<".equals(rel))
					query = query.and(key).lessThan(val);
				else if("<=".equals(rel))
					query = query.and(key).lessThanEquals(val);
				else if(">".equals(rel))
					query = query.and(key).greaterThan(val);
				else if(">=".equals(rel))
					query = query.and(key).greaterThanEquals(val);
				else {
					System.err.println("Unknown mongo relation: " + rel);
				}
			}
		}
		
		DBObject queryInstance = null;
		try {
			queryInstance = query.get();
		}
		catch(Exception e){
			e.printStackTrace();
			return null;
		}
		
		DBObject cols  = new BasicDBObject();
		cols.put("designator", 1 );

		DBCursor cursor = coll.find(queryInstance, cols);
		cursor.sort(new BasicDBObject("__recorded", -1));
		try {
			if(cursor.hasNext()) {
				desig = new Designator().readFromDBObject(
					(BasicDBObject) cursor.next().get("designator"));
			}
		}
		catch(Exception e){
			e.printStackTrace();
		}
		finally {
			cursor.close();
		}
		return desig;
	}


	/**
	 * Get all times when an object has been detected
	 *
	 * @param object
	 * @return
	 */
	public List<Date> getUIMAPerceptionTimes(String object) {

		List<Date> times = new ArrayList<Date>();
		DBCollection coll = getDatabase().getCollection("logged_designators");

		// TODO: This will always return a single result since the ID is unique
		DBObject query = QueryBuilder
				.start("designator._id").is(object)
				.and("designator.POSE").notEquals(null).get();

		DBObject cols  = new BasicDBObject();
		cols.put("__recorded", 1 );

		DBCursor cursor = coll.find(query, cols);
		cursor.sort(new BasicDBObject("__recorded", -1));
		try {
			while(cursor.hasNext()) {

				DBObject row = cursor.next();
				Date currentTime = (new ISODate(0).readFromDBObject((BasicDBObject) row.get("__recorded"))).getDate();
				times.add(currentTime);

			}
		} catch(Exception e){
			e.printStackTrace();
		} finally {
			cursor.close();
		}
		return times;
	}
	
	/**
	 * Find distinct designator values for given DB key.
	 * @param key The DB key
	 * @return Array of different values
	 */
	public Object[] getDistinctDesignatorValues(String key) {
		DBCollection coll = getDatabase().getCollection("logged_designators");
		
		List<?> l = coll.distinct(key);
		Object[] out = new Object[l.size()];
		int index = 0;
		for(Object v : l) {
			out[index] = v;
			index += 1;
		}
		
		return out;
	}
	
	/**
	 * Compute world transformation matrix that represents the pose
	 * of a designator.
	 * @param id The designator id
	 * @return The transformation matrix
	 */
	@SuppressWarnings("unchecked")
	public Matrix4d getDesignatorLocation(String id) {
		// FIXME: bad assumption
		final String targetFrame = "/map";
		
		Stamped<Matrix4d> poseMatrix = null;
		DBCollection coll = getDatabase().getCollection("logged_designators");
		DBObject query = QueryBuilder
				.start("designator._id").is(id).get();

		DBObject cols  = new BasicDBObject();
		cols.put("__recorded", 1 );
		cols.put("designator", 1 );

		DBCursor cursor = coll.find(query, cols);
		try {
			cursor.sort(new BasicDBObject("__recorded", -1));
			
			while(cursor.hasNext()) {

				DBObject row = cursor.next();
				Object mat = null;
				
				Designator d = new Designator().readFromDBObject((BasicDBObject) row.get("designator"));
				Designator loc = (Designator)d.get("AT");
				if(loc != null) {
					mat = loc.get("POSE");
				}
				if(mat == null) {
					mat = d.get("POSE");
				}
				
				if(mat!=null && mat instanceof Stamped<?>) {
					poseMatrix = (Stamped<Matrix4d>)mat;
					if(!poseMatrix.frameID.startsWith("/"))
						poseMatrix.frameID = "/"+poseMatrix.frameID;
					
					// Transform pose to target frame if required
					if(poseMatrix.frameID!=null && !targetFrame.equals(poseMatrix.frameID)) {
						final Stamped<Matrix4d> worldFrame = new Stamped<Matrix4d>();
						worldFrame.setData(new Matrix4d());
						worldFrame.getData().setIdentity();
						
						if(transformPose(targetFrame, poseMatrix, worldFrame))
							poseMatrix = worldFrame;
					}
					
					break;
				}

			}
		} catch(Exception e){
			e.printStackTrace();
		} finally {
			cursor.close();
		}
		if(poseMatrix == null)
			return null;
		else
			return poseMatrix.getData();
	}

	/**
	 * Computes a timestamp that corresponds to the specified date.
	 * date format must be as follows: "yyyy-MM-dd'T'HH:mm:ss.SSS'Z'"
	 * @throws ParseException 
	 */
	public String getMongoTimestamp(String date) throws ParseException {
		// Avoid scientific notation
		DecimalFormat df = new DecimalFormat("#");
		df.setMaximumFractionDigits(9);
		return df.format(mongoDateFormat.parse(date).getTime()/1000.0);
	}

	public static void main(String[] args) {

//		MongoDBInterface m = new MongoDBInterface();
//
//		Designator d = m.getDesignatorByID("designator_C4yixt3iPwKHCt");
//		
//		
//		ArrayList<String> k = new ArrayList<String>();
//		ArrayList<String> v = new ArrayList<String>();
//		
//		
//		k.add("designator.TYPE");
//		k.add("designator.GOAL.TO");
//		k.add("designator.GOAL.OBJ.TYPE");
//
//		v.add("NAVIGATION");
//		v.add("SEE");
//		v.add("PANCAKEMIX");
//		
//		Designator[] res = m.getDesignatorsByPattern(
//				new String[]{"designator.TYPE", "designator.GOAL.TO", "designator.GOAL.OBJ.TYPE"}, 
//				new String[]{"navigation", "see", "PANCAKEMIX"});
//		
//		System.out.println(res.length);

		// test transformation lookup based on DB information

//		Timestamp timestamp = Timestamp.valueOf("2013-07-26 14:27:22.0");
//		Time t = new Time(1396512420);
//		Time t = new Time(1396512422); // no
//		Time t = new Time(1396512424);  //1

		TFMemory tf = TFMemory.getInstance();

		try {
			SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
			Date date = sdf.parse("2014-04-30 13:31:51.224");
			Time t = new Time(date.getTime()/1000.0);
			System.out.println("UTC " + date + " -> " + date.getTime()/1000.0);
			sdf.setTimeZone(TimeZone.getTimeZone("GMT"));
			date = sdf.parse("2014-04-30 13:31:51.224");
			System.out.println("GMT " + date + " -> " + date.getTime()/1000.0);
			
			t = new Time(date.getTime()/1000.0);
			
			System.out.println(tf.lookupTransform("/map", "/RightHand", t));
		}
		catch (ParseException e) {
			e.printStackTrace();
		}
		
		Timestamp timestamp = Timestamp.valueOf("2014-08-27 13:30:35.0");
		//Time t = new Time(timestamp.getTime());  //1
		System.out.println(timestamp.getTime());


		Time t_st  = new Time(1396512420);
		Time t_end = new Time(1396512422);

		long t0 = System.nanoTime();
		System.out.println(tf.lookupTransform("/base_link", "/l_gripper_palm_link", t_end));
		long t1 = System.nanoTime();
		System.out.println(tf.lookupTransform("/base_link", "/l_gripper_palm_link", t_end));
		long t2 = System.nanoTime();
		System.out.println(tf.lookupTransform("/base_link", "/l_gripper_palm_link", t_st));
		long t3 = System.nanoTime();

		double first  = (t1-t0)/ 1E6;
		double second = (t2-t1)/ 1E6;
		double third  = (t3-t2)/ 1E6;
		
		System.out.println("Time to look up first transform: " + first + "ms");
		System.out.println("Time to look up second transform: " + second + "ms");
		System.out.println("Time to look up second transform: " + third + "ms");

		// test lookupTransform wrapper
//		trans = m.lookupTransform("/map", "/head_mount_kinect_ir_link", 1377766521);
//		System.out.println(trans);

//		// test UIMA result interface
//		Designator d = m.latestUIMAPerceptionBefore(1377766521);
//		System.out.println(d);
//
//		// test designator reading
//		d = m.getDesignatorByID("designator_bunEaUUmPbuoLN");
//		System.out.println(d);
	}
}


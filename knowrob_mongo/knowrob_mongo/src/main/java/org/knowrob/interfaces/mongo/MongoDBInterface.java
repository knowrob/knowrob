package org.knowrob.interfaces.mongo;

import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Map;

import javax.vecmath.Matrix4d;

import org.knowrob.interfaces.mongo.types.Designator;
import org.knowrob.interfaces.mongo.types.ISODate;
import org.knowrob.tfmemory.TFMemory;
import org.ros.message.Time;

import tfjava.Stamped;
import tfjava.StampedTransform;

import com.mongodb.BasicDBObject;
import com.mongodb.DB;
import com.mongodb.DBCollection;
import com.mongodb.DBCursor;
import com.mongodb.DBObject;
import com.mongodb.MongoClient;
import com.mongodb.QueryBuilder;


public class MongoDBInterface {

	MongoClient mongoClient;
	DB db;

	TFMemory mem;

	/**
	 * Constructor
	 *
	 * Initialize DB client and connect to database.
	 *
	 */
	public MongoDBInterface() {

		String host = "localhost";
		int port = 27017;
		
		// check if MONGO_PORT_27017_TCP_ADDR and MONGO_PORT_27017_TCP_PORT 
		// environment variables are set
		
        Map<String, String> env = System.getenv();
        if(env.containsKey("MONGO_PORT_27017_TCP_ADDR")) {
        	host = env.get("MONGO_PORT_27017_TCP_ADDR");
        }
        
        if(env.containsKey("MONGO_PORT_27017_TCP_PORT")) {
        	port = Integer.valueOf(env.get("MONGO_PORT_27017_TCP_PORT"));
        }
        
		try {
			mongoClient = new MongoClient(host, port);
			db = mongoClient.getDB("roslog");

		} catch (UnknownHostException e) {
			e.printStackTrace();
		}
		mem = TFMemory.getInstance();
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


	/**
	 * Read designator value from either the uima_uima_results collection
	 * or the logged_designators collection.
	 *
	 * @param designator Designator ID to be read
	 * @return Instance of a Designator
	 */
	public Designator getDesignatorByID(String designator) {
		DBCollection coll = db.getCollection("logged_designators");
		DBObject query = QueryBuilder
				.start("designator._id").is(designator).get();

		DBObject cols  = new BasicDBObject();
		cols.put("__recorded", 1 );		
		cols.put("designator", 1 );

		DBCursor cursor = coll.find(query, cols);

		while(cursor.hasNext()) {
			DBObject row = cursor.next();
			Designator desig = new Designator().readFromDBObject((BasicDBObject) row.get("designator"));
			return desig;
		}
		cursor.close();
		
		return null;
	}


	/**
	 * Read the latest perception before the time point identified by posix_ts
	 *
	 * @param posix_ts Time stamp in POSIX format (seconds since 1.1.1970)
	 * @return Designator object returned by the last perception before that time
	 */
	public Designator latestUIMAPerceptionBefore(int posix_ts) {

		Designator desig = null;
		DBCollection coll = db.getCollection("logged_designators");

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
	 * Get all times when an object has been detected
	 *
	 * @param object
	 * @return
	 */
	public List<Date> getUIMAPerceptionTimes(String object) {

		List<Date> times = new ArrayList<Date>();
		DBCollection coll = db.getCollection("logged_designators");

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


	@SuppressWarnings("unchecked")
	public Matrix4d getDesignatorLocation(String id) {
		Matrix4d poseMatrix = null;
		DBCollection coll = db.getCollection("logged_designators");
		DBObject query = QueryBuilder
				.start("designator._id").is(id).get();

		DBObject cols  = new BasicDBObject();
		cols.put("__recorded", 1 );
		cols.put("designator", 1 );

		DBCursor cursor = coll.find(query, cols);
		cursor.sort(new BasicDBObject("__recorded", -1));
		try {
			while(cursor.hasNext()) {

				DBObject row = cursor.next();
				Designator res = new Designator().readFromDBObject((BasicDBObject) row.get("designator"));
				Designator res2 = (Designator)res.get("AT");
				poseMatrix = ((Stamped<Matrix4d>)res2.get("POSE")).getData();
				break;

			}
		} catch(Exception e){
			e.printStackTrace();
		} finally {
			cursor.close();
		}
		return poseMatrix;
	}

	public static void main(String[] args) {

//		MongoDBInterface m = new MongoDBInterface();


		// test transformation lookup based on DB information

//		Timestamp timestamp = Timestamp.valueOf("2013-07-26 14:27:22.0");
//		Time t = new Time(1377766521);
//		Time t = new Time(1383143712); // no
//		Time t = new Time(1383144279);  //1



		Time t_st  = new Time(1392799358);
		Time t_end = new Time(1392799363);

		long t0 = System.nanoTime();
		TFMemory tf = TFMemory.getInstance();
		System.out.println(tf.lookupTransform("/base_link", "/l_gripper_palm_link", t_end));
		long t1 = System.nanoTime();
		System.out.println(tf.lookupTransform("/base_link", "/l_gripper_palm_link", t_end));
		long t2 = System.nanoTime();
		System.out.println(tf.lookupTransform("/base_link", "/l_gripper_palm_link", t_st));
		long t3 = System.nanoTime();

		double first  = (t1-t0)/ 1E9;
		double second = (t2-t1)/ 1E9;
		double third  = (t3-t2)/ 1E9;
		
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


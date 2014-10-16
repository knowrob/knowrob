package org.knowrob.interfaces.mongo;

import java.net.UnknownHostException;
import java.sql.Timestamp;
import java.text.DecimalFormat;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Map;
import java.util.TimeZone;
import java.util.regex.Pattern;

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

	final SimpleDateFormat mongoDateFormat;
	
	/**
	 * Constructor
	 *
	 * Initialize DB client and connect to database.
	 *
	 */
	public MongoDBInterface() {

		String host = "localhost";
		int port = 27017;
		
		// Format of dates as saved in mongo
		mongoDateFormat = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'");
		mongoDateFormat.setTimeZone(TimeZone.getTimeZone("GMT"));
		
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
		
		DBCollection coll = db.getCollection("logged_designators");
		
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

//		TFMemory tf = TFMemory.getInstance();
//
//		try {
//			SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
//			Date date = sdf.parse("2014-04-30 13:31:51.224");
//			Time t = new Time(date.getTime()/1000.0);
//			System.out.println("UTC " + date + " -> " + date.getTime()/1000.0);
//			sdf.setTimeZone(TimeZone.getTimeZone("GMT"));
//			date = sdf.parse("2014-04-30 13:31:51.224");
//			System.out.println("GMT " + date + " -> " + date.getTime()/1000.0);
//			
//			t = new Time(date.getTime()/1000.0);
//			
//			System.out.println(tf.lookupTransform("/map", "/RightHand", t));
//		}
//		catch (ParseException e) {
//			e.printStackTrace();
//		}
//		
//		Timestamp timestamp = Timestamp.valueOf("2014-08-27 13:30:35.0");
//		Time t = new Time(timestamp.getTime());  //1
//		System.out.println(timestamp.getTime());
//
//
//		Time t_st  = new Time(1396512420);
//		Time t_end = new Time(1396512422);
//
//		long t0 = System.nanoTime();
//		System.out.println(tf.lookupTransform("/base_link", "/l_gripper_palm_link", t_end));
//		long t1 = System.nanoTime();
//		System.out.println(tf.lookupTransform("/base_link", "/l_gripper_palm_link", t_end));
//		long t2 = System.nanoTime();
//		System.out.println(tf.lookupTransform("/base_link", "/l_gripper_palm_link", t_st));
//		long t3 = System.nanoTime();
//
//		double first  = (t1-t0)/ 1E6;
//		double second = (t2-t1)/ 1E6;
//		double third  = (t3-t2)/ 1E6;
//		
//		System.out.println("Time to look up first transform: " + first + "ms");
//		System.out.println("Time to look up second transform: " + second + "ms");
//		System.out.println("Time to look up second transform: " + third + "ms");
		
		SimpleDateFormat sdf_date = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'");
		try {
			sdf_date.setTimeZone(TimeZone.getTimeZone("GMT")); //necessary because MongoDB time is 2 hours behind
//			Date date = sdf_date.parse("2014-04-03T08:07:00.000Z");
			Date date = sdf_date.parse("1970-01-01T00:00:03.357Z");
			System.out.println(date.getTime());
			Time timestamp_GMT = new Time(date.getTime()/1000.0);
			System.out.println("Timestamp: " + timestamp_GMT);
			
		
		TFMemory tf = TFMemory.getInstance();
		System.out.println(tf.lookupTransform("kitchen_world", "spatula", timestamp_GMT));
		}
		
		
//		TFMemory tf = TFMemory.getInstance();
//
//		try {
//			SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
//			Date date = sdf.parse("2014-04-30 13:31:51.224");
//			Time t = new Time(date.getTime()/1000.0);
//			System.out.println("UTC " + date + " -> " + date.getTime()/1000.0);
//			sdf.setTimeZone(TimeZone.getTimeZone("GMT"));
//			date = sdf.parse("2014-04-30 13:31:51.224");
//			System.out.println("GMT " + date + " -> " + date.getTime()/1000.0);
//			
//			t = new Time(date.getTime()/1000.0);
//			
//			System.out.println(tf.lookupTransform("/map", "/RightHand", t));
//		}
		catch (ParseException e) {
			e.printStackTrace();
		}
//		

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


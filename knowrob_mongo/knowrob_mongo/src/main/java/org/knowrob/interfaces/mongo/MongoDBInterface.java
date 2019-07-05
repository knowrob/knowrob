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

import java.net.UnknownHostException;
import java.text.DecimalFormat;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.TimeZone;
import java.util.regex.Pattern;
import java.io.IOException;

import javax.vecmath.Matrix4d;

import org.knowrob.interfaces.mongo.types.Designator;
import org.knowrob.tfmemory.TFMemory;
import org.ros.message.Time;

import org.knowrob.tfmemory.Stamped;
import org.knowrob.tfmemory.StampedTransform;

import com.mongodb.BasicDBObject;
import com.mongodb.DB;
import com.mongodb.DBCollection;
import com.mongodb.DBCursor;
import com.mongodb.DBObject;
import com.mongodb.QueryBuilder;
import com.mongodb.MongoClient;
import com.mongodb.util.JSON;

import org.apache.log4j.Logger;

public class MongoDBInterface {
	final static Logger logger = Logger.getLogger(MongoDBInterface.class);

	TFMemory mem;

	final SimpleDateFormat mongoDateFormat;
	
	public static final String COLLECTION_TF = "tf";
	public static final String COLLECTION_LOGGED_DESIGNATORS = "logged_designators";

	MongoClient mongoClient;
	DB db;
	
	public static MongoClient connect() throws UnknownHostException {
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
        
		return new MongoClient(host, port);
	};
	
	/**
	 * Constructor
	 *
	 * Initialize DB client and connect to database.
	 *
	 */
	public MongoDBInterface() {
		try {
			mongoClient = MongoDBInterface.connect();
			
		} catch (UnknownHostException e) {
			e.printStackTrace(); // FIXME bad exception handling
		}
		
		// Format of dates as saved in mongo
		mongoDateFormat = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'");
		mongoDateFormat.setTimeZone(TimeZone.getTimeZone("GMT"));
        
		mem = TFMemory.getInstance();
		
		setDatabase("roslog");
	}
	
	/**
	 * @return DB handle of currently active DB
	 */
	public DB getDatabase() {
		return db;
	}
	
	public String getDatabaseName() {
		return db.getName();
	}
	
	/**
	 * Set the DB name that is used for mongo queries.
	 * @param name The DB name
	 * @return DB handle
	 */
	public DB setDatabase(String name) {
		db = mongoClient.getDB(name);
		mem.setDatabase(name);
		return getDatabase();
	}
	
	public String[] getCollections() {
		List<String> out = new LinkedList<String>();
		for(String name : db.getCollectionNames()) {
			out.add(name);
		}
		return (String[]) out.toArray(new String[out.size()]);
	}
	
	public void createIndex(String collection, String keys[]) {
		DBCollection coll = db.getCollection(collection);
		for(String k : keys) {
			coll.createIndex(new BasicDBObject(k,1));
		}
	}
	
	public void dump(String path) {
		try {
			Runtime.getRuntime().exec("mongodump --db " + getDatabaseName() + " --out " + path);
		} catch (IOException ex) {
			logger.error(ex.getMessage());
		}
	}
	
	public void restore(String path) {
		try {
			Runtime.getRuntime().exec("mongorestore " + path);
		} catch (IOException ex) {
			logger.error(ex.getMessage());
		}
	}
	
	/**
	 * Store a JSON encoded string in a mongo collection with
	 * the given name.
	 */
	public DBObject store(String collection, String json_string) {
		DBCollection coll = db.getCollection(collection);
		DBObject dbObject = (DBObject)JSON.parse(json_string);
		coll.insert(dbObject);
		return dbObject;
	}
	
	/**
	 * Drop a collection.
	 */
	public void drop(String collection) {
		db.getCollection(collection).drop();
	}
	
	/**
	 * Update an existing document with the JSON encoded data provided.
	 */
	public void update(String collection, BasicDBObject a, String json_string) {
		DBCollection coll = db.getCollection(collection);
		DBObject b = (DBObject)JSON.parse(json_string);
		coll.update(
			new BasicDBObject("_id", a.getObjectId("_id")),
			new BasicDBObject("$set", b)
		);
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
	
	public BasicDBObject queryOne(String collection, String[] keys, String[] relations, Object[] values) {
		return one( query(collection, keys, relations, values) );
	}
	
	public BasicDBObject[] querySome(String collection, String[] keys, String[] relations, Object[] values, int count) {
		return some( query(collection, keys, relations, values), count );
	}
	
	public BasicDBObject[] queryAll(String collection, String[] keys, String[] relations, Object[] values) {
		return all( query(collection, keys, relations, values) );
	}
	
	public DBObject queryBuildConjunction(String[] keys, String[] relations, Object[] values) {
		QueryBuilder query = QueryBuilder.start();
		// Parse the query
		if(relations!=null && keys!=null && values!=null &&
		   relations.length==keys.length && keys.length==values.length)
		{
			for(int i=0; i<relations.length; ++i) {
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
				else if("exist".equals(rel) || "exists".equals(rel))
					query = query.and(key).exists(val);
				else if("or".equals(rel))
					query = query.and((DBObject)val);
				else {
					logger.error("Unknown mongo relation: " + rel);
				}
			}
		}
		return query.get();
	}
	
	public DBObject disjunction(String[] keys, String[] relations, Object[] values) {
		QueryBuilder query = QueryBuilder.start();
		List<DBObject> ors = new LinkedList<DBObject>();
		
		// Parse the query
		if(relations!=null && keys!=null && values!=null &&
		   relations.length==keys.length && keys.length==values.length)
		{
			for(int i=0; i<relations.length; ++i) {
				String rel = relations[i];
				String key = keys[i];
				Object val = values[i];
				
				if("==".equals(rel) || "=".equals(rel) || "is".equals(rel))
					ors.add(QueryBuilder.start(key).is(val).get());
				else if("!=".equals(rel))
					ors.add(QueryBuilder.start(key).notEquals(val).get());
				else if("<".equals(rel))
					ors.add(QueryBuilder.start(key).lessThan(val).get());
				else if("<=".equals(rel))
					ors.add(QueryBuilder.start(key).lessThanEquals(val).get());
				else if(">".equals(rel))
					ors.add(QueryBuilder.start(key).greaterThan(val).get());
				else if(">=".equals(rel))
					ors.add(QueryBuilder.start(key).greaterThanEquals(val).get());
				else if("exist".equals(rel) || "exists".equals(rel))
					ors.add(QueryBuilder.start(key).exists(val).get());
				else {
					logger.error("Unknown mongo relation: " + rel);
				}
			}
		}
		
		return query.or((DBObject[])
			ors.toArray(new DBObject[ors.size()])).get();
	}
	
	public DBCursor query(String collection, String[] keys, String[] relations, Object[] values) {
		try {
			return query(collection, queryBuildConjunction(keys, relations, values));
		}
		catch (Exception e) {
			// TODO: throw exception
			logger.error("MONGO query failed: " + e.getMessage());
			e.printStackTrace();
			return null;
		}
	}
	
	public DBCursor query(String collection) {
		return query(collection, QueryBuilder.start().get());
	}
	
	public DBCursor query(String collection, DBObject query) {
		try {
			DBCollection coll = getDatabase().getCollection(collection);
			DBObject cols  = new BasicDBObject();
			DBCursor cursor = coll.find(query, cols);
			return cursor;
		}
		catch (Exception e) {
			// TODO: throw exception
			logger.error("MONGO query failed: " + e.getMessage());
			e.printStackTrace();
			return null;
		}
	}
	
	public DBCursor ascending(DBCursor cursor, String key) {
		try {
			return cursor.sort(new BasicDBObject(key, 1));
		}
		catch(Exception e) {
			// TODO: throw exception
			logger.error("Failed to sort cursor: " + e.getMessage());
			e.printStackTrace();
			return null;
		}
	}
	
	public DBCursor descending(DBCursor cursor, String key) {
		try {
			return cursor.sort(new BasicDBObject(key, -1));
		}
		catch(Exception e) {
			// TODO: throw exception
			logger.error("Failed to sort cursor: " + e.getMessage());
			e.printStackTrace();
			return null;
		}
	}

	public DBCursor limit(DBCursor cursor, int number) {
		try {
			return cursor.limit(number);
		}
		catch(Exception e) {
			// TODO: throw exception
			logger.error("Failed to limit cursor: " + e.getMessage());
			e.printStackTrace();
			return null;
		}
	}
	
	public BasicDBObject one(DBCursor cursor) {
		if(cursor.hasNext())
			return (BasicDBObject) cursor.next();
		else
			return null;
	}
	
	public BasicDBObject[] some(DBCursor cursor, int count) {
		LinkedList<BasicDBObject> out = new LinkedList<BasicDBObject>();
		while(cursor.hasNext() && out.size()<count) {
			out.add((BasicDBObject) cursor.next());
		}
		if(out.isEmpty()) {
			return null;
		}
		else {
			return out.toArray(new BasicDBObject[out.size()]);
		}
	}
	
	public BasicDBObject[] all(DBCursor cursor) {
		LinkedList<BasicDBObject> out = new LinkedList<BasicDBObject>();
		while(cursor.hasNext()) {
			out.add((BasicDBObject) cursor.next());
		}
		if(out.isEmpty()) {
			return null;
		}
		else {
			return out.toArray(new BasicDBObject[out.size()]);
		}
	}
	
	public void close(DBCursor cursor) {
		cursor.close();
	}
	
	/**
	 * Find distinct designator values for given DB key.
	 * @param key The DB key
	 * @return Array of different values
	 */
	public Object[] distinctValues(String collection, String key) {
		DBCollection coll = getDatabase().getCollection(collection);
		
		List<?> l = coll.distinct(key);
		Object[] out = new Object[l.size()];
		int index = 0;
		for(Object v : l) {
			out[index] = v;
			index += 1;
		}
		
		return out;
	}

	public Designator designator(BasicDBObject obj) {
		java.util.Date instant = (java.util.Date) obj.get("__recorded");
		return new Designator(instant).readFromDBObject((BasicDBObject) obj.get("designator"));
	}

	@SuppressWarnings("unchecked")
	public Matrix4d location(Designator desig) {
		Object mat=null;
		Stamped<Matrix4d> poseMatrix = null;
		// FIXME: bad assumption
		final String targetFrame = "/map";

		Designator loc = (Designator)desig.get("AT");
		if(loc != null) {
			mat = loc.get("POSE");
		}
		if(mat == null) {
			mat = desig.get("POSE");
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
}


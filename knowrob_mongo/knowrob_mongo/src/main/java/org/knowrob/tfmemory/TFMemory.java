/*
 * Copyright (c) 2013 Moritz Tenorth, Sjoerd van den Dries
 *
 * Based on the TFListener class by Sjoerd v.d. Dries in tfjava
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

package org.knowrob.tfmemory;

import java.net.UnknownHostException;
import java.util.Date;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.PriorityQueue;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.knowrob.interfaces.mongo.types.ISODate;
import org.ros.message.Duration;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;

import geometry_msgs.TransformStamped;
import tfjava.Stamped;
import tfjava.StampedTransform;

import com.mongodb.BasicDBList;
import com.mongodb.BasicDBObject;
import com.mongodb.DB;
import com.mongodb.DBCollection;
import com.mongodb.DBCursor;
import com.mongodb.DBObject;
import com.mongodb.MongoClient;
import com.mongodb.QueryBuilder;

/**
 * A client that reads transforms from the DB, stores them in a buffer and allows transformation
 * lookups from one frame to another.
 *
 * All tf messages published on the /tf topic are stored in
 * a buffer, first sorted by child frame, then by parent frame, then by time stamp. This allows fast
 * lookup of transformations. Tf's that are MAX_STORAGE_TIME older than the newest tf in the corresponding
 * time cache are ignored.
 *
 * To calculate a transformation from some source frame S to a target frame T at time t, TFListener uses a graph
 * search to find the best path from S to T. At the moment, 'best' means that the largest difference between
 * the time stamps of the transformations on the path and time t is minimized. If the tf graph is a tree, as is
 * the case with original C++-implementation of tf, the graph will simply return the only path available (if any).
 *
 * TFlistener is implemented as a singleton, which guarantees that at any time at most one client per system is
 * listening to the /tf topic.
 *
 * @author Sjoerd van den Dries, Moritz Tenorth
 * @version March 4, 2011
 */
public class TFMemory {

	/** The singleton instance */
	protected static TFMemory instance;

	/** Map that maps frame IDs (names) to frames */
	protected HashMap<String, Frame> frames;

	/** size of the local buffer of transforms (equiv. of the 10s buffer of common tf) */
	protected final static float BUFFER_SIZE = 1f;

	/** Maximum buffer storage time considered during garbage collection */
	public static final long MAX_EXTRAPOLATION_TIME = (new Duration(10, 0)).totalNsecs();

	MongoClient mongoClient;
	DB db;

	private String tfTableName;

	/* **********************************************************************
	 * *                           INITIALIZATION                           *
	 * ********************************************************************** */

	/**
	 * Returns the TFListener instance.
	 */
	public synchronized static TFMemory getInstance() {

		if (instance == null) {
			instance = new TFMemory();
		}
		return instance;
	}

	/**
	 * Class constructor.
	 */
	protected TFMemory() {

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

		frames = new HashMap<String, Frame>();

		tfTableName = "tf";

	}

	public String getTfTableName() {
		return tfTableName;
	}
	
	public void setTfTableName(String tfTableName) {
		this.tfTableName = tfTableName;
	}
	
	public DB getDatabase() {
		return db;
	}
	
	public DB setDatabase(String name) {
		db = mongoClient.getDB(name);
		frames = new HashMap<String, Frame>();
		return db;
	}

	/* **********************************************************************
	 * *                            TF LISTENER                             *
	 * ********************************************************************** */

	protected boolean setTransforms(BasicDBList transforms) {

		for(Object transform : transforms) {
			setTransform((BasicDBObject) transform);
		}
		return true;
	}

	/**
	 * Converts transform (a geometry msg) to a TransformStorage object and adds it to the buffer.
	 */
	protected boolean setTransform(BasicDBObject db_transform) {

		// resolve the frame ID's
		String childFrameID = assertResolved("",
				getFrameName(db_transform.get("child_frame_id")));
		String frameID = assertResolved("",
				getFrameName(((BasicDBObject) db_transform.get("header")).get("frame_id")));

		boolean errorExists = false;
		if (childFrameID == frameID) {
			System.err.println("TF_SELF_TRANSFORM: Ignoring transform with frame_id and child_frame_id  \"" + childFrameID + "\" because they are the same");
			errorExists = true;
		}

		if (childFrameID == "/") { //empty frame id will be mapped to "/"
			System.err.println("TF_NO_CHILD_FRAME_ID: Ignoring transform because child_frame_id not set ");
			errorExists = true;
		}

		if (frameID == "/") { //empty parent id will be mapped to "/"
			System.err.println("TF_NO_FRAME_ID: Ignoring transform with child_frame_id \"" + childFrameID + "\" because frame_id not set");
			errorExists = true;
		}

		if (errorExists) return false;


		// add frames to map
		Frame childFrame  = lookupOrInsertFrame(childFrameID);
		Frame parentFrame = lookupOrInsertFrame(frameID);

		TransformStorage tf = new TransformStorage().readFromDBObject(db_transform, childFrame, parentFrame);

		// try to insert tf in corresponding time cache. If result is FALSE, the tf contains old data.
		if (!childFrame.insertData(tf)) {
			System.err.println("TF_OLD_DATA ignoring data from the past for frame \"" + childFrameID + "\" at time " + ((double)tf.getTimeStamp() / 1E9));
			return false;
		}

		return true;
	}

	/**
	 * Looks up and returns the frame belonging to the given frame ID.
	 * If the frame does not exist yet, it is first added to the map.
	 */
	protected Frame lookupOrInsertFrame(String frameID) {
		Frame frame = frames.get(frameID);
		if (frame == null) {
			frame = new Frame(frameID, MAX_EXTRAPOLATION_TIME);
			frames.put(frameID, frame);
		}
		return frame;
	}


	/* **********************************************************************
	 * *                         TRANSFORM METHODS                          *
	 * ********************************************************************** */

	/**
	 * Transforms a stamped point to the given target frame, and returns the result in stampedOut.
	 */
	public void transformPoint(String targetFrameID, Stamped<Point3d> stampedIn, Stamped<Point3d> stampedOut) {
		StampedTransform transform = lookupTransform(targetFrameID, stampedIn.frameID, stampedIn.timeStamp);
		transform.transformPoint(stampedIn.getData(), stampedOut.getData());
		stampedOut.frameID = targetFrameID;
		stampedOut.timeStamp = stampedIn.timeStamp;
	}

	/**
	 * Transforms a stamped point to the given target frame and time, based on a given fixed frame, and
	 * returns the result in stampedOut.
	 */
	public void transformPoint(String targetFrameID, Time targetTime, Stamped<Point3d> stampedIn,
			String fixedFrameID, Stamped<Point3d> stampedOut) {
		StampedTransform transform = lookupTransform(targetFrameID, targetTime, stampedIn.frameID, stampedIn.timeStamp, fixedFrameID);
		transform.transformPoint(stampedIn.getData(), stampedOut.getData());
		stampedOut.frameID = targetFrameID;
		stampedOut.timeStamp = stampedIn.timeStamp;
	}
	/**
	 * Transforms a stamped pose to the given target frame, and returns the result in stampedOut.
	 */
	public boolean transformPose(String targetFrameID, Stamped<Matrix4d> stampedIn, Stamped<Matrix4d> stampedOut) {
		StampedTransform transform = lookupTransform(targetFrameID, stampedIn.frameID, stampedIn.timeStamp);
		if(transform!=null) {
			transform.transformPose(stampedIn.getData(), stampedOut.getData());
			stampedOut.frameID = targetFrameID;
			stampedOut.timeStamp = stampedIn.timeStamp;
			return true;
		}
		return false;
	}

	/**
	 * Transforms a stamped pose to the given target frame and time, based on a given fixed frame, and
	 * returns the result in stampedOut.
	 */
	public void transformPose(String targetFrameID, Time targetTime, Stamped<Matrix4d> stampedIn,
			String fixedFrameID, Stamped<Matrix4d> stampedOut) {
		StampedTransform transform = lookupTransform(targetFrameID, targetTime, stampedIn.frameID, stampedIn.timeStamp, fixedFrameID);
		transform.transformPose(stampedIn.getData(), stampedOut.getData());
		stampedOut.frameID = targetFrameID;
		stampedOut.timeStamp = stampedIn.timeStamp;
	}

	/* **********************************************************************
	 * *                          LOOKUP METHODS                            *
	 * ********************************************************************** */

	/**
	 * Returns the transform from the specified source frame to the target frame at a given time; returns
	 * null if no transformation could be found.
	 */
	public StampedTransform lookupTransform(String targetFrameID, String sourceFrameID, Time time) {
		// resolve the source and target IDs
		String resolvedTargetID = assertResolved("", targetFrameID);
		String resolvedSourceID = assertResolved("", sourceFrameID);

		// if source and target are the same, return the identity transform
		if (resolvedSourceID.equals(resolvedTargetID)) {
			StampedTransform out = StampedTransform.getIdentity();
			out.timeStamp = time;
			out.frameID = resolvedSourceID;
			out.childFrameID = resolvedTargetID;
			return out;
		}

		// load data from DB if the current time point is already in the buffer
		Frame sourceFrame = verifyDataAvailable(time, resolvedSourceID);
		if(sourceFrame==null) {
			//System.err.println("Cannot transform: source frame \"" + resolvedSourceID + "\" does not exist.");
			return null;
		}

		Frame targetFrame = verifyDataAvailable(time, resolvedTargetID);
		if(targetFrame==null) {
			//System.err.println("Cannot transform: target frame \"" + resolvedTargetID + "\" does not exist.");
			return null;
		}

		// list that will contain transformations from source frame to some frame F
		LinkedList<TransformStorage> inverseTransforms = new LinkedList<TransformStorage>();
		// list that will contain transformations from frame F to target frame
		LinkedList<TransformStorage> forwardTransforms = new LinkedList<TransformStorage>();

		// fill the lists using lookupLists. If it returns FALSE, no transformation could be found.
		if (!lookupLists(targetFrame, sourceFrame, time.totalNsecs(), inverseTransforms, forwardTransforms)) {
			//System.err.println("Cannot transform: source + \"" + resolvedSourceID + "\" and target \""
			//		+ resolvedTargetID + "\" are not connected.");
			return null;
		}

		// create an identity transform with the correct time stamp
		StampedTransform out = StampedTransform.getIdentity();
		out.timeStamp = time;

		out.frameID = resolvedTargetID;
		out.childFrameID = resolvedSourceID;

		// multiply all transforms from source frame to frame F
		for(TransformStorage t : inverseTransforms) {
			out.mul(StorageToStampedTransform(t));
		}

		// multiply all transforms from frame F to target frame
		for(TransformStorage t : forwardTransforms) {
			out.mul(StorageToStampedTransform(t).invert(), out);
		}

		// return transform
		return out;
	}

	/**
	 * Check if there are transforms for this time and this frame in the buffer,
	 * try to load from DB otherwise (-> use DB only when needed)
	 *
	 * @param time
	 * @param sourceFrame
	 */
	private Frame verifyDataAvailable(Time time, String frameID) {

		// lookup frame
		Frame frame = frames.get(frameID);

		// check if frame is inside time cache
		if(frame!=null && frame.getParentFrames()!=null) {
			for(Frame f : frame.getParentFrames()) {

				if(frame.getTimeCache(f).timeInBufferRange(new ISODate(time).getMilliSeconds())) {
					return frame;
				}
			}

		}

		// load data from DB if frame is unknown or time not buffered yet
		loadTransformFromDB(frameID, new ISODate(time).getDate());
		frame = frames.get(frameID);
		return frame;

	}


	/**
	 * Load transforms from the DB and add them to the local tf buffer
	 *
	 * @param childFrameID
	 * @param date
	 * @return
	 */
	private StampedTransform loadTransformFromDB(String childFrameID, Date date) {

		DBCollection coll = db.getCollection(tfTableName);
		DBObject query = new BasicDBObject();

		// select time slice from BUFFER_SIZE seconds before to half a second after given time
		Date start = new Date((long) (date.getTime() - ((int) (BUFFER_SIZE * 1000) ) ));
		Date end   = new Date((long) (date.getTime() + 500));

		// read all frames in time slice
		// FIXME: Lookup is to slow.
		query = QueryBuilder.start("transforms.header.stamp").greaterThanEquals( start )
							.and("transforms.header.stamp").lessThan( end )
							.get();
		// TODO: This 'quick' lookup is ok for Yuen's experiment
		//DBObject query = new BasicDBObject("transforms.header.stamp", start);

		// TODO: check if we can read only the latest transforms for the child frame
		// -> should be feasible since verifyDataAvailable should load data when needed,
		//    maybe needs to be made recursive
		// query = QueryBuilder.start("transforms")
		// 		.elemMatch(new BasicDBObject("child_frame_id", childFrameID))
		// 		.and("__recorded").greaterThanEquals( start )
		// 		.and("__recorded").lessThan( end )
		// 		.get();


		// read only transforms
		DBObject cols  = new BasicDBObject();
		cols.put("transforms",  1);

		DBCursor cursor = coll.find(query, cols );

		StampedTransform res = null;
		try {
			while(cursor.hasNext()) {
				setTransforms((BasicDBList) cursor.next().get("transforms"));
			}
		} finally {
			cursor.close();
		}
		
		return res;
	}





	/**
	 * Returns the transform from the specified source frame at sourceTime to the target frame at a given
	 * targetTime, based on a given fixed frame; returns null if no transformation could be found.
	 */
	public StampedTransform lookupTransform(String targetID, Time targetTime, String sourceID, Time sourceTime, String fixedID) {
		// lookup transform from source to fixed frame, at sourceTime
		StampedTransform t1 = lookupTransform(fixedID, sourceID, sourceTime);
		// lookup transform from fixed frame to target frame, at targetTime
		StampedTransform t2 = lookupTransform(targetID, fixedID, targetTime);

		// if either of the two transformations did not succeed, return null
		if (t1 == null || t2 == null) return null;

		// multiply transformation t2 with t1, and return
		t2.mul(t1);
		return t2;
	}

	/**
	 * Performs a bi-directional best-first graph search on the tf graph to try to find a path from sourceFrame
	 * to targetFrame, at the given time. One priority queue is used to keep a sorted list of all search nodes
	 * (from both directions, ordered descending by their potential of contributing to a good solution). At
	 * the moment, the cost of the path from A to B is defined as the largest absolute difference between the
	 * time stamps of the transforms from A to B and the given time point. This corresponds to searching for a
	 * transform path that needs the least amount of inter- and extrapolation.
	 *
	 * Note: often in search, if we talk about expanding a search node, we say that the node expands and its
	 * _children_ are added to the queue. Yet, the tf graph is stored by linking child frames to their _parent_
	 * frames, not the other way around. So, if a search node is expanded, the _parent_ frames are added to the
	 * queue. This may be a bit confusing.
	 */
	protected boolean lookupLists(Frame targetFrame, Frame sourceFrame, long time,
			LinkedList<TransformStorage> inverseTransforms, LinkedList<TransformStorage> forwardTransforms) {
		
		// wrap the source and target frames in search nodes
		SearchNode<Frame> sourceNode = new SearchNode<Frame>(sourceFrame);
		SearchNode<Frame> targetNode = new SearchNode<Frame>(targetFrame);

		// set beginning of forward path (from source)
		sourceNode.backwardStep = sourceNode;
		// set beginning of backward path (form target)
		targetNode.forwardStep = targetNode;

		// create a hash map that map frames to search nodes. This is necessary to keep track of
		// which frames have already been visited (and from which direction).
		HashMap<Frame, SearchNode<Frame>> frameToNode = new HashMap<Frame, SearchNode<Frame>>();

		// add source and target search nodes to the map
		frameToNode.put(sourceFrame, sourceNode);
		frameToNode.put(targetFrame, targetNode);

		// create a priority queue, which will hold the search nodes ordered by cost (descending)
		PriorityQueue<SearchNode<Frame>> Q = new PriorityQueue<SearchNode<Frame>>();

		// at the source and target search nodes to the queue
		Q.add(sourceNode);
		Q.add(targetNode);

		// perform the search
		while(!Q.isEmpty()) {
			// poll most potential search node from queue
			SearchNode<Frame> frameNode = Q.poll();
			Frame frame = frameNode.content;

			// if the node is both visited from the source and from the target node, a path has been found
			if (frameNode.backwardStep != null && frameNode.forwardStep != null) {
				// found the best path from source to target through FRAME.

				// create inverse list (from source to FRAME)
				SearchNode<Frame> node = frameNode;
				while(node.content != sourceNode.content) {

					TransformStorage st = node.backwardStep.content.getData(time, node.content);
					if(st==null) {
						return false;
					} else {
						inverseTransforms.addLast(st);
						node = node.backwardStep;
					}
				}

				// create forward list (from FRAME to target)
				node = frameNode;
				while(node.content != targetNode.content) {

					TransformStorage st = node.forwardStep.content.getData(time, node.content);
					if(st==null) {
						return false;
					} else {
						forwardTransforms.addLast(st);
						node = node.forwardStep;
					}
				}
				return true;
			}

			// expand search node
			for(Frame parentFrame : frame.getParentFrames()) {
				SearchNode<Frame> parentFrameNode = frameToNode.get(parentFrame);

				boolean addToQueue = false;
				if (parentFrameNode == null) {
					// node was not yet visited
					parentFrameNode = new SearchNode<Frame>(parentFrame);
					frameToNode.put(parentFrame, parentFrameNode);
					addToQueue = true;
				} else {
					// node is already visited
					if ((parentFrameNode.backwardStep == null && frameNode.forwardStep == null)
							|| (parentFrameNode.forwardStep == null && frameNode.backwardStep == null)) {
						// node was visited, but from other direction.
						// create new search node that represents this frame, visited from both sides
						// this allows the other search node of this frame to still be expanded first
						parentFrameNode = new SearchNode<Frame>(parentFrameNode);
						addToQueue = true;
					}
				}

				// add search node belonging to parent frame to the queue
				if (addToQueue) {
					// determine cost (based on max absolute difference in time stamp)
					TimeCache cache = frame.getTimeCache(parentFrame);
					parentFrameNode.cost = Math.max((double)cache.timeToNearestTransform(time),
							Math.max(parentFrameNode.cost, frameNode.cost));
					// if visiting forward (from source), set backward step to remember path
					if (frameNode.backwardStep != null) parentFrameNode.backwardStep = frameNode;
					// if visiting backward (from target), set forward step to remember path
					if (frameNode.forwardStep != null) parentFrameNode.forwardStep = frameNode;
					// add node to queue
					Q.add(parentFrameNode);
				}
			}
		}

		// target and source frames are not connected.
		return false;
	}

	/**
	 * Wrapper search node that can be used for bi-directional best-first search.
	 * Keeps track of search path by maintaining links to parent nodes, in both directions
	 * (i.e., from source and from target node).
	 *
	 * @author Sjoerd van den Dries
	 * @param <V> Content type of the search node
	 */
	protected class SearchNode<V> implements Comparable<SearchNode<V>> {
		/** Content of search node */
		V content;
		/** Cost of path up and until this search node */
		double cost;
		/** Refers to parent node in forward path */
		SearchNode<V> backwardStep;
		/** Refers to parent node in backward path */
		SearchNode<V> forwardStep;

		/** Default constructor; sets specified content and cost to 0, steps to null. */
		SearchNode(V content) {
			this.content = content;
			this.cost = 0;
			this.backwardStep = null;
			this.forwardStep = null;
		}

		/** Copy constructor */
		SearchNode(SearchNode<V> orig) {
			this.content = orig.content;
			this.cost = orig.cost;
			this.backwardStep = orig.backwardStep;
			this.forwardStep = orig.forwardStep;
		}

		/** Comparator method: low cost < high cost. */
		public int compareTo(SearchNode<V> other) {
			if (this.cost < other.cost) return -1;
			if (this.cost > other.cost) return 1;
			return 0;
		}

	}

	/* **********************************************************************
	 * *                          HELPER METHODS                            *
	 * ********************************************************************** */

	//	/**
	//	 * Converts the given TransformStamped message to the TransformStorage datastructure
	//	 */
	//	protected TransformStorage transformStampedMsgToTF(TransformStamped msg) {
	//	    ros.pkg.geometry_msgs.msg.Vector3 tMsg = msg.transform.translation;
	//	    ros.pkg.geometry_msgs.msg.Quaternion rMsg = msg.transform.rotation;
	//
	//	    // add frames to map
	//	    Frame childFrame = lookupOrInsertFrame(msg.child_frame_id);
	//	    Frame parentFrame = lookupOrInsertFrame(msg.header.frame_id);
	//
	//	    return new TransformStorage(new Vector3d(tMsg.x, tMsg.y, tMsg.z),
	//	                                new Quat4d(rMsg.x, rMsg.y, rMsg.z, rMsg.w),
	//	                                msg.header.stamp.totalNsecs(),
	//	                                parentFrame, childFrame);
	//	}

	/**
	 * Converts the given TransformStorage datastructure to a TransformStamped message
	 */
	protected TransformStamped TFToTransformStampedMsg(TransformStorage tf) {

		// convert quaternion and translation vector to corresponding messages
		
		NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
		MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
		TransformStamped msg = messageFactory.newFromType(geometry_msgs.TransformStamped._TYPE);
		
		Vector3d tTF = tf.getTranslation();
		Quat4d rTF = tf.getRotation();
		
		geometry_msgs.Vector3 tMsg = msg.getTransform().getTranslation(); 
		geometry_msgs.Quaternion rMsg = msg.getTransform().getRotation();
		tMsg.setX(tTF.x); tMsg.setY(tTF.y); tMsg.setZ(tTF.z);
		rMsg.setX(rTF.x); rMsg.setY(rTF.y); rMsg.setZ(rTF.z); rMsg.setW(rTF.w);

		// create TransformStamped message
		msg.getHeader().setFrameId(tf.getParentFrame().getFrameID());
		msg.getHeader().setStamp(new Time(tf.getTimeStamp()));
		msg.setChildFrameId(tf.getChildFrame().getFrameID());

		return msg;
	}

	/**
	 * Converts the TransformStorage datastructure (represented by quaternion and vector) to
	 * the StampedTransform datastructure (represented by a 4x4 matrix)
	 */
	protected StampedTransform StorageToStampedTransform(TransformStorage ts) {
		return new StampedTransform(ts.getTranslation(), ts.getRotation(), new Time(ts.getTimeStamp()),
				ts.getParentFrame().getFrameID(), ts.getChildFrame().getFrameID());
	}

	/**
	 * Returns the resolved version of the given frame ID, and asserts a debug message if the name
	 * was not fully resolved.
	 */
	private String assertResolved(String prefix, String frameID) {
		//if (!frameID.startsWith("/"))
		//	System.err.println("TF operating on not fully resolved frame id " + frameID +", resolving using local prefix " + prefix);
		return resolve(prefix, frameID);
	}

	/**
	 * Returns the resolves version of the given frame ID.
	 */
	private static String resolve(String prefix, String frameID)	{
		if (frameID.startsWith("/")) {
			return frameID;
		}

		if (prefix.length() > 0) {
			if (prefix.startsWith("/")) {
				return prefix + "/" + frameID;
			} else {
				return "/" + prefix + "/" + frameID;
			}
		}  else {
			return "/" + frameID;
		}
	}
	
	/**
	 * Makes sure that frame name starts with '/'
	 */
	private String getFrameName(Object obj) {
		String out = obj.toString();
		if(out.startsWith("/"))
			return out;
		else
			return "/"+out;
	}
}

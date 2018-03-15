/* 
 * Copyright (c) 2011, Sjoerd van den Dries
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
 */


package tfjava;

import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Subscriber;

import tf.tfMessage;
import geometry_msgs.TransformStamped;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Point3d;
import javax.vecmath.Matrix4d;

import org.ros.message.Duration;

import java.util.HashMap;
import java.util.PriorityQueue;
import java.util.LinkedList;

/**
 * A client that listens to the /tf topic, stores transforms in a buffer and allows transformation
 * lookups from one frame to another.
 * 
 * The listener runs in a separate thread. All tf messages published on the /tf topic are stored in
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
 * @author Sjoerd van den Dries
 * @version March 4, 2011
 */
public class TFListener extends AbstractNodeMain {
    
    /** Maximum buffer storage time */
    public static final long MAX_STORAGE_TIME = (new Duration(10, 0)).totalNsecs(); 
    
    /** The singleton instance */
    protected static TFListener instance;    
    /** Map that maps frame ID's (names) to frames */    
    protected HashMap<String, Frame> frames;
    /** TF name prefix, currently not used (TODO) */
    protected String tfPrefix = "";
    
    protected Node node;
    
    /**
     * Returns the TFListener instance.
     */    
    public synchronized static TFListener getInstance() {
        if (instance == null) {
            instance = new TFListener();
        }
        return instance;
    }
    
    /* **********************************************************************
     * *                           INITIALIZATION                           *
     * ********************************************************************** */     

	  @Override
	  public GraphName getDefaultNodeName() {
	    return GraphName.of("tfjava_listener");
	  }

	
	@Override
	public void onStart(ConnectedNode connectedNode) {

        frames = new HashMap<String, Frame>();  
		node = connectedNode;
		
		Subscriber<tf.tfMessage> subscriber = connectedNode.newSubscriber("/tf", tf.tfMessage._TYPE);
		subscriber.addMessageListener(new MessageListener<tf.tfMessage>() {
			@Override
			public void onNewMessage(tfMessage tfm) {
	            if (tfm != null) {
	                for(TransformStamped tf : tfm.getTransforms()) {                 
	                    setTransform(tf);               
	                }
	            }
			}
		});
	}
	
	
    /* **********************************************************************
     * *                            TF LISTENER                             *
     * ********************************************************************** */	
	
	/**
	 * Converts transform (a geometry msg) to a TransformStorage object and adds it to the buffer.
	 */    
    protected boolean setTransform(TransformStamped transform) {
	    // resolve the frame ID's
        String childFrameID = assertResolved(tfPrefix, transform.getChildFrameId());
	    String frameID = assertResolved(tfPrefix, transform.getHeader().getFrameId());
 
	    boolean errorExists = false;
	    if (childFrameID == frameID) {
	    	node.getLog().error("TF_SELF_TRANSFORM: Ignoring transform with frame_id and child_frame_id  \"" + childFrameID + "\" because they are the same");
	        errorExists = true;
	    }

	    if (childFrameID == "/") { //empty frame id will be mapped to "/"
	    	node.getLog().error("TF_NO_CHILD_FRAME_ID: Ignoring transform because child_frame_id not set ");
	        errorExists = true;
	    }

	    if (frameID == "/") { //empty parent id will be mapped to "/"
	    	node.getLog().error("TF_NO_FRAME_ID: Ignoring transform with child_frame_id \"" + childFrameID + "\" because frame_id not set");
	        errorExists = true;
	    }

	    if (errorExists) return false;	    
	    
	    // lookup or insert child frame
	    Frame frame = lookupOrInsertFrame(childFrameID);
	    
	    // convert tf message to JTransform datastructure
	    transform.setChildFrameId(childFrameID);
	    transform.getHeader().setFrameId(frameID); 
	    TransformStorage tf = transformStampedMsgToTF(transform);
	    
	    // try to insert tf in corresponding time cache. If result is FALSE, the tf contains old data.
	    if (!frame.insertData(tf)) {
	    	node.getLog().warn("TF_OLD_DATA ignoring data from the past for frame \"" + childFrameID + "\" at time " + ((double)tf.getTimeStamp() / 1E9));
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
            frame = new Frame(frameID, MAX_STORAGE_TIME);
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
	public void transformPose(String targetFrameID, Stamped<Matrix4d> stampedIn, Stamped<Matrix4d> stampedOut) {
	    StampedTransform transform = lookupTransform(targetFrameID, stampedIn.frameID, stampedIn.timeStamp);
	    transform.transformPose(stampedIn.getData(), stampedOut.getData());	    
        stampedOut.frameID = targetFrameID;
        stampedOut.timeStamp = stampedIn.timeStamp;
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
	    String resolvedTargetID = assertResolved(tfPrefix, targetFrameID);
        String resolvedSourceID = assertResolved(tfPrefix, sourceFrameID);
        
        // if source and target are the same, return the identity transform
        if (resolvedSourceID == resolvedTargetID) {
            StampedTransform out = StampedTransform.getIdentity();
            out.timeStamp = time;
            out.frameID = resolvedSourceID;
            out.childFrameID = resolvedTargetID;            
            return out;
        }

        // lookup source and target frame
        Frame sourceFrame = frames.get(resolvedSourceID);
        Frame targetFrame = frames.get(resolvedTargetID);
        
        if (sourceFrame == null) {
        	node.getLog().error("Cannot transform: source frame \"" + resolvedSourceID + "\" does not exist.");
            return null;
        }    
            
        if (targetFrame == null) {
        	node.getLog().error("Cannot transform: target frame \"" + resolvedTargetID + "\" does not exist.");
            return null;
        } 
	    
        // list that will contain transformations from source frame to some frame F        
	    LinkedList<TransformStorage> inverseTransforms = new LinkedList<TransformStorage>();
	    // list that will contain transformations from frame F to target frame
	    LinkedList<TransformStorage> forwardTransforms = new LinkedList<TransformStorage>();
	    
	    // fill the lists using lookupLists. If it returns FALSE, no transformation could be found.
	    if (!lookupLists(targetFrame, sourceFrame, time.totalNsecs(), inverseTransforms, forwardTransforms)) {
	        // TODO give warning
	    	node.getLog().error("Cannot transform: source + \"" + resolvedSourceID + "\" and target \""
	                        + resolvedTargetID + "\" are not connected.");
	        return null;
	    }        
	    
	    // create an identity transform with the correct time stamp
	    StampedTransform out = StampedTransform.getIdentity();	    
	    out.timeStamp = time;
	    
        // multiply all transforms from source frame to frame F TODO: right?
        for(TransformStorage t : inverseTransforms) {           
            out.mul(StorageToStampedTransform(t));
        }
	    
	    // multiply all transforms from frame F to target frame TODO: right?
	    for(TransformStorage t : forwardTransforms) {	        
	        out.mul(StorageToStampedTransform(t).invert(), out);
        }	    
	    
	    // return transform
	    return out;
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
                    inverseTransforms.addLast(node.backwardStep.content.getData(time, node.content));
                    node = node.backwardStep;
                }
                
                // create forward list (from FRAME to target)
                node = frameNode;
                while(node.content != targetNode.content) {
                    forwardTransforms.addLast(node.forwardStep.content.getData(time, node.content));
                    node = node.forwardStep;
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
	
	/**
	 * Converts the given TransformStamped message to the TransformStorage datastructure
	 */	
	protected TransformStorage transformStampedMsgToTF(TransformStamped msg) {
	    geometry_msgs.Vector3 tMsg = msg.getTransform().getTranslation();
	    geometry_msgs.Quaternion rMsg = msg.getTransform().getRotation();
	    
	    // add frames to map
	    Frame childFrame = lookupOrInsertFrame(msg.getChildFrameId());
	    Frame parentFrame = lookupOrInsertFrame(msg.getHeader().getFrameId());
	    
	    return new TransformStorage(new Vector3d(tMsg.getX(), tMsg.getY(), tMsg.getZ()),
	                                new Quat4d(rMsg.getX(), rMsg.getY(), rMsg.getZ(), rMsg.getW()),
	                                msg.getHeader().getStamp().totalNsecs(),
	                                parentFrame, childFrame);
	}
	
    /**
     * Converts the given TransformStorage datastructure to a TransformStamped message
     */ 	
	protected TransformStamped TFToTransformStampedMsg(TransformStorage tf) {
	    Vector3d tTF = tf.getTranslation();
	    Quat4d rTF = tf.getRotation();
	    
	    // convert quaternion and translation vector to corresponding messages
	    geometry_msgs.Vector3 tMsg = node.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
	    geometry_msgs.Quaternion rMsg = node.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);
	    tMsg.setX(tTF.x); tMsg.setY(tTF.y); tMsg.setZ(tTF.z);
        rMsg.setX(rTF.x); rMsg.setY(rTF.y); rMsg.setZ(rTF.z); rMsg.setW(rTF.w);
        
        // create TransformStamped message
	    TransformStamped msg = node.getTopicMessageFactory().newFromType(TransformStamped._TYPE);
	    msg.getHeader().setFrameId(tf.getParentFrame().getFrameID());
	    msg.getHeader().setStamp(new Time(tf.getTimeStamp()));
	    msg.setChildFrameId(tf.getChildFrame().getFrameID());
	    msg.getTransform().setTranslation(tMsg);
	    msg.getTransform().setRotation(rMsg);
	    
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
	 * Returns the resolves version of the given frame ID, and asserts a debug message if the name
	 * was not fully resolved.
     */ 	
	private String assertResolved(String prefix, String frameID) {
	  if (!frameID.startsWith("/"))	    
		  node.getLog().debug("TF operating on not fully resolved frame id " + frameID +", resolving using local prefix " + prefix);
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
	
	/* Returns the tf prefix from the parameter list
	 * 
	 * TODO: does not work yet
	private static String getPrefixParam(NodeHandle nh) {
		String param; 
		if (!nh.hasParam("tf_prefix")) return ""; 
		
		try {
			return nh.getStringParam("tf_prefix", false);
		} catch (Exception e) {
			e.printStackTrace();
		}
		return "";
	}
	*/	
	
}

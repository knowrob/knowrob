/*
 * Copyright (c) 2013-15 Moritz Tenorth, Arne Stefes, Daniel Beßler, Zhou Fang
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

package org.knowrob.vis;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Vector;
import java.util.concurrent.ConcurrentHashMap;
import java.util.List;
import java.text.DecimalFormat;

import javax.vecmath.Vector3d;

import org.ros.message.Duration;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.apache.commons.logging.Log;
import org.knowrob.interfaces.mongo.types.Designator;
import org.knowrob.owl.OWLThing;
import org.knowrob.prolog.PrologInterface;
import org.knowrob.tfmemory.TFMemory;
import org.knowrob.vis.meshes.CheckerBoardMesh;
import org.knowrob.vis.meshes.ColladaMesh;
import org.knowrob.vis.meshes.ContourMesh;

import tfjava.StampedTransform;
import visualization_msgs.Marker;
import visualization_msgs.MarkerArray;
import geometry_msgs.Pose;

/**
 * Visualization module for the KnowRob knowledge base
 *
 * The objects to be visualized are published as 'Markers'
 * on the visualization_marker topic and can be visualized
 * in the 'rviz' program.
 * 
 * Note: Make sure to set search index for 'transforms.header.stamp'
 * in mongo: db.tf.ensureIndex( { "transforms.header.stamp" : 1 } )
 *
 * @author tenorth@cs.uni-bremen.de
 * @author danielb@cs.uni-bremen.de
 *
 */
public class MarkerVisualization extends AbstractNodeMain {

	private static final String HTML_RED = "ff0000";

	Publisher<MarkerArray> pub;
	
	Publisher<Pose> cam_pub;
	
	Publisher<std_msgs.String> text_pub;
	
	Publisher<data_vis_msgs.Speech> speech_pub;

	ConnectedNode node;

	static String reference_frame = null; 
	

	/**
	 * Store the markers to be published
	 */
	protected Map<String, Marker> markers;
	/**
	 * Store all added markers (for highlighting)
	 */
	protected Map<String, Marker> markersCache;

	/**
	 * Stores original colors of highlighted objects
	 */
	protected Map<String, float[]> highlighted;

	/**
	 * Counter for marker IDs
	 */
	private static int id = 0;

	/**
	 * Mapping of TF name to trajectory marker names.
	 * E.g., "/RightHand" -> "/RightHand"
	 */
	protected Map<String, List<String>> trajectories;

	/**
	 * Stores the set of joints which are available from the XSens motion
	 * capturing suite.
	 */
	private Map<String, Skeleton> agentSkeletons;

	/**
	 * Logger of ROS node.
	 */
	private Log log;

	/**
	 * Constructor. Starts the marker publisher in a parallel thread.
	 */
	public MarkerVisualization() {
		markers =  new ConcurrentHashMap<String, Marker>(8, 0.9f, 1);
		markersCache =  new ConcurrentHashMap<String, Marker>(8, 0.9f, 1);
		highlighted = new ConcurrentHashMap<String, float[]>(8, 0.9f, 1);
		trajectories = new HashMap<String, List<String>>();
		agentSkeletons = new HashMap<String, Skeleton>();
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		node = connectedNode;
		pub = connectedNode.newPublisher("/visualization_marker_array", visualization_msgs.MarkerArray._TYPE);
		cam_pub = connectedNode.newPublisher("/camera/pose", geometry_msgs.Pose._TYPE);
		text_pub = connectedNode.newPublisher("/canvas/text", std_msgs.String._TYPE);
		speech_pub = connectedNode.newPublisher("/canvas/speech", data_vis_msgs.Speech._TYPE);
		log = connectedNode.getLog();
		reference_frame = node.getParameterTree().getString("knowrob_reference_frame","/map");
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("knowrob_vis");
	}
	

        //Accepts 4x4 Transformation matrix for adjusting camera pose
        public void setCameraTransform(final String[] transforms) {
		try {
			final Pose pose = cam_pub.newMessage();
			
			pose.getPosition().setX(Float.parseFloat(transforms[3]));
			pose.getPosition().setY(Float.parseFloat(transforms[7]));
			pose.getPosition().setZ(Float.parseFloat(transforms[11]));

			float xx = Float.parseFloat(transforms[0]);
			float xy = Float.parseFloat(transforms[1]);
			float xz = Float.parseFloat(transforms[2]);
			float yx = Float.parseFloat(transforms[4]);
			float yy = Float.parseFloat(transforms[5]);
			float yz = Float.parseFloat(transforms[6]);
			float zx = Float.parseFloat(transforms[8]);
			float zy = Float.parseFloat(transforms[9]);
			float zz = Float.parseFloat(transforms[10]);			


			final float t = xx + yy + zz;
			float w,x,y,z;
			if (t >= 0) { // |w| >= .5
				float s = (float)Math.sqrt(t + 1); 
				w = 0.5f * s;
				s = 0.5f / s; 
				x = (zy - yz) * s;
				y = (xz - zx) * s;
				z = (yx - xy) * s;
		  	}else if ((xx > yy) && (xx > zz)) {
				float s = (float)Math.sqrt(1.0 + xx - yy - zz); // |s|>=1
				x = s * 0.5f; // |x| >= .5
				s = 0.5f / s;
				y = (yx + xy) * s;
				z = (xz + zx) * s;
				w = (zy - yz) * s;
			} else if (yy > zz) {
				float s = (float)Math.sqrt(1.0 + yy - xx - zz); // |s|>=1
				y = s * 0.5f; // |y| >= .5
				s = 0.5f / s;
				x = (yx + xy) * s;
				z = (zy + yz) * s;
				w = (xz - zx) * s;
			} else {
				float s = (float)Math.sqrt(1.0 + zz - xx - yy); // |s|>=1
				z = s * 0.5f; // |z| >= .5
				s = 0.5f / s;
				x = (xz + zx) * s;
				y = (zy + yz) * s;
				w = (yx - xy) * s;
			}
					
			pose.getOrientation().setX(w);
			pose.getOrientation().setY(x);
			pose.getOrientation().setZ(y);
			pose.getOrientation().setW(z);
			
			cam_pub.publish(pose);
		}
		catch(Exception e) {
			e.printStackTrace();
		}

        }

	public void setCameraPose(final String[] positions, final String[] orientations) {
		try {
			final Pose pose = cam_pub.newMessage();
			
			pose.getPosition().setX(Float.parseFloat(positions[0]));
			pose.getPosition().setY(Float.parseFloat(positions[1]));
			pose.getPosition().setZ(Float.parseFloat(positions[2]));
			
			pose.getOrientation().setX(Float.parseFloat(orientations[0]));
			pose.getOrientation().setY(Float.parseFloat(orientations[1]));
			pose.getOrientation().setZ(Float.parseFloat(orientations[2]));
			pose.getOrientation().setW(Float.parseFloat(orientations[3]));
			
			cam_pub.publish(pose);
		}
		catch(Exception e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * Add object 'identifier' to the visualization.
	 *
	 * @param identifier OWL identifier of an object instance
	 * @param timepoint  OWL identifier of a timepoint instance
	 */
	public void addObject(String identifier, String timepoint) {
		try {
			addMarker(identifier, timepoint);

			publishMarkers();
		} catch(Exception e) {
			e.printStackTrace();
		}

	}
	/**
	 * Add each object in 'identifiers' to the visualization.
	 *
	 * @param identifiers  Array of OWL identifier of an object instance
	 * @param timepoint    OWL identifier of a timepoint instance
	 */
	public void addObjects(String[] identifiers, String timepoint) {
		try {
			for(String i : identifiers) {
				addMarker(i, timepoint);	
			}
			
			publishMarkers();
		} catch(Exception e) {
			e.printStackTrace();
		}

	}


	/**
	 * Add object 'identifier' and all its parts to the visualization.
	 *
	 * @param identifier OWL identifier of an object instance
	 * @param timepoint  OWL identifier of a timepoint instance
	 */
	public void addObjectWithChildren(String identifier, String timepoint) {
		try {
			// add this object
			addMarker(identifier, timepoint);

			// read children and add them too
			for(String child : readChildren(identifier))
				addMarker(child, timepoint);

			publishMarkers();
		}
		catch(Exception e) {
			e.printStackTrace();
		}

	}

	/**
	 * Update object 'identifier' in the visualization.
	 *
	 * @param identifier OWL identifier of an object instance
	 * @param timepoint  OWL identifier of a timepoint instance
	 */
	public void updateObject(String identifier, String timepoint) {
		try {
			updateMarker(identifier, timepoint);

			publishMarkers();
		} catch(Exception e) {
			e.printStackTrace();
		}

	}
	
	/**
	 * Update object 'identifier' and all its parts in the visualization.
	 *
	 * @param identifier OWL identifier of an object instance
	 * @param timepoint  OWL identifier of a timepoint instance
	 */
	public void updateObjectWithChildren(String identifier, String timepoint) {
		try {
			// update this object
			updateMarker(identifier, timepoint);

			// read children and update them too
			for(String child : readChildren(identifier))
				updateMarker(child, timepoint);

			publishMarkers();
		}
		catch(Exception e) {
			e.printStackTrace();
		}

	}


	/**
	 * Remove object 'identifier' from the visualization.
	 *
	 * @param identifier OWL identifier of an object instance
	 */
	public void removeObject(String identifier) {
		try {
			// remove the object from the list
			Marker m = eraseMarker(identifier);
			if(m!=null) {
				final MarkerArray arr = pub.newMessage();
				arr.getMarkers().add(m);
				pub.publish(arr);
			}
		} catch(Exception e) {
			e.printStackTrace();
		}

	}

	/**
	 * Remove the object 'identifier' as well as its children
	 *
	 * @param identifier OWL identifier of an object instance
	 */
	public void removeObjectWithChildren(String identifier) {
		try {
			final MarkerArray arr = pub.newMessage();
			// remove this object
			Marker m = eraseMarker(identifier);
			if(m!=null) arr.getMarkers().add(m);

			// remove children and remove them too
			for(String child : readChildren(identifier)) {
				Marker c = eraseMarker(child);
				if(c!=null) arr.getMarkers().add(c);
			}

			pub.publish(arr);
		}
		catch(Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Remove all objects from the visualization
	 */
	public void clear() {
		try {
			// wait for node to be ready
			waitForNode();
			
			final MarkerArray arr = pub.newMessage();
			synchronized (markersCache) {
				for(Marker m : markersCache.values()) {
					m.setAction(Marker.DELETE);
					arr.getMarkers().add(m);
				}
				markersCache.clear();
			}
			synchronized (markers) {
				markers.clear();
			}
			synchronized (trajectories) {
				trajectories.clear();
			}
			synchronized (highlighted) {
				highlighted.clear();
			}
			pub.publish(arr);
		}
		catch(Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Highlight the object 'identifier' in red
	 *
	 * @param identifier OWL identifier of an object instance
	 * @param highlight True to set, False to remove highlight
	 */
	public void highlight(String identifier) {
		highlight(identifier, 255, 0, 0, 122);
	}

	/**
	 * Highlight the object 'identifier' in 'color'
	 *
	 * @param identifier OWL identifier of an object instance
	 * @param highlight True to set, False to remove highlight
	 * @param color String of the form #RRGGBB
	 */
	public void highlight(String identifier, String color) {
		int col = Integer.valueOf(color, 16);

		highlight(identifier, col);
	}
	
	/**
	 * Highlight the object 'identifier' in 'color'
	 *
	 * @param identifier OWL identifier of an object instance
	 * @param highlight True to set, False to remove highlight
	 * @param col integer representation of ##RRGGBB
	 */
	public void highlight(String identifier, int col) {
		
		int r = (col & 0xff0000) >> 16;
		int g = (col & 0x00ff00) >> 8;
		int b = (col & 0x0000ff);
		
		highlight(identifier, r, g, b, 255);
	}


	public void highlightMesh(String identifier) {
		highlightMesh(identifier, "FF0000");
	}

	public void highlightMesh(String identifier, String color) {
		int col = Integer.valueOf(color, 16);
		int r = (col & 0xff0000) >> 16;
		int g = (col & 0x00ff00) >> 8;
		int b = (col & 0x0000ff);
		int a = 125;
		highlightMesh(identifier,  r, g, b, a);
	}

	public void highlightMesh(String identifier, int r, int g, int b, int a) {
		final Marker ref_marker = markersCache.get(identifier);
		if(ref_marker==null){
			System.out.println("Refmarker "+identifier+"does not exist");
			return;
		}
		String m_id = identifier +"_highlight";
		Marker m = markers.get(m_id);
		
		if(m==null) {
			m = node.getTopicMessageFactory().newFromType(visualization_msgs.Marker._TYPE);
		}
		m.getHeader().setFrameId(MarkerVisualization.getReferenceFrame());
		m.setId(ref_marker.getId()+100000);//FIXME This should be safe as long as there are less then 100000 markers. But it should be fixed
		m.setNs(ref_marker.getNs());
		m.setMeshResource(ref_marker.getMeshResource());
		m.setMeshUseEmbeddedMaterials(false);
		m.setAction(0);//Add or change object...
		m.getColor().setR(((float)r)/255.0f);
		m.getColor().setG(((float)g)/255.0f);
		m.getColor().setB(((float)b)/255.0f);
		m.getColor().setA(((float)a)/255.0f);
		m.setPose(ref_marker.getPose());
		m.setScale(ref_marker.getScale());
		m.setType(ref_marker.getType());
		synchronized (markers) {
				markers.put(m_id,m);
		}
		synchronized (markersCache) {
			markersCache.put(m_id,m);
		}
		publishMarkers();
	}
	
	/**
	 * Highlight the object or trajectory that corresponds to
	 * the specified identifier with given color.
	 *
	 * @param identifier OWL identifier of an object instance
	 * @param highlight True to set, False to remove highlight
	 * @param r Red value (0--255)
	 * @param g Green value (0--255)
	 * @param b Blue value (0--255)
	 * @param a Alpha value (0--255)
	 */
	public void highlight(String identifier, int r, int g, int b, int a) {
		try {
		boolean success = __highlight__(identifier, r, g, b, a);
		// Try to highlight trajectory
		final List<String> traj = trajectories.get(identifier);
		if(traj!=null) {
			for(final String x : traj) {
				if(__highlight__(x, r, g, b, a)) success = true;
			}
		}
		if(success) {
			publishMarkers();
		}
		else {
			// XXX logging to console
			System.err.println("Unable to find marker for identifier '" + identifier + "'.");
			log.warn("Unable to find marker for identifier '" + identifier + "'.");
		}
		} catch(Exception e) {
			e.printStackTrace();
		}

	}

	boolean __highlight__(String identifier, int r, int g, int b, int a) {
		
		//Asil: using Ben's way temporarily
		final Marker m = markersCache.get(identifier);//add special case for mesh...
		if(m==null) return false;
		__highlightMesh__(m, ((float) r/255), ((float) g/255) ,((float) b/255) ,((float) a/255));
		return true;
		

		//todo(Asil): Investigate what turns the canvas into a white space in the good old highlighting way down below
		/*final Marker m = markersCache.get(identifier);//add special case for mesh...
		if(m==null) return false;
		// Remember default color
		if(!highlighted.containsKey(identifier)) {
			float val[] = new float[4];
			val[0] = m.getColor().getR();
			val[1] = m.getColor().getG();
			val[2] = m.getColor().getB();
			val[3] = m.getColor().getA();
			synchronized (highlighted) {
				highlighted.put(identifier, val);
			}
		}
		// Set highlight color
		m.getColor().setR(((float) r)/255);
		m.getColor().setG(((float) g)/255);
		m.getColor().setB(((float) b)/255);
		m.getColor().setA(((float) a)/255);
		synchronized (markers) {
			markers.put(identifier, m);
		}
		return true;*/
	}

	

	public void __highlightMesh__(Marker ref_marker,float r , float g ,float b ,float a) {
		
		//final Marker ref_marker = markersCache.get(identifier);
		if(ref_marker==null){
			System.out.println("Refmarker does not exist");
			return;
		}
		String m_id = ref_marker.getId() +"_highlight";
		Marker m = markers.get(m_id);
		
		if(m==null) {
			m = node.getTopicMessageFactory().newFromType(visualization_msgs.Marker._TYPE);
		}
		m.getHeader().setFrameId(MarkerVisualization.getReferenceFrame());
		m.setId(ref_marker.getId()+100000);//FIXME This should be safe as long as there are less then 100000 markers. But it should be fixed
		m.setNs(ref_marker.getNs());
		m.setMeshResource(ref_marker.getMeshResource());
		m.setMeshUseEmbeddedMaterials(false);
		m.setAction(0);//Add or change object...
		m.getColor().setR(r);
		m.getColor().setG(g);
		m.getColor().setB(b);
		m.getColor().setA(a);
		m.setPose(ref_marker.getPose());
		m.setScale(ref_marker.getScale());
		m.setType(ref_marker.getType());
		synchronized (markers) {
				markers.put(m_id,m);
		}
		synchronized (markersCache) {
			markersCache.put(m_id,m);
		}
		publishMarkers();
		
		
	}
	
	public void removeMeshHighlight(String identifier) {
		String m_id = identifier +"_highlight";
		Marker m = markersCache.remove(m_id);
		if(m !=null){
			
			m.setAction(2);//REMOVE Marker
			markers.put(m_id, m);
			publishMarkers();
		}
		
	}
	
	/**
	 * Remove object highlight.
	 */
	public void removeHighlight(String identifier) {
		boolean success = __removeHighlight__(identifier);
		// Try to highlight trajectory
		final List<String> traj = trajectories.get(identifier);
		if(traj!=null) {
			for(final String x : traj) {
				if(__removeHighlight__(x)) success = true;
			}
		}
		if(success) {
			publishMarkers();
		}
		else {
			log.warn("Unable to find marker for identifier '" + identifier + "'.");
		}
	}

	public boolean __removeHighlight__(String identifier) {
		final Marker m = markersCache.get(identifier);
		if(m==null) return false;

		final float[] col = highlighted.get(identifier);
		if(col==null) return false;

		synchronized (markers) {
			// Set default color
			m.getColor().setR(col[0]);
			m.getColor().setG(col[1]);
			m.getColor().setB(col[2]);
			m.getColor().setA(col[3]);
			markers.put(identifier, m);
		}
		synchronized (highlighted) {
			highlighted.remove(identifier);
		}

		return true;
	}

	/**
	 * Remove highlight of object and children.
	 */
	public void removeHighlightWithChildren(String identifier) {
		// remove this object
		removeHighlight(identifier);
		// remove children and highlight them too
		for(String child : readChildren(identifier)) {
			removeHighlight(child);
		}
		publishMarkers();
	}

	/**
	 * Clear all highlights
	 */
	public void clearHighlight() {
		// reset colors to cached original ones
		synchronized (highlighted) {
			for(String identifier : highlighted.keySet()) {
				final Marker m = markersCache.get(identifier);
				if(m==null) {
					log.warn("Unable to find marker for identifier '" + identifier + "'.");
					continue;
				}
				final float[] col = highlighted.get(identifier);
				// Set default color
				m.getColor().setR(col[0]);
				m.getColor().setG(col[1]);
				m.getColor().setB(col[2]);
				m.getColor().setA(col[3]);
				synchronized (markers) {
					markers.put(identifier, m);
				}
			}
			highlighted.clear();
		}
		publishMarkers();
	}

	/**
	 * Highlight the object 'identifier' and its children in red
	 *
	 * @param identifier OWL identifier of an object instance
	 * @param highlight True to set, False to remove highlight
	 */
	public void highlightWithChildren(String identifier) {
		highlightWithChildren(identifier, HTML_RED);
	}

	/**
	 * Highlight the object 'identifier' and its children in the given color
	 *
	 * @param identifier OWL identifier of an object instance
	 * @param highlight True to set, False to remove highlight
	 * @param color Integer of the form #AARRGGBB
	 */
	public void highlightWithChildren(String identifier, String color) {
		// remove this object
		highlight(identifier, color);

		// remove children and highlight them too
		for(String child : readChildren(identifier)) {
			highlight(child, color);
		}
		publishMarkers();
	}
	
	/**
	 * Show average trajectory in visualization canvas.
	 * Averages by dividing each trajectory in equal segments and averaging the positions at each point of interest
	 *
	 * @param tflink TF identifier
	 * @param array of starttime OWL identifiers of a timepoint instance
	 * @param array of endtime OWL identifiers of a timepoint instance
	 * @param interval in how many segments the trajectory should be segmented (i.e. basically says how many markers there will be). For multiple trajectories this makes more sense, because trajectories will differ in length 
	 * @param markertype marker type id (see ROS Marker message)
	 */
	public void showAverageTrajectory(String tflink, String[] starttimes, String[] endtimes, int intervalsegments, int markertype) {
		String identifier, formattedTime;
		String tflink_ = (tflink.startsWith("/") ? tflink : "/"+tflink);
//		for(String test : starttimes)
//		{
//			System.out.println(test);
//		}
//		for(String test : endtimes)
//		{
//			System.out.println(test);
//		}
		if(starttimes.length != endtimes.length)
		{
			System.out.println("Specified start and endtimes of intervals are of different length");
		}
		else
		{
			final LinkedList<String> out = new LinkedList<String>();
			//for every timepoint of interest in the interval
			for(int isegment=0; isegment<=intervalsegments; isegment++)
			{
				//initialize marker
				Marker m = null;
				//for every exp/trajectory to be taken into account for the average
				for(int itraj=0; itraj<starttimes.length; itraj++)
				{
					double t0 = parseTime_d(starttimes[itraj]);
					double t1 = parseTime_d(endtimes[itraj]);
					//how long is one segment in the current trajectory?
					double segment_length = (t1-t0)/intervalsegments;
					//the time we'd like to currently request from every trajectory:
					double itime = t0+isegment*segment_length;
					formattedTime = new DecimalFormat("###.###").format(itime);
					// read marker from Prolog
					Marker tempm = null;
					try {
						tempm = readLinkMarkerFromProlog(tflink_, "'timepoint_" + formattedTime + "'", markertype);
					}
					catch(Exception e) {
						log.warn("Unable to read marker for time point '" + formattedTime + "'.", e);
					}
					if(tempm==null) continue;
					else if(itraj==0) //the current marker is the first in line
					{
						assert m==null;
						m = tempm;
					}
					else //average positions
					{
						double avgx = (m.getPose().getPosition().getX()*itraj+tempm.getPose().getPosition().getX())/(itraj+1);
						double avgy = (m.getPose().getPosition().getY()*itraj+tempm.getPose().getPosition().getY())/(itraj+1);
						double avgz = (m.getPose().getPosition().getZ()*itraj+tempm.getPose().getPosition().getZ())/(itraj+1);
						double avgw = (m.getPose().getOrientation().getW()*itraj+tempm.getPose().getOrientation().getW())/(itraj+1);
						double avgxr = (m.getPose().getOrientation().getX()*itraj+tempm.getPose().getOrientation().getX())/(itraj+1);
						double avgyr = (m.getPose().getOrientation().getY()*itraj+tempm.getPose().getOrientation().getY())/(itraj+1);
						double avgzr = (m.getPose().getOrientation().getZ()*itraj+tempm.getPose().getOrientation().getZ())/(itraj+1);

						m.getPose().getPosition().setX(avgx);
						m.getPose().getPosition().setY(avgy);
						m.getPose().getPosition().setZ(avgz);
						m.getPose().getOrientation().setW(avgw);
						m.getPose().getOrientation().setX(avgxr);
						m.getPose().getOrientation().setY(avgyr);
						m.getPose().getOrientation().setZ(avgzr);
					}
				}
				// add marker to map
				identifier = tflink_ + "_avg_" +isegment;

				out.add(identifier);
				synchronized (markers) {
					markers.put(identifier, m);
				}
				synchronized (markersCache) { 
					markersCache.put(identifier, m);
				}
			}
			synchronized (trajectories) {
				trajectories.put(tflink_, out);
			}
			publishMarkers();
		}
	}

	/**
	 * Show trajectory in visualization canvas.
	 *
	 * @param tflink TF identifier
	 * @param starttime OWL identifier of a timepoint instance
	 * @param endtime OWL identifier of a timepoint instance
	 * @param interval in seconds
	 * @param markertype marker type id (see ROS Marker message)
	 */
	public void showTrajectory(String tflink, String starttime, String endtime, double interval, int markertype) {
		String identifier, formattedTime;
		String tflink_ = (tflink.startsWith("/") ? tflink : "/"+tflink);

		double t0 = parseTime_d(starttime);
		double t1 = parseTime_d(endtime);

		final LinkedList<String> out = new LinkedList<String>();
		for (double i = t0; i <= t1; i += interval) {
			formattedTime = new DecimalFormat("###.###").format(i);
			// read marker from Prolog
			Marker m = null;
			try {
				m = readLinkMarkerFromProlog(tflink_, "'timepoint_" + formattedTime + "'", markertype);
			}
			catch(Exception e) {
				log.warn("Unable to read marker for time point '" + formattedTime + "'.", e);
			}
			if(m==null) continue;

			// add marker to map
			identifier = tflink_ + formattedTime;

			out.add(identifier);
			synchronized (markers) {
				markers.put(identifier, m);
			}
			synchronized (markersCache) { 
				markersCache.put(identifier, m);
			}
		}
		synchronized (trajectories) {
			trajectories.put(tflink_, out);
		}

		publishMarkers();
	}
	/**
	 * Show trajectory in visualization canvas.
	 *
	 * @param tflink TF identifier
	 * @param starttime OWL identifier of a timepoint instance
	 * @param endtime OWL identifier of a timepoint instance
	 * @param interval in seconds
	 */
	public void showTrajectory(String tflink, String starttime, String endtime, double interval) {
		showTrajectory(tflink, starttime, endtime, interval, Marker.ARROW);
	}

	/**
	 * @param tflink The name of the TF frame
	 * @return Array of marker identifier that correspond to a TF frame
	 */
	public String[] getTrajectoryMarker(String tflink) {
		String tflink_ = (tflink.startsWith("/") ? tflink : "/"+tflink);
		List<String> out = trajectories.get(tflink_);
		if(out==null) {
			return new String[]{};
		}
		else {
			return out.toArray(new String[out.size()]);
		}
	}

	/**
	 * @param tflink The name of the TF frame
	 * @param starttime Returned marker identifiers must correspond to a later time
	 * @param endtime Returned marker identifiers must correspond to a earlier time
	 * @return Array of marker identifier that correspond to a TF frame
	 */
	public String[] getTrajectoryMarker(String tflink, String starttime, String endtime) {
		String tflink_ = (tflink.startsWith("/") ? tflink : "/"+tflink);
		double t0 = parseTime_d(starttime);
		double t1 = parseTime_d(endtime);
		List<String> traj = trajectories.get(tflink_);
		if(traj==null) {
			return new String[]{};
		}
		else {
			List<String> out = new LinkedList<String>();
			for(String x : traj) {
				double t = Double.parseDouble(x.substring(tflink_.length()));
				if(t0<=t && t<=t1) {
					out.add(x);
				}
			}
			return out.toArray(new String[out.size()]);
		}
	}

	/**
	 * Remove trajectory markers.
	 *
	 * @param tflink TF identifier
	 */
	public void removeTrajectory(String tflink) {
		
		// wait for node to be ready
		waitForNode();
		
		final List<String> traj = trajectories.get(tflink);
		if(traj == null) {
			return;
		}

		final MarkerArray arr = pub.newMessage();

		for(final String identifier : traj) {
			final Marker m = markersCache.get(identifier);
			if(m==null) {
				continue;
			}
			m.setAction(Marker.DELETE);
			arr.getMarkers().add(m);
		}

		synchronized (trajectories) {
			trajectories.remove(tflink);
		}

		pub.publish(arr);
	}
	
	public void clearTrajectories() {
		List<String> keys = new LinkedList<String>();
		keys.addAll(trajectories.keySet());
		for(String link : keys) {
			removeTrajectory(link);
		}
	}
         /**
	 * Show pointer in visualization canvas.
	 *
	 * @param tfSlink TF identifier, source link
	 * @param tfElink TF identifier, target link
	 * @param starttime OWL identifier of a timepoint instance
	 * @param markertype marker type id (see ROS Marker message)
	 */
         public void addPointer(String tfSlink, String tfElink, String timepoint) {
	     try {
		 StampedTransform tr = TFMemory.getInstance().lookupTransform(tfSlink, tfElink, parseTime(timepoint));
		 Marker m = null;
		 if(m == null) {
		     m = createMarker();
		     m.setType(Marker.ARROW);
		     m.setMeshUseEmbeddedMaterials(true);
		 }

		 if(tr==null) {
		     log.warn("TF data missing for '" + tfSlink + "' "+ " and '" + tfElink + "' " + timepoint + " missing in mongo.");
		     return;
		 }
		 else {
		     m.getPose().getPosition().setX(tr.getTranslation().x);
		     m.getPose().getPosition().setY(tr.getTranslation().y);
		     m.getPose().getPosition().setZ(tr.getTranslation().z);
		     m.getPose().getOrientation().setW(tr.getRotation().w);
		     m.getPose().getOrientation().setX(tr.getRotation().x);
		     m.getPose().getOrientation().setY(tr.getRotation().y);
		     m.getPose().getOrientation().setZ(tr.getRotation().z);
		     
		     m.getScale().setX(0);
		     m.getScale().setY(0);
		     m.getScale().setZ(1.0);
		     
		     m.getColor().setR(0.5f);
		     m.getColor().setG(0.5f);
		     m.getColor().setB(1.0f);
		     m.getColor().setA(1.0f);
		 }
		 //add marker to map
		 final StringBuilder identifier = new StringBuilder();
		 identifier.append(m.getNs()).append('_').append(m.getId());
		 
		 synchronized(markers) {
		     markers.put(identifier.toString(),m);
		 }
		 
		 synchronized(markersCache) {
		     markersCache.put(identifier.toString(),m);
		 }
		 
		 publishMarkers();
	     }
	     catch(Exception e){
		 e.printStackTrace();
		 return;
	     }
	 }

    


	// // // // // // // // // // // // // // // // // // // // // // // // // // //
	//
	// Agent visualization
	//
	// TODO: bbrieber: do we really need tfSuffixes. I have never seen something like this in the ROS world...
	public void visualizeAgent(
			String identifier, String individualName,
			String timepoint, String tfSuffix,String tfPrefix,
			int creatCylindersBetweenLinks)
	{
		// wait for node to be ready
		waitForNode();
		if(!tfPrefix.startsWith("/")) {
			tfPrefix = "/"+tfPrefix;//add the leading '/' if needed
		}
		if(tfPrefix.endsWith("/")) {
			tfPrefix = tfPrefix.substring(0, tfPrefix.length()-1);//remove the trailing '/' if needed
		}
		// Lookup skeletal structure
		final Skeleton skeleton;
		try {
			skeleton = getIndividualSkeleton("'" + individualName + "'");
			if(creatCylindersBetweenLinks!=0) {
				skeleton.setDefaultColor(1.0f, 1.0f, 0.0f, 1.0f);
				skeleton.setMarkerType(Marker.SPHERE);
			}
		}
		catch(Exception exc) {
			// TODO: Why is log.error() not printed to console? And where is it printed to?
			System.err.println("Failed to initialize skeleton.");
			System.err.println(exc.getMessage());
			exc.printStackTrace();
			return;
		}

		final Time time = parseTime(timepoint);

		try {
			for(Skeleton.Link sourceLink : skeleton.getLinks()) {
				final Skeleton.StampedLink sl0 = new Skeleton.StampedLink(identifier,sourceLink,time,tfSuffix,tfPrefix);

				if(sl0.link.hasVisual && !addAgentMarker(skeleton.updateLinkMarker(node,sl0))) {
					log.warn("Unable to create marker for '" + sourceLink.sourceFrame + "'.");
				}

				if(creatCylindersBetweenLinks!=0) {
					for(String conn : sourceLink.succeeding) {
						final Skeleton.Link targetLink = skeleton.getLink(conn);
						if(targetLink==null) {
							log.warn("Link not known '" + conn + "'. Source Link: " + sourceLink.sourceFrame);
							continue;
						}
						final Skeleton.StampedLink sl1 = new Skeleton.StampedLink(identifier,targetLink,time,tfSuffix,tfPrefix);
	
						if(sl0.link.hasVisual && sl1.link.hasVisual &&
						  !addAgentMarker(skeleton.createCylinderMarker(node,sl0,sl1))) {
							System.err.println("Unable to create cylinder marker between '" +
									sourceLink.sourceFrame + "' and '" + conn + "'.");
						}
					}
				}
			}
		}
		catch(Exception exc) {
			// TODO: Why is log.error() not printed to console? And where is it printed to?
			System.err.println("Failed to add markers for skeleton.");
			System.err.println(exc.getMessage());
			return;
		}

		publishMarkers();
	}

	boolean addAgentMarker(Marker marker) {
		// wait for node to be ready
		waitForNode();
		
		if(marker!=null) {
			final StringBuilder identifier = new StringBuilder();
			identifier.append(marker.getNs()).append('_').append(marker.getId());

			synchronized (markers) {
				markers.put(identifier.toString(), marker);
			}
			synchronized (markersCache) {
				markersCache.put(identifier.toString(), marker);
			}
			return true;
		}
		else {
			return false;
		}
	}

	/**
	 * Removes markers that belong to an agent individual.
	 */
	public void removeAgent(String identifier, String individual) {
		try {
			// wait for node to be ready
			waitForNode();
			
			Skeleton skel = agentSkeletons.get("'" + individual + "'");
			if(skel == null) {
				System.err.println("No agent known: " + individual);
				return;
			}

			final MarkerArray arr = pub.newMessage();
			
			for(Entry<String, Marker> e : skel.getMarkers()) {
				System.err.println("Removing " + e.getKey());
				e.getValue().setAction(Marker.DELETE);
				arr.getMarkers().add(e.getValue());
			}

			pub.publish(arr);
		}
		catch (Exception e) {
			System.err.println("Failed to remove agent: " + e.getMessage());
			e.printStackTrace();
		}
	}
	
	Skeleton getIndividualSkeleton(String individualName) {
		Skeleton out = agentSkeletons.get(individualName);
		if(out==null) {
			out = new Skeleton(individualName, markersCache);
			agentSkeletons.put(individualName, out);
		}
		return out;
	}

	// // // // // // // // // // // // // // // // // // // // // // // // // // //
	//
	// Mesh rendering
	//
	
	public void addDesignatorContourMesh(String markerId, Designator designator, String timepoint, String colorStr[]) {
		try {
			Vector3d color = new Vector3d(
				Double.valueOf(colorStr[0]),
				Double.valueOf(colorStr[1]),
				Double.valueOf(colorStr[2])
			);
			List<Vector3d> contourPointsCamRel = ContourMesh.getContourPoints(designator);
			List<Vector3d> contourPoints = new LinkedList<Vector3d>();
			
			// FIXME: Hacky method for getting time since TF data recorded that way for pizza rolling
			double posix_ts = parseTime_d(timepoint)*1000000000;
			Time time = new Time();
			time.secs = (int)posix_ts;
			time.nsecs = (int) (1E9 * (posix_ts - ((int) posix_ts)));

			// FIXME: use parameter instead
			String sourceFrame = "/head_mount_kinect2_rgb_optical_frame";
			System.err.println("Displaying contour mesh for timepoint: " + timepoint);
                        System.err.println(time);
			
			// transform to /map
			for(Vector3d camP : contourPointsCamRel) {
				StampedTransform tr = TFMemory.getInstance().lookupTransform(sourceFrame, reference_frame, time);
				if(tr==null) {
					log.warn("TF data missing for '" + sourceFrame + "' " + timepoint + " missing in mongo.");
					return;
				}
				Vector3d p_out = new Vector3d();
				double[] middleTransform = new double[3];
				 
				tr.transformVector(camP, p_out);
				p_out.get(middleTransform);
				middleTransform[0] = -1 * middleTransform[0] - 1.02;
				middleTransform[1] = -1 * middleTransform[1] + 0.27;
				middleTransform[2] = -1 * middleTransform[2] + 0.1;
				p_out.set(middleTransform);
				contourPoints.add(p_out);
			}
			
			// compute extends
			Vector3d min = new Vector3d(contourPoints.get(0));
			Vector3d max = new Vector3d(contourPoints.get(0));
			for(Vector3d p : contourPoints) {
				if(p.x<min.x) min.x=p.x;
				else if(p.x>max.x) max.x=p.x;
				if(p.y<min.y) min.y=p.y;
				else if(p.y>max.y) max.y=p.y;
				if(p.z<min.z) min.z=p.z;
				else if(p.z>max.z) max.z=p.z;
			}

			Marker m = markersCache.get(markerId);
			if(m==null) {
				m = createMarker();
				m.setType(Marker.CUBE);
				m.getColor().setR(new Double(color.x).floatValue());
				m.getColor().setG(new Double(color.y).floatValue());
				m.getColor().setB(new Double(color.z).floatValue());
				m.getColor().setA(1.0f);
			}
	
			m.getPose().getPosition().setX(0.5*(min.x+max.x));
			m.getPose().getPosition().setY(0.5*(min.y+max.y));
			m.getPose().getPosition().setZ(0.5*(min.z+max.z));

			m.getPose().getOrientation().setW(1.0);
			m.getPose().getOrientation().setX(0.0);
			m.getPose().getOrientation().setY(0.0);
			m.getPose().getOrientation().setZ(0.0);
			
			m.getScale().setX(max.x-min.x);
			m.getScale().setY(max.y-min.y);
			m.getScale().setZ(max.z-min.z);
	
			// add marker to map
			synchronized (markers) {
				markers.put(markerId, m);
			}
			synchronized (markersCache) {
				markersCache.put(markerId, m);
			}
			
			publishMarkers();
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	
	public double getDesignatorContourSize(Designator designator) {
		try {
			List<Vector3d> contourPointsCamRel = ContourMesh.getContourPoints(designator);
			
			Vector3d min = new Vector3d(contourPointsCamRel.get(0));
			Vector3d max = new Vector3d(contourPointsCamRel.get(0));
			for(Vector3d p : contourPointsCamRel) {
				if(p.x<min.x) min.x=p.x;
				else if(p.x>max.x) max.x=p.x;
				if(p.y<min.y) min.y=p.y;
				else if(p.y>max.y) max.y=p.y;
				if(p.z<min.z) min.z=p.z;
				else if(p.z>max.z) max.z=p.z;
			}
			
			double size = Math.abs(max.x - min.x) * Math.abs( max.y - min.y);// * Math.abs(max.z - min.z);
			return size *10000;
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		
		return 0.0;
	}
	
	public void addDesignatorCheckerboardMesh(String markerId, Designator designator) {
		// TODO
		try {
			ColladaMesh m = CheckerBoardMesh.createCheckerBoardMesh(designator);
			//ProfileCOMMON profile = m.setPhongMaterial(
			//		new double[] {0.0, 0.0, 0.0, 1.0},
			//		new double[] {0.137255, 0.403922, 0.870588, 1},
			//		new double[] {0.5, 0.5, 0.5, 1});
			
			//String imgPath = "../kitchen/food-drinks/pizza/pizza_sauce_DIFF.png";
			//m.addDiffuseTexturePhong(profile, "tomato-sauce-diff", "UVMap", imgPath);
			
			//String meshPath = m.marshal("blue-cube-" + new Long(System.currentTimeMillis()).toString());
			
			//addMeshMarker(meshPath, "'"+meshPath+"'", position, rotation, scale);
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	// TODO: remove this
	/*
	public void addBlueBoxMarker(String position[], String rotation[], String scale[]) {
		try {
			//ColladaMesh m = ColladaMesh.createCube();
			ColladaMesh m = CheckerBoardMesh.createCheckerBoardMesh();
			ProfileCOMMON profile = m.setPhongMaterial(
					new double[] {0.0, 0.0, 0.0, 1.0},
					new double[] {0.137255, 0.403922, 0.870588, 1},
					new double[] {0.5, 0.5, 0.5, 1});
			
			String imgPath = "../kitchen/food-drinks/pizza/pizza_sauce_DIFF.png";
			m.addDiffuseTexturePhong(profile, "tomato-sauce-diff", "UVMap", imgPath);
			
			String meshPath = m.marshal("blue-cube-" + new Long(System.currentTimeMillis()).toString());
			
			addMeshMarker(meshPath, "'"+meshPath+"'", position, rotation, scale);
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	*/
	
	public void addMeshMarker(String markerId, String meshPath) {
		addMeshMarker(markerId, meshPath,
				new String[] {"0", "0", "0"},
				new String[] {"1", "0", "0", "0"},
				new String[] {"0", "0", "0"});
	}
	
	public void addMeshMarker(String markerId, String meshPath, String position[], String rotation[], String scale[]) {
		try {
			Marker m = markersCache.get(markerId);
			if(m==null) {
				m = createMarker();
				m.setType(Marker.MESH_RESOURCE);
				m.setMeshResource(OWLThing.removeSingleQuotes(meshPath));
				m.setMeshUseEmbeddedMaterials(true);
				// Color must be set to zero for mesh textures
				m.getColor().setR(0.0f);
				m.getColor().setG(0.0f);
				m.getColor().setB(0.0f);
				m.getColor().setA(0.0f);
			}
	
			m.getPose().getPosition().setX(Float.parseFloat(position[0]));
			m.getPose().getPosition().setY(Float.parseFloat(position[1]));
			m.getPose().getPosition().setZ(Float.parseFloat(position[2]));

			m.getPose().getOrientation().setW(Float.parseFloat(rotation[0]));
			m.getPose().getOrientation().setX(Float.parseFloat(rotation[1]));
			m.getPose().getOrientation().setY(Float.parseFloat(rotation[2]));
			m.getPose().getOrientation().setZ(Float.parseFloat(rotation[3]));
			
			m.getScale().setX(Float.parseFloat(scale[0]));
			m.getScale().setY(Float.parseFloat(scale[1]));
			m.getScale().setZ(Float.parseFloat(scale[2]));
	
			// add marker to map
			synchronized (markers) {
				markers.put(markerId, m);
			}
			synchronized (markersCache) {
				markersCache.put(markerId, m);
			}
			
			publishMarkers();
		}
		catch(Exception exc) {
			log.warn("Failed to add text marker.", exc);
			return;
		}
	}

	// // // // // // // // // // // // // // // // // // // // // // // // // // //
	//
	// Text rendering
	//
	
	public void addText(String identifier, String text, String position[]) {
		try {
			Marker m = markersCache.get(identifier);
			if(m==null) {
				m = createMarker();
				
				m.setType(Marker.TEXT_VIEW_FACING);
				m.getColor().setR(0.6f);
				m.getColor().setG(0.9f);
				m.getColor().setB(0.6f);
				m.getColor().setA(1.0f);
			}
			
			m.setText(text);
			// The height of an uppercase 'A'
			m.getScale().setZ(1.0);
			
			m.getPose().getPosition().setX(Float.parseFloat(position[0]));
			m.getPose().getPosition().setY(Float.parseFloat(position[1]));
			m.getPose().getPosition().setZ(Float.parseFloat(position[2]));
			
			// add marker to map
			synchronized (markers) {
				markers.put(identifier, m);
			}
			synchronized (markersCache) {
				markersCache.put(identifier, m);
			}
			
			publishMarkers();
		}
		catch(Exception exc) {
			log.warn("Failed to add text marker.", exc);
			return;
		}
	}
	
	public void addSpeechBubble(String identifier, String text, String position[], int duration) {
		try {
			final data_vis_msgs.Speech msg = speech_pub.newMessage();
			
			msg.setId(identifier);
			msg.setDuration(duration);
			msg.setText(text);
			
			msg.getPosition().setX(Float.parseFloat(position[0]));
			msg.getPosition().setY(Float.parseFloat(position[1]));
			msg.getPosition().setZ(Float.parseFloat(position[2]));
			
			speech_pub.publish(msg);
		}
		catch(Exception exc) {
			log.warn("Failed to add speech bubble.", exc);
			return;
		}
	}
	
	public void addHUDText(String identifier, String text) {
		try {
			final std_msgs.String msg = text_pub.newMessage();
			msg.setData(text);
			text_pub.publish(msg);
		}
		catch(Exception exc) {
			log.warn("Failed to add HUD text.", exc);
			return;
		}
	}

	// // // // // // // // // // // // // // // // // // // // // // // // // // //
	//
	// Helper methods: read data from Prolog, create data structures
	//

	/**
	 * Create a MarkerArray from the internal 'markers' buffer and publish 
	 * it to the visualization_marker_array topic.
	 */
	public void publishMarkers() {
		try {
			// wait for node to be ready
			waitForNode();

			synchronized (markers) {
				MarkerArray arr = pub.newMessage();

				for(Marker mrk : markers.values()) {
					arr.getMarkers().add(mrk);
				}
				pub.publish(arr);
				markers.clear();
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
	}


	/**
	 * Helper method: create Marker instance and add it to the internal buffer
	 * 
	 * @param identifier
	 * @param timepoint
	 */
	boolean addMarker(String identifier, String timepoint) {
		
		// read marker from Prolog
		final Marker m = readMarkerFromProlog(identifier, timepoint);
		if(m==null) return false;
		
		// add marker to map
		synchronized (markers) {
			markers.put(identifier, m);
		}
		synchronized (markersCache) {
			markersCache.put(identifier, m);
		}
		return true;
	}
	
	boolean updateMarker(String identifier, String timepoint) {
		Marker m = markersCache.get(identifier);
		if(m!=null) {
			if(setMarkerPose(m, identifier, timepoint)) {
				m.setAction(Marker.ADD);
			}
			else {
				m.setAction(Marker.DELETE);
			}
			markers.put(identifier, m);
			return true;
		}
		else {
			return false;
		}
	}

	/**
	 * @return a marker that belongs to the 'knowrob_vis' namespace
	 */
	Marker createMarker() {
		waitForNode();

		Marker m = node.getTopicMessageFactory().newFromType(visualization_msgs.Marker._TYPE);
		m.getHeader().setFrameId(reference_frame);
		m.getHeader().setStamp(node.getCurrentTime());
		m.setNs("knowrob_vis");
		m.setId(id++);
		m.setAction(Marker.ADD);
		m.setLifetime(new Duration());
		m.getColor().setR(1.0f);
		m.getColor().setG(1.0f);
		m.getColor().setB(0.0f);
		m.getColor().setA(1.0f);
		return m;
	}

	/**
	 * @param identifier OWL identifier of an individual
	 * @return true if blacklisted
	 */
	boolean isBlackListed(String identifier) {
		HashMap<String, Vector<String>> blk = PrologInterface.executeQuery(
				"owl_individual_of('"+ identifier + "', 'http://knowrob.org/kb/srdl2-comp.owl#UrdfJoint') ;" +
						"owl_individual_of('"+ identifier + "', 'http://knowrob.org/kb/knowrob.owl#RoomInAConstruction')");
		return blk!=null;
	}

	boolean setMarkerPose(Marker m, String identifier, String timepoint) {
		boolean hasPose = false;

		try {
			// read object pose
			String query = "object_pose_at_time('"+ identifier + "', '"+ timepoint + "', [X,Y,Z], [QW,QX,QY,QZ])";
			//log.info(query);
			HashMap<String, Vector<String>> res = PrologInterface.executeQuery(query);

			if (res!=null && res.get("X") != null && res.get("X").size() > 0 && res.get("X").get(0)!=null) {
				m.getPose().getOrientation().setW(Double.valueOf(res.get("QW").get(0)));
				m.getPose().getOrientation().setX(Double.valueOf(res.get("QX").get(0)));
				m.getPose().getOrientation().setY(Double.valueOf(res.get("QY").get(0)));
				m.getPose().getOrientation().setZ(Double.valueOf(res.get("QZ").get(0)));
				m.getPose().getPosition().setX(Double.valueOf(res.get("X").get(0)));
				m.getPose().getPosition().setY(Double.valueOf(res.get("Y").get(0)));
				m.getPose().getPosition().setZ(Double.valueOf(res.get("Z").get(0)));
				hasPose = true;
			}
		}
		catch (Exception e) {
			log.warn("Unable to lookup pose for '" + identifier + "'.", e);
		}
		
		if(hasPose) {
			try {
				HashMap<String, Vector<String>> res = PrologInterface.executeQuery(
					"rdf_has('"+identifier+"', knowrob:'visuallyAbove', literal(type(_,Value)))");
				
				// HACK: Force object to be visually above given Z value
				if (res!=null && res.get("Value") != null) {
					double val = Double.valueOf(OWLThing.removeSingleQuotes(res.get("Value").get(0))).doubleValue();
					if(val > m.getPose().getPosition().getZ()) {
						m.getPose().getPosition().setZ(val);
					}
				}
			}
			catch (Exception e)
			{
				log.warn("Failed to read property.", e);
			}
		}
		
		return hasPose;
	}
	
	/**
	 * Read object information from Prolog and create a marker from it
	 *
	 * @param identifier OWL identifier of an object instance
	 * @param timepoint  OWL identifier of a timepoint instance
	 * @return Marker with the object information
	 */
	Marker readMarkerFromProlog(String identifier, String timepoint) {

		// wait for node to be ready
		waitForNode();
		
		if(isBlackListed(identifier)) return null;
		
		try {
			String query = "rdf_has('"+identifier+"', knowrob:'hasVisual', literal(type(_,Value)))";
			HashMap<String, Vector<String>> res = PrologInterface.executeQuery(query);
			
			if (res!=null && res.get("Value") != null) {
				if("false".equals(OWLThing.removeSingleQuotes(res.get("Value").get(0)))) {
					// Object has no visual
					return null;
				}
			}
		}
		catch (Exception e)
		{}
		
		final Marker m = createMarker();
		m.setType(Marker.CUBE);
		// set light grey color by default
		m.getColor().setR(0.6f);
		m.getColor().setG(0.6f);
		m.getColor().setB(0.6f);
		m.getColor().setA(1.0f);
		m.getScale().setX(1.0);
		m.getScale().setY(1.0);
		m.getScale().setZ(1.0);
		
		if(!setMarkerPose(m, identifier, timepoint)) {
			return null;
		}
		
		try {
			// check if mesh is available for this object
			String query = "get_model_path('"+ identifier + "', Path)";
			//log.info(query);
			HashMap<String, Vector<String>> res = PrologInterface.executeQuery(query);
			
			if (res!=null && res.get("Path") != null && res.get("Path").size() > 0 && res.get("Path").get(0)!=null) {
				m.setType(Marker.MESH_RESOURCE);
				m.setMeshResource(OWLThing.removeSingleQuotes(res.get("Path").get(0)));
				
				if(OWLThing.removeSingleQuotes(res.get("Path").get(0)).endsWith(".dae") ||
				   OWLThing.removeSingleQuotes(res.get("Path").get(0)).endsWith(".DAE")){
					m.setMeshUseEmbeddedMaterials(true);
					// Color must be set to zero for mesh textures
					m.getColor().setR(0.0f);
					m.getColor().setG(0.0f);
					m.getColor().setB(0.0f);
					m.getColor().setA(0.0f);
				}
			}
		}
		catch (Exception e) {
			log.warn("Unable to lookup mesh for '" + identifier + "'.", e);
		}

		try {
			// read object dimensions if available
			String query = "object_dimensions('"+identifier+"', D, W, H)";
			//log.info(query);
			HashMap<String, Vector<String>> res = PrologInterface.executeQuery(query);

			if (res!=null && res.get("D") != null && res.get("D").size() > 0 && res.get("D").get(0)!=null) {
				m.getScale().setX(Double.valueOf(OWLThing.removeSingleQuotes(res.get("D").get(0))));
				m.getScale().setY(Double.valueOf(OWLThing.removeSingleQuotes(res.get("W").get(0))));
				m.getScale().setZ(Double.valueOf(OWLThing.removeSingleQuotes(res.get("H").get(0))));
			} else if(m.getType() != Marker.MESH_RESOURCE) {
				m.getScale().setX(0.05);
				m.getScale().setY(0.05);
				m.getScale().setZ(0.05);
			}
		}
		catch (Exception e) {
			log.warn("Unable to lookup dimensions for '" + identifier + "'.", e);
		}
		
		try {
			// read object color if available
			// This removes colors from meshes!!
			String query = "object_color('"+identifier+"', COL)";
			//log.info(query);
			HashMap<String, Vector<String>> res = PrologInterface.executeQuery(query);
			
			if(res!=null && res.get("COL")!=null) {
				String c[] = res.get("COL").get(0).split(" ");
				if(c.length==4) {
					m.getColor().setR(Float.valueOf(OWLThing.removeSingleQuotes(c[0])));
					m.getColor().setG(Float.valueOf(OWLThing.removeSingleQuotes(c[1])));
					m.getColor().setB(Float.valueOf(OWLThing.removeSingleQuotes(c[2])));
					m.getColor().setA(Float.valueOf(OWLThing.removeSingleQuotes(c[3])));
				}
				else {
					log.warn("Invalid color for '" + identifier + "'.");
				}
			}
		}
		catch (Exception e) {
			log.warn("Unable to lookup color for '" + identifier + "'.", e);
		}

		return m;
	}

	/**
	 * Read link transform and create a marker from it
	 *
	 * @param link TF frame name
	 * @param timepoint OWL identifier of a timepoint instance
	 * @return Marker with the object information
	 */
	Marker readLinkMarkerFromProlog(String link, String timepoint, int marker_type) {

		// wait for node to be ready
		waitForNode();
		
		
		final Marker m = createMarker();

		// Set marker size
		m.getScale().setX(0.05);
		m.getScale().setY(0.05);
		m.getScale().setZ(0.05);

		// Set marker shape
		if(marker_type >= 0 && marker_type <= 8)//validate markers
			m.setType(marker_type);
		else
			m.setType(Marker.ARROW);

		// Lookup TF transform that corresponds to specified link
		try {
			StampedTransform tr = TFMemory.getInstance().lookupTransform(reference_frame, link, parseTime(timepoint));
			if(tr==null) {
				log.warn("TF data missing for '" + link + "' " + timepoint + " missing in mongo.");
				return null;
			}
			else {
				m.getPose().getPosition().setX(tr.getTranslation().x);
				m.getPose().getPosition().setY(tr.getTranslation().y);
				m.getPose().getPosition().setZ(tr.getTranslation().z);
				m.getPose().getOrientation().setW(tr.getRotation().w);
				m.getPose().getOrientation().setX(tr.getRotation().x);
				m.getPose().getOrientation().setY(tr.getRotation().y);
				m.getPose().getOrientation().setZ(tr.getRotation().z);
			}
		}
		catch (Exception e) {
			log.warn("Unable to lookup transform for '" + link + "'.", e);
		}

		return m;
	}

	/**
	 * Read children of an object instance that are either linked
	 * by the 'properPhysicalParts' property or (inversely) by the
	 * 'describedInMap' property.
	 *
	 * @param parent The parent whose children are to the returned, as OWL identifier
	 * @return Array of OWL identifiers of all children
	 */
	String[] readChildren(String parent) {

		HashSet<String> children = new HashSet<String>();

		// if parent is map, read map objects; otherwise, the rdf_reachable returns the parent itself

		HashMap<String, Vector<String>> mapParts = PrologInterface.executeQuery(
				"rdf_reachable(PART, knowrob:describedInMap, '"+parent+"')");

		if(mapParts != null && mapParts.get("PART") != null) {
			for(int i=0;i<mapParts.get("PART").size();i++) {
				final String p_i = mapParts.get("PART").get(i);
				if(p_i.equals(parent)) continue;

				// add object to children set
				children.add(OWLThing.removeSingleQuotes(p_i));

				// read all physical parts of all child objects
				HashMap<String, Vector<String>> parts = PrologInterface.executeQuery(
						"rdf_reachable("+p_i+", knowrob:properPhysicalParts, P);" +
								"rdf_reachable("+p_i+", 'http://knowrob.org/kb/srdl2-comp.owl#subComponent', P);" +
								"rdf_reachable("+p_i+", 'http://knowrob.org/kb/srdl2-comp.owl#successorInKinematicChain', P)");

				if(parts != null && parts.get("P") != null) {
					for(int j=0;j<parts.get("P").size();j++) {
						if(!parts.get("P").get(j).toString().equals(p_i)) {
							children.add(OWLThing.removeSingleQuotes(parts.get("P").get(j)));
						}
					}
				}
			}
		}

		return children.toArray(new String[]{});
	}

	/**
	 * Delete a single marker from the marker array
	 * and send the marker once with delete flag set.
	 * @param identifier Marker identifier
	 */
	Marker eraseMarker(String identifier) {
		Marker m = markersCache.remove(identifier);
		if(m!=null) {
			m.setAction(Marker.DELETE);
			trajectories.remove(identifier);
			highlighted.remove(identifier);
			markers.remove(identifier);
		}
		return m;
	}

	/**
	 * Parses String with common time format 'timepoint_%d'
	 * and returns a Time object.
	 */
	Time parseTime(String timepoint) {
		double posix_ts = parseTime_d(timepoint);
		Time time = new Time();
		time.secs = (int)posix_ts;
		time.nsecs = (int) (1E9 * (posix_ts - ((int) posix_ts)));
		return time;
	}
	/**
	 * Parses String with common time format 'timepoint_%d'
	 * and returns a double precision number that represents
	 * the time passed since 1970.
	 */
	double parseTime_d(String timepoint) {
		String x[] = timepoint.split("timepoint_");
		// Also allow input strings without 'timepoint_' prefix
		String ts = (x.length==1 ? x[0] : x[1]);
		return Double.valueOf(ts.replaceAll("[^0-9.]", ""));
	}
	

	public static class MarkerVisualizationMain extends MarkerVisualization {
		public MarkerVisualizationMain() {
			super();
			// only needed for testing from Java:
			PrologInterface.initJPLProlog("knowrob_vis");
			PrologInterface.executeQuery("register_ros_package(knowrob_mongo)");
			PrologInterface.executeQuery("register_ros_package(mod_srdl)");
		}
	}
	
	
	private void waitForNode() {
	
		// wait for node to be ready
		try {
			while(node == null || pub == null) {
				Thread.sleep(200);
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	public double getTrajectoryLength(String link, String start_timepoint, String end_timepoint, double interval) 
	{
		String tflink_ = (link.startsWith("/") ? link : "/"+link);

		double t0 = parseTime_d(start_timepoint);
		double t1 = parseTime_d(end_timepoint);

		double length = 0;

		Marker prev = null;
		for (double i = t0; i <= t1; i += interval) {
			String timepoint = "timepoint_" + new DecimalFormat("###.###").format(i);
			
			Marker m = readLinkMarkerFromProlog(tflink_, timepoint, 0);

			if(prev !=null)
			{
				length += java.lang.Math.pow(
					java.lang.Math.pow(m.getPose().getPosition().getX() - prev.getPose().getPosition().getX(),2)
					+ java.lang.Math.pow(m.getPose().getPosition().getY() - prev.getPose().getPosition().getY(),2)
					+ java.lang.Math.pow(m.getPose().getPosition().getZ() - prev.getPose().getPosition().getZ(),2), 0.5);
			}
			
			prev = m;
		}
		
		return length;
	}

	public static void main(String args[]) {
//		MarkerVisualization vis = new MarkerVissualizationMain();

//				MarkerVisualization vis = new MarkerVisualization();
//				vis.addObjectWithChildren("http://knowrob.org/kb/ias_semantic_map.owl#SemanticEnvironmentMap0", "http://knowrob.org/kb/knowrob.owl#timepoint_1377766542");
//				vis.highlight("http://knowrob.org/kb/knowrob.owl#Refrigerator67", true, 150, 0, 0, 180);

		//		PrologInterface.executeQuery("mng_robot_pose_at_time(pr2:'PR2Robot1', '/map', knowrob:timepoint_1392799360, Pose)");
		//		PrologInterface.executeQuery("add_object_with_children(pr2:'PR2Robot1', knowrob:timepoint_1392799360)");

		//		vis.addObjectWithChildren("pr2:'PR2Robot1'", "http://knowrob.org/kb/knowrob.owl#timepoint_1377766542");


		//		vis.addObjectWithChildren("pr2:'PR2Robot1'", "http://knowrob.org/kb/knowrob.owl#timepoint_1377766542");

	}

	public static String getReferenceFrame() {
		return reference_frame;
	}
}

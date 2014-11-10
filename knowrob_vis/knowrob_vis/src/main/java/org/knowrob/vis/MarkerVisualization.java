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

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;

import org.ros.message.Duration;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.apache.commons.logging.Log;
import org.eclipse.jetty.server.Handler;
import org.eclipse.jetty.server.Server;
import org.eclipse.jetty.server.handler.DefaultHandler;
import org.eclipse.jetty.server.handler.HandlerList;
import org.eclipse.jetty.server.handler.ResourceHandler;
import org.knowrob.owl.OWLThing;
import org.knowrob.prolog.PrologInterface;
import org.knowrob.tfmemory.TFMemory;
import org.knowrob.utils.ros.RosUtilities;

import tfjava.StampedTransform;
import visualization_msgs.Marker;
import visualization_msgs.MarkerArray;

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
	
	ConnectedNode node;
	
	Server server;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("knowrob_vis");
	}

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
	private Map<String, HumanSkeleton> humanSkeletons;
	
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
		humanSkeletons = new HashMap<String, HumanSkeleton>();

		startWebServer(1111);
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		node = connectedNode;
		pub = connectedNode.newPublisher("/visualization_marker_array", visualization_msgs.MarkerArray._TYPE);
		log = connectedNode.getLog();
	}

	/**
	 * Add object 'identifier' to the visualization.
	 *
	 * @param identifier OWL identifier of an object instance
	 * @param timepoint  OWL identifier of a timepoint instance
	 */
	public void addObject(String identifier, String timepoint) {
		addMarker(identifier, timepoint);
		publishMarkers();
	}



	/**
	 * Add object 'identifier' and all its parts to the visualization.
	 *
	 * @param identifier OWL identifier of an object instance
	 * @param timepoint  OWL identifier of a timepoint instance
	 */
	public void addObjectWithChildren(String identifier, String timepoint) {

		// add this object
		addMarker(identifier, timepoint);

		// read children and add them too
		for(String child : readChildren(identifier))
			addMarker(child, timepoint);

		publishMarkers();
	}


	/**
	 * Remove object 'identifier' from the visualization.
	 *
	 * @param identifier OWL identifier of an object instance
	 */
	public void removeObject(String identifier) {
		// remove the object from the list
		synchronized (markers) {
			eraseMarker(identifier);
		}
	}

	/**
	 * Remove the object 'identifier' as well as its children
	 *
	 * @param identifier OWL identifier of an object instance
	 */
	public void removeObjectWithChildren(String identifier) {
		// remove this object
		eraseMarker(identifier);

		// remove children and remove them too
		for(String child : readChildren(identifier))
			eraseMarker(child);

		publishMarkers();
	}

	/**
	 * Remove all objects from the visualization
	 */
	public void clear() {
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

	/**
	 * Highlight the object 'identifier' in red
	 *
	 * @param identifier OWL identifier of an object instance
	 * @param highlight True to set, False to remove highlight
	 */
	public void highlight(String identifier) {
		highlight(identifier, 200, 0, 0, 255);
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
		
		int r = (col & 0xff0000) >> 16;
		int g = (col & 0x00ff00) >> 8;
		int b = (col & 0x0000ff);
		
		highlight(identifier, r, g, b, 255);
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
			log.warn("Unable to find marker for identifier '" + identifier + "'.");
		}
	}
	
	boolean __highlight__(String identifier, int r, int g, int b, int a) {
		final Marker m = markersCache.get(identifier);
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
		return true;
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
		final List<String> traj = trajectories.get(tflink);
		if(traj == null) {
			log.warn("Unable to find trajectory for identifier '" + tflink + "'.");
			return;
		}
		
		final MarkerArray arr = pub.newMessage();
		
		for(final String identifier : traj) {
			final Marker m = markersCache.get(identifier);
			if(m==null) {
				log.warn("Unable to find marker for identifier '" + identifier + "'.");
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

	/**
	 * Shows the human pose at given time point as
	 * a stick-man in the visualization canvas.
	 * The pose is read from the 'roslog' mongo database.
	 * 
	 * @param humanIndividual The name of the human individual which defines the skeletal structure
	 * @param timepoint The timepoint for TF lookup
	 * @param humanId Allows to add multiple visualizations in the same canvas
	 * @param tfSuffix Suffix used for TF frames
	 */
	public void addHumanPose(String humanIndividual, String timepoint, int humanId, String tfSuffix) {
		long t0 = System.currentTimeMillis();
		
		// Lookup skeletal structure
		final HumanSkeleton skeleton;
		try {
			skeleton = getHumanSkeleton("'" + humanIndividual + "'");
		}
		catch(Exception exc) {
			log.warn("Failed to initialize human skeleton.", exc);
			return;
		}

		final Time time = parseTime(timepoint);
		int index = 0;

		try {
			// Add cylinder marker between links
			for(HumanSkeleton.Link sourceLink : skeleton.getLinks()) {
				final HumanSkeleton.StampedLink sl0 = new HumanSkeleton.StampedLink(sourceLink,time,humanId,tfSuffix);

				if(!addHumanMarker(skeleton.createSphereMarker(node,sl0),index,humanId)) {
					log.warn("Unable to create sphere marker for '" + sourceLink.sourceFrame + "'.");
				}
				index += 1;
				
				for(String conn : sourceLink.succeeding) {
					final HumanSkeleton.Link targetLink = skeleton.getLink(conn);
					if(targetLink==null) {
						log.warn("Link not known '" + conn + "'.");
						continue;
					}
					final HumanSkeleton.StampedLink sl1 = new HumanSkeleton.StampedLink(targetLink,time,humanId,tfSuffix);
					
					if(!addHumanMarker(skeleton.createCylinderMarker(node,sl0,sl1),index,humanId)) {
						log.warn("Unable to create cylinder marker between '" +
								sourceLink.sourceFrame + "' and '" + conn + "'.");
					}
					index += 1;
				}
			}
		}
		catch(Exception exc) {
			log.warn("Failed to add markers for human skeleton.", exc);
			return;
		}
		
		publishMarkers();
		
		log.info("addHumanPose took " + (System.currentTimeMillis()-t0) + " ms.");
	}
	
	HumanSkeleton getHumanSkeleton(String humanIndividualName) {
		HumanSkeleton out = humanSkeletons.get(humanIndividualName);
		if(out==null) {
			out = new HumanSkeleton(humanIndividualName);
			humanSkeletons.put(humanIndividualName, out);
		}
		return out;
	}

	/**
	 * Removes previously added stick-man markers.
	 * @param id the id of the stick-man
	 */
	public void removeHumanPose(int id) {
		final String ns = HumanSkeleton.getHumanMarkerNs(id);
		final List<String> toRemove = new LinkedList<String>();
		final MarkerArray arr = pub.newMessage();
		
		synchronized (markersCache) {
			for(Entry<String, Marker> e : markersCache.entrySet()) {
				if(e.getValue().getNs().equals(ns)) {
					toRemove.add(e.getKey());
					e.getValue().setAction(Marker.DELETE);
					arr.getMarkers().add(e.getValue());
				}
			}
			for(String x : toRemove) markersCache.remove(x);
		}
		
		pub.publish(arr);
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
		synchronized (markers) {
			MarkerArray arr = pub.newMessage();

			for(Marker mrk : markers.values()) {
				arr.getMarkers().add(mrk);
			}
			pub.publish(arr);
			markers.clear();
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
	
	/**
	 * @return a marker that belongs to the 'knowrob_vis' namespace
	 */
	Marker createMarker() {
		Marker m = node.getTopicMessageFactory().newFromType(visualization_msgs.Marker._TYPE);
		m.getHeader().setFrameId("/map");
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

	/**
	 * Read object information from Prolog and create a marker from it
	 *
	 * @param identifier OWL identifier of an object instance
	 * @param timepoint  OWL identifier of a timepoint instance
	 * @return Marker with the object information
	 */
	Marker readMarkerFromProlog(String identifier, String timepoint) {
		if(isBlackListed(identifier)) return null;
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

		try {
			// read object pose
			HashMap<String, Vector<String>> res = PrologInterface.executeQuery(
					"object_pose_at_time('"+ identifier + "', '"+ timepoint +
					"', [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33])");

			if (res!=null && res.get("M00") != null && res.get("M00").size() > 0 && res.get("M00").get(0)!=null) {
				double[] p = new double[16];
				Matrix4d poseMat = new Matrix4d(p);

				for(int i=0;i<4;i++) {
					for(int j=0;j<4;j++) {
						poseMat.setElement(i, j, Double.valueOf(res.get("M"+i+j).get(0)));
					}
				}

				Quat4d q = new Quat4d();
				q.set(poseMat);

				m.getPose().getOrientation().setW(q.w);
				m.getPose().getOrientation().setX(q.x);
				m.getPose().getOrientation().setY(q.y);
				m.getPose().getOrientation().setZ(q.z);

				m.getPose().getPosition().setX(poseMat.m03);
				m.getPose().getPosition().setY(poseMat.m13);
				m.getPose().getPosition().setZ(poseMat.m23);

				// debug
				//System.err.println("adding " + identifier + " at pose [" + m.pose.position.x + ", " + m.pose.position.y + ", " + m.pose.position.z + "]");
			}
		}
		catch (Exception e) {
			log.warn("Unable to lookup pose for '" + identifier + "'.", e);
		}

		try {
			// read object dimensions if available
			HashMap<String, Vector<String>> res = PrologInterface.executeQuery(
					"rdf_has('"+identifier+"', knowrob:depthOfObject,  literal(type(_, D))), " +
					"rdf_has('"+identifier+"', knowrob:widthOfObject,  literal(type(_, W))), " +
					"rdf_has('"+identifier+"', knowrob:heightOfObject, literal(type(_, H)))");

			if (res!=null && res.get("D") != null && res.get("D").size() > 0 && res.get("D").get(0)!=null) {
				m.getScale().setX(Double.valueOf(OWLThing.removeSingleQuotes(res.get("D").get(0))));
				m.getScale().setY(Double.valueOf(OWLThing.removeSingleQuotes(res.get("W").get(0))));
				m.getScale().setZ(Double.valueOf(OWLThing.removeSingleQuotes(res.get("H").get(0))));

			} else {
				m.getScale().setX(0.05);
				m.getScale().setY(0.05);
				m.getScale().setZ(0.05);
			}
		}
		catch (Exception e) {
			log.warn("Unable to lookup dimensions for '" + identifier + "'.", e);
		}

		try {
			// check if mesh is available for this object
			HashMap<String, Vector<String>> res = PrologInterface.executeQuery("get_model_path('"+ identifier + "', Path)");

			if (res!=null && res.get("Path") != null && res.get("Path").size() > 0 && res.get("Path").get(0)!=null) {
				m.setType(Marker.MESH_RESOURCE);
				m.setMeshResource(OWLThing.removeSingleQuotes(res.get("Path").get(0)));
				m.getScale().setX(1.0);
				m.getScale().setY(1.0);
				m.getScale().setZ(1.0);
				//if the file is a collada file use the embedded textures
				//are there any other common file endings/supported formats
				if(OWLThing.removeSingleQuotes(res.get("Path").get(0)).endsWith(".dae") || 
						OWLThing.removeSingleQuotes(res.get("Path").get(0)).endsWith(".DAE")){
				  m.setMeshUseEmbeddedMaterials(true);
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
			// read object color if available
			// This removes colors from meshes!!
			HashMap<String, Vector<String>> res = PrologInterface.executeQuery(
					"rdf_has('"+identifier+"', knowrob:mainColorOfObject, literal(type(_, COL)))");
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
			StampedTransform tr = TFMemory.getInstance().lookupTransform("/map", link, parseTime(timepoint));
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
	void eraseMarker(String identifier) {
		Marker m = markers.remove(identifier);
		if(m!=null) {
			final MarkerArray arr = pub.newMessage();
			m.setAction(Marker.DELETE);
			arr.getMarkers().add(m);
			pub.publish(arr);
			
			trajectories.remove(identifier);
			highlighted.remove(identifier);
			markersCache.remove(identifier);
		}
	}
	
	boolean addHumanMarker(Marker marker, int index, int id) {
		if(marker!=null) {
			final StringBuilder identifier = new StringBuilder();
			identifier.append("human_").append(id).append('_').append(index);
			marker.setId(index);
			
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

	/////////////////
	////// Human Pose Stop
	/////////////////

	public void startWebServer(int port) {
		
        server = new Server(port);
        
        ResourceHandler resource_handler = new ResourceHandler();
        
        resource_handler.setDirectoriesListed(true);
        resource_handler.setWelcomeFiles(new String[]{ "index.html", "robohow.html" });
        resource_handler.setResourceBase(RosUtilities.rospackFind("knowrob_vis") + "/html");
 
        DefaultHandler def = new DefaultHandler();
        def.setServeIcon(false);
        
        HandlerList handlers = new HandlerList();
        handlers.setHandlers(new Handler[] { resource_handler,  def});
        server.setHandler(handlers);
 
        try {
			server.start();
//	        server.join();
		}
        catch (Exception e) {
			log.warn("Unable to start knowrob_vis server.", e);
		}
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
	
	public static void main(String args[]) {
		//MarkerVisualization vis = new MarkerVisualizationMain();
		
		//		MarkerVisualization vis = new MarkerVisualization();
		//		vis.addObjectWithChildren("http://knowrob.org/kb/ias_semantic_map.owl#SemanticEnvironmentMap0", "http://knowrob.org/kb/knowrob.owl#timepoint_1377766542");
		//		vis.highlight("http://knowrob.org/kb/knowrob.owl#Refrigerator67", true, 150, 0, 0, 180);

		//		PrologInterface.executeQuery("mng_robot_pose_at_time(pr2:'PR2Robot1', '/map', knowrob:timepoint_1392799360, Pose)");
		//		PrologInterface.executeQuery("add_object_with_children(pr2:'PR2Robot1', knowrob:timepoint_1392799360)");

		//		vis.addObjectWithChildren("pr2:'PR2Robot1'", "http://knowrob.org/kb/knowrob.owl#timepoint_1377766542");


		//		vis.addObjectWithChildren("pr2:'PR2Robot1'", "http://knowrob.org/kb/knowrob.owl#timepoint_1377766542");

	}
}

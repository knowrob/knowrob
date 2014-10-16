package org.knowrob.vis;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Vector;
import java.util.concurrent.ConcurrentHashMap;
import java.util.List;
import java.util.ArrayList;
import java.awt.Color;
import java.text.DecimalFormat;

import javax.swing.text.html.StyleSheet;
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

import std_msgs.ColorRGBA;
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
 * @author tenorth@cs.uni-bremen.de
 * @author danielb@cs.uni-bremen.de
 *
 */
public class MarkerVisualization extends AbstractNodeMain {

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
	 * Store the highlights and the original colors
	 */
	protected Map<String, ColorRGBA> highlighted;


	/**
	 * Counter for marker IDs
	 */
	private static int id = 0;
	
	/**
	 *
	 */
	protected Map<String, List<String>> trajectories;
	//private static List<String> trajectoryIds = new ArrayList<String>();
	
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

		// only needed for testing from Java:
		//PrologInterface.initJPLProlog("knowrob_vis");
		//PrologInterface.executeQuery("register_ros_package(knowrob_mongo)");
		//PrologInterface.executeQuery("register_ros_package(mod_srdl)");

		markers =  new ConcurrentHashMap<String, Marker>(8, 0.9f, 1);
		markersCache =  new ConcurrentHashMap<String, Marker>(8, 0.9f, 1);
		highlighted = new ConcurrentHashMap<String, ColorRGBA>(8, 0.9f, 1);
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
		synchronized (markers) {
			for(Marker m : markers.values()) {
				m.setAction(Marker.DELETE);
				arr.getMarkers().add(m);
			}
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
	public void highlight(String identifier, boolean highlight) {
		highlight(identifier, highlight, 200, 0, 0, 255);
	}



	/**
	 * Highlight the object 'identifier' in 'color'
	 *
	 * @param identifier OWL identifier of an object instance
	 * @param highlight True to set, False to remove highlight
	 * @param color String of the form #RRGGBB
	 */
	public void highlight(String identifier, boolean highlight, String color) {

		int col = Integer.valueOf(color, 16);
		
		int r = (col & 0xff0000) >> 16;
		int g = (col & 0x00ff00) >> 8;
		int b = (col & 0x0000ff);
		
		highlight(identifier, highlight, r, g, b, 255);
	}


	/**
	 * Highlight the object 'identifier' with color RGBA
	 *
	 * @param identifier OWL identifier of an object instance
	 * @param highlight True to set, False to remove highlight
	 * @param r Red value (0--255)
	 * @param g Green value (0--255)
	 * @param b Blue value (0--255)
	 * @param a Alpha value (0--255)
	 */
	public void highlight(String identifier, boolean highlight, int r, int g, int b, int a) {

		if(!highlight) {
			synchronized (markersCache) { synchronized (markers) {
				markersCache.get(identifier).setColor(highlighted.get(identifier));
				markers.put(identifier, markersCache.get(identifier));
			}}

		} else {

			synchronized (highlighted) {
				synchronized (markersCache) { synchronized (markers) {
					if(markersCache.get(identifier)!=null) {
						highlighted.put(identifier, markersCache.get(identifier).getColor());
						markers.put(identifier, markersCache.get(identifier));
					}
				}}
			}

			synchronized (markersCache) {
				if(markersCache.get(identifier)!=null) {
					markersCache.get(identifier).getColor().setR(((float) r)/255);
					markersCache.get(identifier).getColor().setG(((float) g)/255);
					markersCache.get(identifier).getColor().setB(((float) b)/255);
					markersCache.get(identifier).getColor().setA(((float) a)/255);
					markers.put(identifier, markersCache.get(identifier));
				}
			}
		}
		publishMarkers();
	}



	/**
	 * Highlight the object 'identifier' and its children in red
	 *
	 * @param identifier OWL identifier of an object instance
	 * @param highlight True to set, False to remove highlight
	 */
	public void highlightWithChildren(String identifier, boolean highlight) {
		highlightWithChildren(identifier, highlight, "ff0000");
	}
	
	/**
	 * Highlight the object 'identifier' and its children in the given color
	 *
	 * @param identifier OWL identifier of an object instance
	 * @param highlight True to set, False to remove highlight
	 * @param color Integer of the form #AARRGGBB
	 */
	public void highlightWithChildren(String identifier, boolean highlight, String color) {

		// remove this object
		highlight(identifier, highlight, color);

		// remove children and highlight them too
		for(String child : readChildren(identifier)) {
			highlight(child, highlight, color);
		}
		publishMarkers();
	}


	/**
	 * Clear all highlights
	 */
	public void clearHighlight() {
		// reset colors to cached original ones

		synchronized (highlighted) {
			synchronized (markersCache) { synchronized (markers) {
				for(String obj : highlighted.keySet()) {
					markersCache.get(obj).setColor(highlighted.get(obj));
					markers.put(obj, markersCache.get(obj));
				}
			}}
		}
		publishMarkers();
	}
	
	
	/**
	 * Show hands and base trajectory in visualization.
	 *
	 * @param starttime OWL identifier of a timepoint instance
	 * @param endtime OWL identifier of a timepoint instance
	 * @param interval in seconds
	 */
	public void showTrajectory(String tflink, String starttime, String endtime, double interval) {

		String identifier;
		String timepoint;
		String tflink_ = (tflink.startsWith("/") ? tflink : "/"+tflink);

		try {
			trajectories.put(tflink_, new ArrayList<String>());
	
			for (double i = Double.parseDouble(starttime.substring(starttime.indexOf("timepoint_") + 10)); i <= Double.parseDouble(endtime.substring(endtime.indexOf("timepoint_") + 10)); i += interval) {
	
				timepoint = "'" + starttime.substring(0, starttime.indexOf("timepoint_")) + starttime.substring(starttime.indexOf("timepoint_"), starttime.indexOf("timepoint_") + 10) + new DecimalFormat("###.###").format(i) + "'";//String.valueOf(i);
				identifier = tflink_ + new DecimalFormat("###.###").format(i);//String.valueOf(i);
	
				// read marker from Prolog
				Marker m = readLinkMarkerFromProlog(tflink_, timepoint);
	
				// add marker to map
				if(m!=null) {
					trajectories.get(tflink_).add(identifier);
					synchronized (markers) { synchronized (markersCache) { 
						markers.put(identifier, m);
						markersCache.put(identifier, m);
					}}
				}
	
			}
			publishMarkers();
		} catch(Exception e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * Show hands and base trajectory in visualization.
	 *
	 * @param starttime OWL identifier of a timepoint instance
	 * @param endtime OWL identifier of a timepoint instance
	 * @param interval in seconds
	 */
	public void showSimTrajectory(String tflink, String starttime, String endtime, double interval, int markertype, String markercol) {

		String identifier;
		String timepoint;
		
		try
		{
			trajectories.put(tflink, new ArrayList<String>());

			System.out.println("starttime: " + starttime);
			System.out.println("endtime: " + endtime);

			for (double i = Double.parseDouble(starttime.substring(starttime.indexOf("timepoint_") + 10)); i <= Double.parseDouble(endtime.substring(endtime.indexOf("timepoint_") + 10)); i += interval) {

				timepoint = "'" + starttime.substring(0, starttime.indexOf("timepoint_")) + starttime.substring(starttime.indexOf("timepoint_"), starttime.indexOf("timepoint_") + 10) + new DecimalFormat("###.###").format(i) + "'";//String.valueOf(i);
				System.out.println("timepoint: " + timepoint);
				//			System.out.println("i: " + i);
				//			System.out.println("end: " + Double.parseDouble(endtime.substring(endtime.indexOf("timepoint_") + 10)));
				identifier = tflink + new DecimalFormat("###.###").format(i);//String.valueOf(i);

				// read marker from Prolog
				Marker m = readLinkMarkerFromPrologSim(tflink, timepoint, markertype, markercol);

				// add marker to map
				if(m!=null) {
					trajectories.get(tflink).add(identifier);
					synchronized (markers) {
						markers.put(identifier, m);
					}
				}

			}
			publishMarkers();
		} catch(Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Remove trajectory markers
	 */
	public void removeTrajectory(String tflink) {
		final MarkerArray arr = pub.newMessage();
		if (trajectories.get(tflink) != null){
//			System.out.println("tflink trajectories not null: " + trajectories.get(tflink).size());
			for (int i = 0; i < trajectories.get(tflink).size(); i++) {
				// remove the object from the list
				synchronized (markers) {
					Marker m = markers.remove(trajectories.get(tflink).get(i));
					m.setAction(Marker.DELETE);
					arr.getMarkers().add(m);
				}
			}
			System.out.println("removing tflink trajectories");
			trajectories.remove(tflink);
		}
		pub.publish(arr);
	}

	/**
	 * Shows the human pose at given time point as
	 * a stick-man in the visualization canvas.
	 * The pose is read from the 'roslog' mongo database.
	 * Note: Make sure to set search index for 'transforms.header.stamp'
	 * in mongo: db.tf.ensureIndex( { "transforms.header.stamp" : 1 } )
	 * 
	 * @param humanIndividual The name of the human individual which defines the skeletal structure
	 * @param timepoint The timepoint for TF lookup
	 * @param humanId Allows to add multiple visualizations in the same canvas
	 * @param tfPrefix Prefix used for TF frames
	 */
	public void addHumanPose(String humanIndividual, String timepoint, int humanId, String tfPrefix) {
		final HumanSkeleton skeleton;
		try {
			// ID allows multiple stick-mans for the same individual.
			skeleton = getHumanSkeleton("'" + humanIndividual + "'");
		}
		catch(Exception exc) {
			log.error("Failed to initialize human skeleton.", exc);
			return;
		}

		final Time time = parseTime(timepoint);
		int index = 0;

		// Add cylinder marker between joints
		for(HumanSkeleton.Link sourceLink : skeleton.getLinks()) {
			for(String conn : sourceLink.succeeding) {
				final HumanSkeleton.Link targetLink = skeleton.getLink(conn);
				if(targetLink==null) {
					log.warn("Link not known '" + conn + "'.");
				}
				if(!addHumanMarker(skeleton.createCylinderMarker(
						sourceLink,targetLink,time,humanId,tfPrefix),index,humanId)) {
					log.warn("No human pose found for timepoint '" + time + "'.");
					return;
				}
				index += 1;
			}
		}
		// Add sphere marker for joints
		for(HumanSkeleton.Link link : skeleton.getLinks()) {
			if(!addHumanMarker(skeleton.createSphereMarker(
					link,time,humanId,tfPrefix),index,humanId)) {
				log.warn("No human pose found for timepoint '" + time + "'.");
				return;
			}
			index += 1;
		}
		
		publishMarkers();
	}
	
	private HumanSkeleton getHumanSkeleton(String humanIndividualName) {
		HumanSkeleton out = humanSkeletons.get(humanIndividualName);
		if(out==null) {
			out = new HumanSkeleton(humanIndividualName, node);
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
		
		synchronized (markers) {
			for(Entry<String, Marker> e : markers.entrySet()) {
				if(e.getValue().getNs().equals(ns)) {
					toRemove.add(e.getKey());
					e.getValue().setAction(Marker.DELETE);
					arr.getMarkers().add(e.getValue());
				}
			}
			for(String x : toRemove) markers.remove(x);
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
	protected void addMarker(String identifier, String timepoint) {

		// read marker from Prolog
		Marker m = readMarkerFromProlog(identifier, timepoint);

		// add marker to map
		if(m!=null) {
			synchronized (markers) {
				markers.put(identifier, m);
			}
			synchronized (markersCache) {
				markersCache.put(identifier, m);
			}
		}
	}

	/**
	 * Read object information from Prolog and create a marker from it
	 *
	 * @param identifier OWL identifier of an object instance
	 * @param timepoint  OWL identifier of a timepoint instance
	 * @return Marker with the object information
	 */
	Marker readMarkerFromProlog(String identifier, String timepoint) {


		// check if object is blacklisted
		HashMap<String, Vector<String>> blk = PrologInterface.executeQuery(
				"owl_individual_of('"+ identifier + "', 'http://knowrob.org/kb/srdl2-comp.owl#UrdfJoint') ;" +
						"owl_individual_of('"+ identifier + "', 'http://knowrob.org/kb/knowrob.owl#RoomInAConstruction')");

		if(blk!=null)
			return null;

		Marker m = node.getTopicMessageFactory().newFromType(visualization_msgs.Marker._TYPE);

		m.getHeader().setFrameId("/map");
		m.getHeader().setStamp(node.getCurrentTime());
		m.setNs("knowrob_vis");
		m.setId(id++);

		m.setAction(Marker.ADD);
		m.setLifetime(new Duration());

		try {
			// read object pose
			HashMap<String, Vector<String>> res = PrologInterface.executeQuery(
					"object_pose_at_time('"+ identifier + "', '"+ timepoint + "', [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33])");

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

			} else {
				m.setType(Marker.CUBE);
			}

		} catch (Exception e) {
			e.printStackTrace();
		}


		try {

			// read object dimensions if available, default to a 10cm cube
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

		} catch (Exception e) {
			e.printStackTrace();
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

			} else {
				m.setType(Marker.CUBE);
			}

			// set light grey color by default
			m.getColor().setR(0.6f);
			m.getColor().setG(0.6f);
			m.getColor().setB(0.6f);
			m.getColor().setA(1.0f);

		} catch (Exception e) {
			e.printStackTrace();
		}

		return m;
	}

	/**
	 * Read link transform and create a marker from it
	 *
	 * @param link TODO explanation
	 * @param timepoint  OWL identifier of a timepoint instance
	 * @return Marker with the object information
	 */
	Marker readLinkMarkerFromPrologSim(String link, String timepoint, int marker_type, String marker_color) {

		Marker m = node.getTopicMessageFactory().newFromType(visualization_msgs.Marker._TYPE);

		m.getHeader().setFrameId("/map");
		m.getHeader().setStamp(node.getCurrentTime());
		m.setNs("knowrob_vis");
		m.setId(id++);

		m.setAction(Marker.ADD);
		m.setLifetime(new Duration());

		m.getScale().setX(0.05);
		m.getScale().setY(0.05);
		m.getScale().setZ(0.05);
		
		if(marker_type >= 0 && marker_type <= 8)//valid markers
			m.setType(marker_type);
		else
			m.setType(Marker.CYLINDER);
		
		//set marker color using stylesheet to convert to rgb
		StyleSheet s = new StyleSheet();
		Color color = s.stringToColor(marker_color);
		if(color == null) //given color is not a HTML3.2 color string
		{
			m.getColor().setR(1.0f);
			m.getColor().setG(1.0f);
			m.getColor().setB(0.0f);
			m.getColor().setA(1.0f);
		}
		else
		{
			m.getColor().setR(color.getRed()/(float)255.0);
			m.getColor().setG(color.getGreen()/(float)255.0);
			m.getColor().setB(color.getBlue()/(float)255.0);
			m.getColor().setA(1.0f);
		}		

		try {

			TFMemory tf = TFMemory.getInstance();

			String ts = timepoint.split("timepoint_")[1];
			double posix_ts = Double.valueOf(ts.substring(0, ts.length()-1));

			Time time = new Time();
			time.secs = (int)posix_ts;
			time.nsecs = (int) (1E9 * (posix_ts - ((int) posix_ts)));

			StampedTransform tr = tf.lookupSimTransform("/map", link, time);
			m.getPose().getPosition().setX(tr.getTranslation().x);
			m.getPose().getPosition().setY(tr.getTranslation().y);
			m.getPose().getPosition().setZ(tr.getTranslation().z);
//			System.out.println("x:" + m.getPose().getPosition().getX()+"; y:" + m.getPose().getPosition().getY()+"; z:" + m.getPose().getPosition().getZ());

			m.getPose().getOrientation().setW(tr.getRotation().w);
			m.getPose().getOrientation().setX(tr.getRotation().x);
			m.getPose().getOrientation().setY(tr.getRotation().y);
			m.getPose().getOrientation().setZ(tr.getRotation().z);
//			System.out.println("w:" + m.getPose().getOrientation().getW() + "; x:" + m.getPose().getOrientation().getX()+"; y:" + m.getPose().getOrientation().getY()+"; z:" + m.getPose().getOrientation().getZ());
		} catch (Exception e) {
			e.printStackTrace();
		}

		return m;
	}
	
	/**
	 * Read link transform and create a marker from it
	 *
	 * @param link TODO explanation
	 * @param timepoint  OWL identifier of a timepoint instance
	 * @return Marker with the object information
	 */
	Marker readLinkMarkerFromProlog(String link, String timepoint) {

		Marker m = node.getTopicMessageFactory().newFromType(visualization_msgs.Marker._TYPE);

		m.getHeader().setFrameId("/map");
		m.getHeader().setStamp(node.getCurrentTime());
		m.setNs("knowrob_vis");
		m.setId(id++);

		m.setAction(Marker.ADD);
		m.setLifetime(new Duration());

		m.getScale().setX(0.05);
		m.getScale().setY(0.05);
		m.getScale().setZ(0.05);

		m.setType(Marker.ARROW);

		m.getColor().setR(1.0f);
		m.getColor().setG(1.0f);
		m.getColor().setB(0.0f);
		m.getColor().setA(1.0f);

		try {

			TFMemory tf = TFMemory.getInstance();

			String ts = timepoint.split("timepoint_")[1];
			double posix_ts = Double.valueOf(ts.substring(0, ts.length()-1));

			Time time = new Time();
			time.secs = (int)posix_ts;
			time.nsecs = (int) (1E9 * (posix_ts - ((int) posix_ts)));

			StampedTransform tr = tf.lookupTransform("/map", link, time);
			m.getPose().getPosition().setX(tr.getTranslation().x);
			m.getPose().getPosition().setY(tr.getTranslation().y);
			m.getPose().getPosition().setZ(tr.getTranslation().z);

			m.getPose().getOrientation().setW(tr.getRotation().w);
			m.getPose().getOrientation().setX(tr.getRotation().x);
			m.getPose().getOrientation().setY(tr.getRotation().y);
			m.getPose().getOrientation().setZ(tr.getRotation().z);

		} catch (Exception e) {
			e.printStackTrace();
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
	private String[] readChildren(String parent) {

		HashSet<String> children = new HashSet<String>();

		// if parent is map, read map objects; otherwise, the rdf_reachable returns the parent itself

		HashMap<String, Vector<String>> mapParts = PrologInterface.executeQuery(
				"rdf_reachable(PART, knowrob:describedInMap, '"+parent+"')");

		if(mapParts != null && mapParts.get("PART") != null) {
			for(int i=0;i<mapParts.get("PART").size();i++) {
				if(!mapParts.get("PART").get(i).equals(parent)) {

					// add object to children set
					children.add(OWLThing.removeSingleQuotes(mapParts.get("PART").get(i)));


					// read all physical parts of all child objects
					HashMap<String, Vector<String>> parts = PrologInterface.executeQuery(
							"rdf_reachable("+mapParts.get("PART").get(i)+", knowrob:properPhysicalParts, P);" +
									"rdf_reachable("+mapParts.get("PART").get(i)+", 'http://knowrob.org/kb/srdl2-comp.owl#subComponent', P);" +
									"rdf_reachable("+mapParts.get("PART").get(i)+", 'http://knowrob.org/kb/srdl2-comp.owl#successorInKinematicChain', P)");

					if(parts != null && parts.get("P") != null) {

						for(int j=0;j<parts.get("P").size();j++) {
							if(!parts.get("P").get(j).toString().equals(mapParts.get("PART").get(i))) {
								children.add(OWLThing.removeSingleQuotes(parts.get("P").get(j)));
							}
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
	private void eraseMarker(String identifier) {
		Marker m = markers.remove(identifier);
		if(m!=null) {
			final MarkerArray arr = pub.newMessage();
			m.setAction(Marker.DELETE);
			arr.getMarkers().add(m);
			pub.publish(arr);
			
			trajectories.remove(identifier);
			highlighted.remove(identifier);
		}
	}
	
	private boolean addHumanMarker(Marker marker, int index, int id) {
		if(marker!=null) {
			final StringBuilder identifier = new StringBuilder();
			identifier.append("human_").append(id).append('_').append(index);
			marker.setId(index);
			
			synchronized (markers) { synchronized (markersCache) {
				markers.put(identifier.toString(), marker);
				markersCache.put(identifier.toString(), marker);
			}}
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
	private Time parseTime(String timepoint) {
		String x[] = timepoint.split("timepoint_");
		// Also allow input strings without 'timepoint_' prefix
		String ts = (x.length==1 ? x[0] : x[1]);
		
		double posix_ts = Double.valueOf(ts.replaceAll("[^0-9.]", ""));
		Time time = new Time();
		time.secs = (int)posix_ts;
		time.nsecs = (int) (1E9 * (posix_ts - ((int) posix_ts)));

		return time;
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
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	public static void main(String args[]) {
		
		
		//		MarkerVisualization vis = new MarkerVisualization();
		//		vis.addObjectWithChildren("http://knowrob.org/kb/ias_semantic_map.owl#SemanticEnvironmentMap0", "http://knowrob.org/kb/knowrob.owl#timepoint_1377766542");
		//		vis.highlight("http://knowrob.org/kb/knowrob.owl#Refrigerator67", true, 150, 0, 0, 180);

		//		PrologInterface.executeQuery("mng_robot_pose_at_time(pr2:'PR2Robot1', '/map', knowrob:timepoint_1392799360, Pose)");
		//		PrologInterface.executeQuery("add_object_with_children(pr2:'PR2Robot1', knowrob:timepoint_1392799360)");

		//		vis.addObjectWithChildren("pr2:'PR2Robot1'", "http://knowrob.org/kb/knowrob.owl#timepoint_1377766542");


		//		vis.addObjectWithChildren("pr2:'PR2Robot1'", "http://knowrob.org/kb/knowrob.owl#timepoint_1377766542");

	}
}

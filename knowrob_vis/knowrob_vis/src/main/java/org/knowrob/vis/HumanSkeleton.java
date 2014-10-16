package org.knowrob.vis;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Vector;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector4d;

import org.apache.commons.logging.Log;
import org.knowrob.prolog.PrologInterface;
import org.knowrob.tfmemory.TFMemory;
import org.ros.message.Time;
import org.ros.node.ConnectedNode;

import tfjava.StampedTransform;
import visualization_msgs.Marker;

/**
 * Represents the skeleton of a human.
 * The skeleton is given by a collection of links and connections
 * between them.
 * Each link has an associated TF frame that is used to lookup
 * the transformation of that link.
 * The skeletal structure is initialized based on a SRDL description
 * of the skeleton (an OWL individual is the entry point for
 * querying the skeletal structure).
 * 
 * @author danielb@cs.uni-bremen.de
 */
public class HumanSkeleton {
	/**
	 * A link in the skeleton of a human.
	 * Each link has an associated TF frame and a set of succeeding TF frames.
	 * Furthermore each link defines a radius that is used for visualization.
	 */
	public static class Link {
		/**
		 * The name of the TF frame that is associated to this link.
		 */
		final String sourceFrame;
		/**
		 * List of TF names of succeeding links.
		 */
		final String succeeding[];
		/**
		 * Size hint for rendering.
		 */
		final double size;
		
		private StampedTransform lastTransform = null;
		private Time lastTime = new Time();

		public Link(String sourceFrame, String[] succeeding, double size) {
			super();
			this.sourceFrame = sourceFrame;
			this.succeeding = succeeding;
			this.size = size;
		}
	}
	
	// TODO: Make this configurable ?
	private static final String targetFrame = "/map";

	/**
	 * The name of the human individual as used in OWL.
	 */
	private final String humanIndividual;
	
	/**
	 * Maps TF name to human skeleton link.
	 */
	private final HashMap<String, Link> links;
	
	/**
	 * The connected ROS node
	 */
	private final ConnectedNode node;

	/**
	 * Logging backend
	 */
	private final Log log;
	
	/**
	 * @param humanIndividual The name of the human individual as used in OWL
	 * @param node The connected ROS node
	 */
	public HumanSkeleton(String humanIndividual, ConnectedNode node) {
		this.humanIndividual = humanIndividual;
		this.node = node;
		this.log = node.getLog();
		this.links = new HashMap<String, Link>();
		
		log.debug("Initializing skeleton for human individual " + humanIndividual + ".");
		for(String linkName : queryLinks()) {
			final String tfFrame = getHumanTfFrame(linkName);
			links.put(tfFrame, new Link(tfFrame,
					querySucceedingLinks(linkName),
					getHumanLinkSize(linkName)));
		}
	}

	/**
	 * @return The set of links of this skeleton.
	 */
	public Collection<Link> getLinks() {
		return links.values();
	}
	
	/**
	 * @param linkName Name of a TF frame
	 * @return The link that is associated to the TF frame or null
	 */
	public Link getLink(String linkName) {
		return links.get(linkName);
	}
	
	/**
	 * Creates a sphere marker based on TF transformation of given skeleton link.
	 * @param link link of the human skeleton
	 * @param timepoint the timepoint used for looking up the transform
	 * @param id human id (to allow multiple humans in parallel)
	 * @param prefix prefix used for TF frames
	 * @return the marker created or null if TF lookup failed
	 */
	public Marker createSphereMarker(
			Link link,
			Time timepoint,
			int id,
			String prefix)
	{
		final Marker m = createMarker(id);
		final StampedTransform tf = lookupPose(link,timepoint,prefix);
		// Frame not available for given timepoint
		if(tf==null) return null;

		m.setType(Marker.SPHERE);
		m.getScale().setX(link.size);
		m.getScale().setY(link.size);
		m.getScale().setZ(link.size);

		m.getPose().getPosition().setX(tf.getTranslation().x);
		m.getPose().getPosition().setY(tf.getTranslation().y);
		m.getPose().getPosition().setZ(tf.getTranslation().z);

		m.getPose().getOrientation().setX(0.0);
		m.getPose().getOrientation().setY(0.0);
		m.getPose().getOrientation().setZ(0.0);
		m.getPose().getOrientation().setW(1.0);

        return m;
	}
	
	/**
	 * Creates a cylinder marker between two points.
	 * @param source one of the links
	 * @param target the other link
	 * @param timepoint the timepoint used for looking up the transform
	 * @param id human id (to allow multiple humans in parallel)
	 * @param prefix prefix used for TF frames
	 * @return the marker created or null if TF lookup failed
	 */
	public Marker createCylinderMarker(
			Link source, Link target,
			Time timepoint,
			int id,
			String prefix)
	{
		final Marker m = createMarker(id);
		
		final StampedTransform sourceTf = lookupPose(source,timepoint,prefix);
		final StampedTransform targetTf = lookupPose(target,timepoint,prefix);
		// Frames not available for given timepoint
		if(sourceTf==null || targetTf==null) return null;
		
		m.setType(Marker.CYLINDER);

		// Vector pointing upwards
		final Vector3d up = new Vector3d(0.0,0.0,1.0);
		// Vector pointing from target joint to source joint
		final Vector3d dp = new Vector3d(targetTf.getTranslation());
		dp.sub(sourceTf.getTranslation());

		// Radius in x/z directions
		m.getScale().setX(0.1);
		m.getScale().setY(0.1);
		// The height of the cylinder
		m.getScale().setZ(dp.length());

		// Compute point between both joints
		final Vector3d center = new Vector3d(targetTf.getTranslation());
		center.add(sourceTf.getTranslation());
		center.scale(0.5);
		m.getPose().getPosition().setX(center.x);
		m.getPose().getPosition().setY(center.y);
		m.getPose().getPosition().setZ(center.z);

		// Compute the orientation
		final Vector4d orient = computeOrientation(up,dp);
		m.getPose().getOrientation().setX(orient.x);
		m.getPose().getOrientation().setY(orient.y);
		m.getPose().getOrientation().setZ(orient.z);
		m.getPose().getOrientation().setW(orient.w);

		return m;
	}
	
	private Marker createMarker(int id) {
		final Marker m = node.getTopicMessageFactory().newFromType(visualization_msgs.Marker._TYPE);

		m.getHeader().setFrameId(targetFrame);
		// Use special namespace for XSens data so that the markers don't conflict
		// with other markers
		m.setNs(getHumanMarkerNs(id));
		m.getColor().setR(1.0f);
		m.getColor().setG(1.0f);
		m.getColor().setB(0.0f);
		m.getColor().setA(1.0f);
		
		return m;
	}
	
	/**
	 * Query knowrob for links of a human skeleton.
	 */
	private Collection<String> queryLinks() {
		HashMap<String, Vector<String>> res = PrologInterface.executeQuery(
				"sub_component("+humanIndividual+", COMP), " +
				"owl_has(COMP, srdl2comp:'baseLinkOfComposition', LINK) ; " +
				"owl_has(COMP, srdl2comp:'endLinkOfComposition', LINK) ; " +
				"(sub_component(COMP, LINK), owl_individual_of(LINK, srdl2comp:'UrdfLink'))");
		final HashSet<String> linkSet = new HashSet<String>();
		linkSet.addAll(res.get("LINK"));
		return linkSet;
	}

	/**
	 * Query knowrob for succeeding links of a human skeleton.
	 */
	private String[] querySucceedingLinks(String linkName) {
		String[] succeeding;
		HashMap<String, Vector<String>> res = PrologInterface.executeQuery(
				"owl_has(" + linkName + ", srdl2comp:'succeedingJoint', A), " +
				"owl_has(A, srdl2comp:'succeedingLink', SUCC)");
		if(res !=null && res.containsKey("SUCC")) {
			final HashSet<String> succSet = new HashSet<String>();
			succSet.addAll(res.get("SUCC"));
			succeeding = new String[succSet.size()];
			int i=0;
			for(String succ : succSet) {
				succeeding[i] = getHumanTfFrame(succ);
				i += 1;
			}
		}
		else { // no succeeding links
			succeeding = new String[]{};
		}
		return succeeding;
	}

	/**
	 * Query knowrob for the size hint of a human link.
	 */
	private double getHumanLinkSize(String linkName) {
		final HashMap<String, Vector<String>> res = PrologInterface.executeQuery(
				"owl_has("+linkName+", knowrob:'radius', RAD_), strip_literal_type(RAD_, RAD)");
		if(res==null) {
			log.warn("Radius query failed for human link: " + linkName);
			return 0.1;
		}
		final Vector<String> vals = res.get("RAD");
		if(vals==null || vals.isEmpty()) {
			log.warn("No radius for human link: " + linkName);
			return 0.1;
		}
		String val = vals.firstElement();
		if(val.startsWith("'")) {
			val = val.substring(1, val.length()-1);
		}
		return Double.parseDouble(val);
	}

	/**
	 * Query knowrob for the TF frame of a human link.
	 */
	private String getHumanTfFrame(String linkName) {
		final HashMap<String, Vector<String>> res = PrologInterface.executeQuery(
				"owl_has("+linkName+", srdl2comp:'urdfName', NAME_), strip_literal_type(NAME_, NAME)");
		if(res==null) return null;
		final Vector<String> names = res.get("NAME");
		if(names==null || names.isEmpty()) return null;
		String frameName = names.firstElement();
		if(frameName.startsWith("'")) {
			frameName = frameName.substring(1, frameName.length()-1);
		}
		if(frameName.startsWith("/")) return frameName;
		else return "/"+frameName;
	}
	
	/**
	 * Computes orientation between two vectors.
	 * @param u the from vector
	 * @param v the to vector
	 * @return quaternion that transforms one vector to the other one
	 */
	private Vector4d computeOrientation(Vector3d u, Vector3d v) {
		final Vector3d uv = new Vector3d();
		uv.cross(u,v);
		double w = Math.sqrt(u.lengthSquared()*v.lengthSquared()) + u.dot(v);
		
		final Vector4d out = new Vector4d(uv.x, uv.y, uv.z, w);
		out.normalize();
		
        return out;
	}
	
	/**
	 * Lookup pose for user specified point in time.
	 * @param prefix 
	 */
	public StampedTransform lookupPose(Link link, Time time, String prefix) {
		// Use last transform if time point didn't changed
		// since last call
		
		if(!link.lastTime.equals(time)) {
			// TODO: Remove caching here. check performance difference
			link.lastTime = time;
			link.lastTransform = TFMemory.getInstance().lookupTransform(targetFrame, prefix+link.sourceFrame, time);
		}
		
		return link.lastTransform;
	}
	
	/**
	 * Special namespace for human skeleton marker messages
	 */
	public static String getHumanMarkerNs(int id) {
		return "human_pose" + id;
	}
}

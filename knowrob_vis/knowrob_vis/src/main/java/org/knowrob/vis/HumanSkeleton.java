package org.knowrob.vis;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Vector;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector4d;

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
		/**
		 * Cached pose
		 */
		private StampedTransform lastPose;
		/**
		 * Time stamp of cached pose
		 */
		private Time lastStamp = null;
		/**
		 * TF suffix of cached pose
		 */
		private String lastSuffix;

		public Link(String sourceFrame, String[] succeeding, double size) {
			super();
			this.sourceFrame = sourceFrame;
			this.succeeding = succeeding;
			this.size = size;
		}

		public StampedTransform lookupTransform(Time stamp, String suffix) {
			if(lastStamp!=null && stamp.equals(lastStamp)
					&& lastSuffix.equals(suffix)) return lastPose;
			lastStamp = stamp;
			lastSuffix = suffix;
			lastPose = TFMemory.getInstance().lookupTransform("/map", sourceFrame+suffix, stamp);
			return lastPose;
		}
	}
	/**
	 * A link in a human skeleton for a particular point in time.
	 */
	public static class StampedLink {
		/**
		 * The human link.
		 */
		final Link link;
		/**
		 * The stamped pose of the link or null.
		 */
		final StampedTransform pose;
		/**
		 * The ID of the human (in order to support different humans
		 * for the same skeleton).
		 */
		final int id;
		
		public StampedLink(Link link, Time stamp, int id, String suffix) {
			super();
			this.link = link;
			this.id = id;
			this.pose = link.lookupTransform(stamp,suffix);
		}
	}

	/**
	 * The name of the human individual as used in OWL.
	 */
	private final String humanIndividual;
	
	/**
	 * Maps TF name to human skeleton link.
	 */
	private final HashMap<String, Link> links;
	
	/**
	 * @param humanIndividual The name of the human individual as used in OWL
	 */
	public HumanSkeleton(String humanIndividual) {
		this.humanIndividual = humanIndividual;
		this.links = new HashMap<String, Link>();
		
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
	public Marker createSphereMarker(ConnectedNode node, StampedLink sl)
	{
		final Marker m = createMarker(node,sl.id);
		// Frame not available for given timepoint
		if(sl.pose==null) return null;

		m.setType(Marker.SPHERE);
		m.getScale().setX(sl.link.size);
		m.getScale().setY(sl.link.size);
		m.getScale().setZ(sl.link.size);

		m.getPose().getPosition().setX(sl.pose.getTranslation().x);
		m.getPose().getPosition().setY(sl.pose.getTranslation().y);
		m.getPose().getPosition().setZ(sl.pose.getTranslation().z);

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
	public Marker createCylinderMarker(ConnectedNode node, StampedLink sl0, StampedLink sl1)
	{
		final Marker m = createMarker(node,sl0.id);
		// Frames not available for given timepoint
		if(sl0.pose==null || sl1.pose==null) return null;
		
		m.setType(Marker.CYLINDER);

		// Vector pointing upwards
		final Vector3d up = new Vector3d(0.0,0.0,1.0);
		// Vector pointing from target joint to source joint
		final Vector3d dp = new Vector3d(sl1.pose.getTranslation());
		dp.sub(sl0.pose.getTranslation());

		// Radius in x/z directions
		m.getScale().setX(0.1);
		m.getScale().setY(0.1);
		// The height of the cylinder
		m.getScale().setZ(dp.length());

		// Compute point between both joints
		final Vector3d center = new Vector3d(sl1.pose.getTranslation());
		center.add(sl0.pose.getTranslation());
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
	
	private Marker createMarker(ConnectedNode node, int id) {
		final Marker m = node.getTopicMessageFactory().newFromType(visualization_msgs.Marker._TYPE);

		m.getHeader().setFrameId("/map");
		// Use special namespace for MOCAP data so that the markers don't conflict
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
				"owl_has("+linkName+", knowrob:'radius', literal(type(_, RAD)))");
		if(res==null) return 0.1;
		
		return Double.parseDouble(unquote(res.get("RAD").firstElement()));
	}

	/**
	 * Query knowrob for the TF frame of a human link.
	 */
	private String getHumanTfFrame(String linkName) {
		final HashMap<String, Vector<String>> res = PrologInterface.executeQuery(
				"owl_has("+linkName+", srdl2comp:'urdfName', literal(type(_, NAME)))");
		if(res==null) return null;
		
		final String frameName = unquote(res.get("NAME").firstElement());
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
	 * Special namespace for human skeleton marker messages
	 */
	public static String getHumanMarkerNs(int id) {
		return "human_pose" + id;
	}
	
	String unquote(final String in) {
		if(in.startsWith("'")) {
			return in.substring(1, in.length()-1);
		}
		else {
			return in;
		}
	}
}

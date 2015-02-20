package org.knowrob.vis;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Vector;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector4d;

import org.knowrob.owl.OWLThing;
import org.knowrob.prolog.PrologInterface;
import org.knowrob.tfmemory.TFMemory;
import org.ros.message.Time;
import org.ros.node.ConnectedNode;

import tfjava.StampedTransform;
import visualization_msgs.Marker;

/**
 * Represents the skeleton of an agent.
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
public class Skeleton {
	/**
	 * A link in the skeleton of an agent.
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
		 * Scale hint for rendering.
		 */
		double scale[] = new double[] {1.0, 1.0, 1.0};
		/**
		 * Color hint for rendering.
		 */
		public float[] color = null;
		/**
		 * Path to CAD model.
		 */
		public String modelPath = null;

		public Link(String sourceFrame, String[] succeeding) {
			super();
			this.sourceFrame = sourceFrame;
			this.succeeding = succeeding;
		}

		public StampedTransform lookupTransform(Time stamp, String suffix,String tfPrefix) {
			return TFMemory.getInstance().lookupTransform(
					MarkerVisualization.getReferenceFrame(),
					tfPrefix + sourceFrame + suffix, stamp);
		}
	}
	/**
	 * A link in a skeleton for a particular point in time.
	 */
	public static class StampedLink {
		/**
		 * The link.
		 */
		final Link link;
		/**
		 * The stamped pose of the link or null.
		 */
		final StampedTransform pose;
		/**
		 * The ID of the visualized agent.
		 */
		final String identifier;
		/**
		 * The prefix of TF frames.
		 */
		final String tfPrefix;
		
		public StampedLink(String identifier, Link link, Time stamp, String suffix,String tfPrefix) {
			super();
			this.link = link;
			this.identifier = identifier;
			this.tfPrefix = tfPrefix;
			this.pose = link.lookupTransform(stamp,suffix,tfPrefix);
		}
	}

	/**
	 * The name of the individual as used in OWL.
	 */
	private final String individualName;
	
	/**
	 * Maps TF name to skeleton link.
	 */
	private final Map<String, Link> links;

	/**
	 * Maps marker identifier to marker.
	 */
	private final Map<String, Marker> markers;

	/**
	 * The marker type that is used for links when no CAD model is defined.
	 */
	private int linkMarkerType = Marker.CUBE;

	/**
	 * The default marker color.
	 */
	private float[] defaultColor = new float[] {0.6f, 0.6f, 0.6f, 1.0f};

	
	/**
	 * @param individualName The name of the individual as used in OWL
	 */
	public Skeleton(String individualName, Map<String, Marker> markerCache) {
		this.individualName = individualName;
		this.links = new HashMap<String, Link>();
		this.markers = markerCache;
		for(String linkName : queryLinks()) {
			final String tfFrame = getURDFName(linkName);
			if(tfFrame==null) { continue; }
			
			final String[] succ = querySucceedingLinks(linkName);
			final Link link = new Link(tfFrame, succ);
			
			final double[] scale = getLinkScale(linkName);
			link.scale[0] = scale[0];
			link.scale[1] = scale[1];
			link.scale[2] = scale[2];
			
			final float[] color = getLinkColor(linkName);
			if(color!=null) {
				link.color = new float[4];
				link.color[0] = color[0];
				link.color[1] = color[1];
				link.color[2] = color[2];
				link.color[3] = color[3];
			}
			
			link.modelPath = getModelPath(linkName);
			
			links.put(tfFrame, link);
		}
	}
	
	public void setDefaultColor(float r, float g, float b, float a) {
		defaultColor[0] = r;
		defaultColor[1] = g;
		defaultColor[2] = b;
		defaultColor[3] = a;
	}
	
	public void setMarkerType(int markerType) {
		this.linkMarkerType = markerType;
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
	 * @param node
	 * @param sl
	 * @param markerIndex
	 * @return the marker created or null if TF lookup failed
	 */
	public Marker updateLinkMarker(ConnectedNode node, StampedLink sl)
	{
		// Marker id is a concatenation of prefixed TF frame and
		// identifier of visualized agent.
		// E.g., the ID is "/human1/neck_human1" when TF prefix is "/human1",
		// the TF frame is "neck_human1" and identifier of the agent is "human1".
		StringBuilder markerId = new StringBuilder();
		markerId.append(sl.tfPrefix).append(sl.link.sourceFrame);
		markerId.append("_");
		markerId.append(sl.identifier);
		
		final Marker m = getMarker(node,markerId.toString());
		// Frame not available for given timepoint
		if(sl.pose==null) return null;

		m.setType(linkMarkerType);

		m.getPose().getPosition().setX(sl.pose.getTranslation().x);
		m.getPose().getPosition().setY(sl.pose.getTranslation().y);
		m.getPose().getPosition().setZ(sl.pose.getTranslation().z);
		m.getPose().getOrientation().setX(sl.pose.getRotation().x);
		m.getPose().getOrientation().setY(sl.pose.getRotation().y);
		m.getPose().getOrientation().setZ(sl.pose.getRotation().z);
		m.getPose().getOrientation().setW(sl.pose.getRotation().w);
		
		if(sl.link.color != null) {
			m.getColor().setR(sl.link.color[0]);
			m.getColor().setG(sl.link.color[1]);
			m.getColor().setB(sl.link.color[2]);
			m.getColor().setA(sl.link.color[3]);
		}
		else {
			m.getColor().setR(defaultColor[0]);
			m.getColor().setG(defaultColor[1]);
			m.getColor().setB(defaultColor[2]);
			m.getColor().setA(defaultColor[3]);
		}
		
		if(sl.link.modelPath != null && !sl.link.modelPath.isEmpty()) {
			m.setType(Marker.MESH_RESOURCE);
			m.setMeshResource(sl.link.modelPath);
			m.getScale().setX(sl.link.scale[0]>0.0 ? sl.link.scale[0] : 1.0);
			m.getScale().setY(sl.link.scale[1]>0.0 ? sl.link.scale[1] : 1.0);
			m.getScale().setZ(sl.link.scale[2]>0.0 ? sl.link.scale[2] : 1.0);
			if(sl.link.modelPath.endsWith(".dae") || sl.link.modelPath.endsWith(".DAE")) {
				m.setMeshUseEmbeddedMaterials(true);
				// Color must be set to zero for mesh textures
				m.getColor().setR(0.0f);
				m.getColor().setG(0.0f);
				m.getColor().setB(0.0f);
				m.getColor().setA(0.0f);
			}
		}
		else {
			// Scale down primitive geometry such as boxes if no scale is defined in OWL file
			m.getScale().setX(sl.link.scale[0]>0.0 ? sl.link.scale[0] : 0.05);
			m.getScale().setY(sl.link.scale[1]>0.0 ? sl.link.scale[1] : 0.05);
			m.getScale().setZ(sl.link.scale[2]>0.0 ? sl.link.scale[2] : 0.05);
		}

        return m;
	}
	
	/**
	 * Creates a cylinder marker between two links.
	 * @param node
	 * @param sl0
	 * @param sl1
	 * @param markerIndex
	 * @return the marker created or null if TF lookup failed
	 */
	public Marker createCylinderMarker(ConnectedNode node, StampedLink sl0, StampedLink sl1)
	{
		// Marker id is a concatenation of prefixed TF frames and
		// identifier of visualized agent.
		// E.g., the ID is "/human1/neck_/human1/head_human1"
		// For cylinder between "/human1/neck" and "/human1/head" for agent with id "human1".
		StringBuilder markerId = new StringBuilder();
		markerId.append(sl0.tfPrefix).append(sl0.link.sourceFrame);
		markerId.append("_");
		markerId.append(sl1.tfPrefix).append(sl1.link.sourceFrame);
		markerId.append("_");
		markerId.append(sl0.identifier);
		
		final Marker m = getMarker(node,markerId.toString());
		// Frames not available for given timepoint
		if(sl0.pose==null || sl1.pose==null) return null;

		m.getColor().setR(defaultColor[0]);
		m.getColor().setG(defaultColor[1]);
		m.getColor().setB(defaultColor[2]);
		m.getColor().setA(defaultColor[3]);
		
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
	
	private Marker getMarker(ConnectedNode node, String identifier) {
		Marker m = markers.get(identifier);
		
		if(m==null) {
			m = node.getTopicMessageFactory().newFromType(visualization_msgs.Marker._TYPE);
			m.getHeader().setFrameId(MarkerVisualization.getReferenceFrame());
			m.setId(0);
			m.setNs(identifier);
			m.getColor().setR(defaultColor[0]);
			m.getColor().setG(defaultColor[1]);
			m.getColor().setB(defaultColor[2]);
			m.getColor().setA(defaultColor[3]);
			markers.put(identifier,m);
		}
		
		return m;
	}
	
	/**
	 * Query knowrob for links of a human skeleton.
	 */
	private Collection<String> queryLinks() {
		HashMap<String, Vector<String>> res = PrologInterface.executeQuery(
				"owl_has("+individualName+", srdl2comp:'subComponent', _Component), " +
				"owl_has(_Component, srdl2comp:'baseLinkOfComposition', _BaseLink), " +
				"rdf_reachable(_BaseLink, srdl2comp:'successorInKinematicChain', Link), " +
				"owl_individual_of(Link, srdl2comp:'UrdfLink')");
		
		final HashSet<String> linkSet = new HashSet<String>();
		linkSet.addAll(res.get("Link"));
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
				succeeding[i] = getURDFName(succ);
				i += 1;
			}
		}
		else { // no succeeding links
			succeeding = new String[]{};
		}
		return succeeding;
	}

	/**
	 * Query knowrob for the size hint of a link.
	 */
	private double[] getLinkScale(String linkName) {

		// read object dimensions if available
		String query = "object_dimensions("+linkName+", D, W, H)";
		//log.info(query);
		HashMap<String, Vector<String>> res = PrologInterface.executeQuery(query);

		double[] out = new double[3];
		if (res!=null && res.get("D") != null && res.get("D").size() > 0 && res.get("D").get(0)!=null) {
			out[0] = Double.valueOf(OWLThing.removeSingleQuotes(res.get("D").get(0)));
			out[1] = Double.valueOf(OWLThing.removeSingleQuotes(res.get("W").get(0)));
			out[2] = Double.valueOf(OWLThing.removeSingleQuotes(res.get("H").get(0)));

		} else {
			out[0] = -1.0;
			out[1] = -1.0;
			out[2] = -1.0;
		}
		
		return out;
	}

	/**
	 * Query knowrob for the color of a link.
	 */
	private float[] getLinkColor(String linkName) {
		final HashMap<String, Vector<String>> res = PrologInterface.executeQuery("object_color("+linkName+", COL)");
		if(res==null) return null;
		String c[] = res.get("COL").get(0).split(" ");
		if(c.length!=4) return null;
		
		float[] out = new float[4];
		for(int i=0; i<4; ++i) {
			out[i] = Float.parseFloat(OWLThing.removeSingleQuotes(c[i]));
		}
		
		return out;
	}

	/**
	 * Query knowrob for the TF frame of a link.
	 */
	private String getURDFName(String linkName) {
		final HashMap<String, Vector<String>> res = PrologInterface.executeQuery(
				"owl_has("+linkName+", srdl2comp:'urdfName', literal(NAME))");
		if(res==null) return null;
		
		final String frameName = unquote(res.get("NAME").firstElement());
		if(frameName.startsWith("/")) return frameName;
		else return "/"+frameName;
	}

	/**
	 * Query knowrob for the model path of a link.
	 */
	private String getModelPath(String linkName) {
		final HashMap<String, Vector<String>> res = PrologInterface.executeQuery(
				"owl_has("+linkName+", knowrob:'pathToCadModel', literal(type(_, MODEL)))");
		if(res==null) return null;
		else return unquote(res.get("MODEL").firstElement());
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
	
	String unquote(final String in) {
		if(in.startsWith("'")) {
			return in.substring(1, in.length()-1);
		}
		else {
			return in;
		}
	}
}

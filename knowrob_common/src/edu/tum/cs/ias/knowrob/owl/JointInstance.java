package edu.tum.cs.ias.knowrob.owl;

import java.util.Vector;

import javax.vecmath.Vector3d;

import edu.tum.cs.ias.knowrob.prolog.PrologInterface;
import edu.tum.cs.ias.knowrob.prolog.PrologQueryUtils;


public class JointInstance extends ObjectInstance {

	private final static String prop_qmin = "http://ias.cs.tum.edu/kb/knowrob.owl#minJointValue";
	private final static String prop_qmax = "http://ias.cs.tum.edu/kb/knowrob.owl#maxJointValue";
	private final static String prop_radius = "http://ias.cs.tum.edu/kb/knowrob.owl#turnRadius";
	
	/**
	 * Direction of the prismatic joint
	 */
	public Vector3d direction;

	/**
	 * Child object that this joint is connected to
	 */
	public ObjectInstance child;

	/**
	 * Parent object that this joint is connected to
	 */
	public ObjectInstance parent;
	


	/**
	 * Constructor. Set the IRI and optionally a label. If none is given, 
	 * it is initialized with the IRI's short name.
	 * 
	 * @param iri Identifier of this thing.
	 */
	protected JointInstance(String iri, String label) {
		
		super(iri, label);
		this.direction = new Vector3d();
	}


	/**
	 * Copy constructor: create MapJoint from more generic {@link ObjectInstance}
	 * 
	 * @param ind {@link ObjectInstance} to be copied into this {@link JointInstance}
	 */
	protected JointInstance(ObjectInstance ind) {
		
		this(ind.getIRI(), ind.getLabel());
		this.types.addAll(ind.getTypes());
		this.data_props.putAll(ind.getDataProperties());
		this.obj_props.putAll(ind.getObjProperties());
		
		this.dimensions.set(ind.getDimensions());
		this.pose_matrix.set(ind.getPoseMatrix());
		
		this.physicalParts.addAll(ind.getPhysicalParts());
	}
	
	
	/**
	 * MapJoint factory. Return existing instance, if available, and create new
	 * MapJoint instance if necessary. Avoids duplicate instances with the same 
	 * IRI.
	 * 
	 * @param iri Identifier of this thing.
	 * @param label Optional natural-language label.
	 * @return Instance of an {@link JointInstance} with the specified IRI
	 */
	public static JointInstance getMapJoint(String iri, String label) {

		// return exact match if available
		if(identifiers.containsKey(iri) && identifiers.get(iri) instanceof JointInstance) {
			return (JointInstance) identifiers.get(iri);			
		}
		
		// create ObjectInstance from higher-level objects if the existing object for this IRI has a more abstract type
		JointInstance res = new JointInstance(ObjectInstance.getObjectInstance(iri, label));
		identifiers.put(iri, res);
		return res;
	}
	
	/**
	 * MapJoint factory. Return existing instance, if available, and create new
	 * MapJoint instance if necessary. Avoids duplicate instances with the same 
	 * IRI.
	 * 
	 * @param iri Identifier of this thing.
	 * @return Instance of an {@link JointInstance} with the specified IRI
	 */
	public static ObjectInstance getMapJoint(String iri) {
		return getMapJoint(iri, null); 
	}
	
	
	
	public double getQ_min() {
		if(data_props.containsKey(prop_qmin)) {
			return Double.valueOf(data_props.get(prop_qmin).firstElement());
		} else return -1;
	}


	public void setQ_min(double q_min) {
		
		if(data_props.containsKey(prop_qmin)) {
			data_props.get(prop_qmin).clear();
			data_props.get(prop_qmin).add(q_min+"");
		} else {
			data_props.put(prop_qmin, new Vector<String>()).add(q_min+"");
		}
	}


	public double getQ_max() {
		if(data_props.containsKey(prop_qmax)) {
			return Double.valueOf(data_props.get(prop_qmax).firstElement());
		} else return -1;
	}


	public void setQ_max(double q_max) {
		
		if(data_props.containsKey(prop_qmax)) {
			data_props.get(prop_qmax).clear();
			data_props.get(prop_qmax).add(q_max+"");
		} else {
			data_props.put(prop_qmax, new Vector<String>()).add(q_max+"");
		}
	}


	public Vector3d getDirection() {
		return direction;
	}


	public void setDirection(Vector3d direction) {
		this.direction = direction;
	}


	public ObjectInstance getChild() {
		return child;
	}


	public void setChild(ObjectInstance child) {
		this.child = child;
	}


	public ObjectInstance getParent() {
		return parent;
	}

	public void setParent(ObjectInstance parent) {
		this.parent = parent;
	}

	public double getRadius() {
		if(data_props.containsKey(prop_radius)) {
			return Double.valueOf(data_props.get(prop_radius).firstElement());
		} else return -1;
	}

	public void setRadius(double radius) {
		
		if(data_props.containsKey(prop_radius)) {
			data_props.get(prop_radius).clear();
			data_props.get(prop_radius).add(radius+"");
		} else {
			data_props.put(prop_radius, new Vector<String>()).add(radius+"");
		}
	}
	
	
	public void writeToProlog() {

		// write general object instance-related stuff
		super.writeToProlog();

		
		// set joint connection between parent and child
		if(types.contains(OWLClass.getOWLClass("http://ias.cs.tum.edu/kb/knowrob.owl#HingedJoint")) && 	child!= null && parent!=null) {
			PrologInterface.executeQuery("rdf_retractall('" + parent.getIRI() + "', 'http://ias.cs.tum.edu/kb/knowrob.owl#hingedTo', _)");
			PrologQueryUtils.assertObjectPropertyForInst(parent.getIRI(), "http://ias.cs.tum.edu/kb/knowrob.owl#hingedTo",  child.getIRI());
			
		} else if(types.contains(OWLClass.getOWLClass("http://ias.cs.tum.edu/kb/knowrob.owl#PrismaticJoint")) && child!= null && parent!=null) {
			PrologInterface.executeQuery("rdf_retractall('" + parent.getIRI() + "', 'http://ias.cs.tum.edu/kb/knowrob.owl#prismaticallyConnectedTo', _)");
			PrologQueryUtils.assertObjectPropertyForInst(parent.getIRI(), "http://ias.cs.tum.edu/kb/knowrob.owl#prismaticallyConnectedTo",  child.getIRI());
		}

		// set rigid connection between joint and parent/child resp.
		if(child!= null) {
			PrologInterface.executeQuery("rdf_retractall('" + iri + "', 'http://ias.cs.tum.edu/kb/knowrob.owl#connectedTo-Rigidly', _)");
			PrologQueryUtils.assertObjectPropertyForInst(iri, "http://ias.cs.tum.edu/kb/knowrob.owl#connectedTo-Rigidly",  child.getIRI());
		}

		if(parent!=null) {
			PrologInterface.executeQuery("rdf_retractall('" + iri + "', 'http://ias.cs.tum.edu/kb/knowrob.owl#connectedTo-Rigidly', _)");
			PrologQueryUtils.assertObjectPropertyForInst(iri, "http://ias.cs.tum.edu/kb/knowrob.owl#connectedTo-Rigidly",  parent.getIRI());
		}

		// write direction vector
		if(direction!=null && !(direction.x==0 && direction.y==0 && direction.z==0)) {
			
			PrologInterface.executeQuery("rdf_has('"+ iri + "', knowrob:'direction', DirVec),!, " +
										 "rdf_retractall('"+ iri + "', knowrob:direction, DirVec), " +
										 "rdf_retractall(DirVec, _, _)");
			
			PrologInterface.executeQuery(
			  "rdf_instance_from_class(knowrob:'Vector', DirVec)," +
		      "rdf_assert(DirVec, knowrob:'vectorX', literal(type(xsd:float, " + direction.x + ")))," +
		      "rdf_assert(DirVec, knowrob:'vectorY', literal(type(xsd:float, " + direction.y + ")))," +
		      "rdf_assert(DirVec, knowrob:'vectorZ', literal(type(xsd:float, " + direction.z + ")))," +
		      "rdf_assert('"+ iri + "', knowrob:'direction', DirVec)");
		}
		
		
	}
	
}

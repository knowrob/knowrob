package edu.tum.cs.ias.knowrob.owl.utils;

import java.io.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.io.RDFXMLOntologyFormat;
import org.semanticweb.owlapi.model.*;
import org.semanticweb.owlapi.util.DefaultPrefixManager;
import org.semanticweb.owlapi.reasoner.*;
import org.semanticweb.owlapi.reasoner.structural.*;

import edu.tum.cs.ias.knowrob.owl.OWLThing;
import edu.tum.cs.ias.knowrob.owl.ObjectInstance;
import edu.tum.cs.ias.knowrob.owl.JointInstance;
import edu.tum.cs.ias.knowrob.utils.ros.RosUtilities;



/**
 * 
 * Utilities for the import and export of OWL files 
 * from/to Java data structures
 * 
 * @author Moritz Tenorth, tenorth@cs.tum.edu
 * @author Lars Kunze, kunzel@cs.tum.edu
 *
 */


public class OWLImportExport {


	////////////////////////////////////////////////////////////////////////////////
	// Set IRIs for the ontologies used here
	//

	// Base IRI for KnowRob ontology
	public final static String KNOWROB = "http://ias.cs.tum.edu/kb/knowrob.owl#";

	// Base IRI for OWL ontology
	public final static String OWL = "http://www.w3.org/2002/07/owl#";

	// Base IRI for RDFS
	public final static String RDFS = "http://www.w3.org/2000/01/rdf-schema#";

	// Base IRI for semantic map ontology	
	public final static String IAS_MAP = "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#";

	// ROS package name for KnowRob
	public final static String KNOWROB_PKG = "ias_knowledge_base";

	// OWL file of the KnowRob ontology (relative to KNOWROB_PKG)
	public final static String KNOWROB_OWL = "owl/knowrob.owl";

	// Prefix manager
	public final static DefaultPrefixManager PREFIX_MANAGER = new DefaultPrefixManager(KNOWROB);
	static {
		PREFIX_MANAGER.setPrefix("knowrob:", KNOWROB);
		PREFIX_MANAGER.setPrefix("owl:",    OWL);
		PREFIX_MANAGER.setPrefix("rdfs:", RDFS);
	}

	// mapping ROS-KnowRob identifiers
	protected static final HashMap<String, String> rosToKnowrob = new HashMap<String, String>();

	OWLDataFactory factory;
	OWLOntologyManager manager;
	DefaultPrefixManager pm;

	public OWLImportExport() {
		//readKnowRobObjectClasses();
	}


	/**
	 * Build a complete semantic map, including name spaces, all contained objects,
	 * and the links between them.
	 *
	 * @param namespace Namespace of the OWL file to be created 
	 * @param map_id Instance identifier of the semantic map
	 * @param map List of {@link ObjectInstance}s
	 * @param address 
	 * @return
	 */
	
	public OWLOntology createOWLMapDescription(String namespace, String map_id, ArrayList<ObjectInstance> map) {
		return createOWLMapDescription(namespace, map_id, map, null);
	}
	
	public OWLOntology createOWLMapDescription(String namespace, String map_id, ArrayList<ObjectInstance> map, ArrayList<String[]> address) {

		OWLOntology ontology = null;
		HashMap<String, OWLNamedIndividual> idToInst = new HashMap<String, OWLNamedIndividual>(); 
		
		try {
			
			// Create ontology manager and data factory
			manager = OWLManager.createOWLOntologyManager();
			factory = manager.getOWLDataFactory();

			// Get prefix manager using the base IRI of the JoystickDrive ontology as default namespace
			pm = PREFIX_MANAGER;

			// Create empty OWL ontology
			ontology = manager.createOntology(IRI.create(namespace));
			PREFIX_MANAGER.setPrefix("map:", namespace);
			manager.setOntologyFormat(ontology, new RDFXMLOntologyFormat());

			// Import KnowRob ontology
			OWLImportsDeclaration oid = factory.getOWLImportsDeclaration(IRI.create(KNOWROB));
			AddImport addImp = new AddImport(ontology,oid);
			manager.applyChange(addImp);

			
			// create SemanticMap object in the ontology
			idToInst.put(namespace + map_id, createSemMapInst("map:", map_id, ontology));

			
			// create address part of the map if 'address' variable is not null
			if(address!=null) {
				
				OWLNamedIndividual child = null;
				

				OWLNamedIndividual sem_map_inst = idToInst.get(namespace+map_id);
				
				for(String[] component : address) {

					child = createAndLinkAddressPart(component, child, sem_map_inst, namespace, ontology);

				}
			}
			

			// create time point 
			OWLNamedIndividual time_inst = createTimePointInst(System.currentTimeMillis()/1000, ontology);

			
			// iterate over all objects and create the respective OWL representations
			for(ObjectInstance map_obj : map) {
				idToInst.put(map_obj.getIRI(), createSemObjectInstanceDescription(map_obj, time_inst, ontology));
			}

			
			// link to parent objects (second loop to avoid problems due to wrong ordering)
			for(ObjectInstance map_obj : map) {
				
				OWLNamedIndividual obj_inst = idToInst.get(map_obj.getIRI());

				// link high-level objects to the map
				if( !(map_obj.hasType("Handle") ||
						map_obj.hasType("HingedJoint") ||
						map_obj.hasType("PrismaticJoint") ||
						map_obj.hasType("ControlKnob") ||
						map_obj.hasType("Door"))) {

					// top-level object, link to map
					OWLObjectProperty describedInMap = factory.getOWLObjectProperty("knowrob:describedInMap", pm);
					manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(describedInMap, obj_inst, idToInst.get(namespace + map_id)));
				}
				
				// write dimensions
				if(map_obj.getDimensions()!=null) {

					OWLDataProperty width = factory.getOWLDataProperty("knowrob:depthOfObject",  pm);
					if(width!=null)
						manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(width, obj_inst, map_obj.getDimensions().x));
					
					OWLDataProperty depth = factory.getOWLDataProperty("knowrob:widthOfObject",  pm);
					if(depth!=null)
						manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(depth, obj_inst, map_obj.getDimensions().y));
					
					OWLDataProperty height = factory.getOWLDataProperty("knowrob:heightOfObject",  pm);
					if(height!=null)
						manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(height, obj_inst, map_obj.getDimensions().z));
					
				}
				

				// write all normal properties contained in the properties hashmap	
				for(String prop : map_obj.getObjProperties().keySet()) {
					for(String val : map_obj.getObjPropValues(prop)) {
						
						// skip values that are written later on
						if(prop.contains("direction") ||
							prop.contains("describedInMap"))
							continue;
						
						OWLObjectProperty prop_short_name = factory.getOWLObjectProperty("knowrob:" + prop.split("#")[1],  pm);
						if(prop_short_name!=null && val!=null) {
							OWLIndividual value = idToInst.get(val);
							
							if(value==null || obj_inst==null)
								continue;
							
							manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(prop_short_name, obj_inst, value));
						}
						
					}
				}
				
				// also link proper physical parts of an object		
				for(ObjectInstance p: map_obj.getPhysicalParts()) {

					OWLIndividual part = idToInst.get(p.getIRI());
					OWLObjectProperty properPhysicalParts = factory.getOWLObjectProperty("knowrob:properPhysicalParts", pm);
					manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(properPhysicalParts, obj_inst, part));
				}

				
				// link hinges in a special way (set child, parent, hingedTo)
				if(map_obj instanceof JointInstance && 
						(((JointInstance) map_obj).parent)!=null && 
						(((JointInstance) map_obj).child)!=null) {

					OWLIndividual child = idToInst.get(((JointInstance) map_obj).child.getIRI());
					OWLIndividual parent = idToInst.get(((JointInstance) map_obj).parent.getIRI());
				
					if(parent==null || child ==null)
						continue;
					
					// set joint connection between parent and child
					if(map_obj.hasType("HingedJoint")) {
						OWLObjectProperty hingedTo = factory.getOWLObjectProperty("knowrob:hingedTo", pm);
						manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(hingedTo, parent, child));
						
					} else if(map_obj.hasType("PrismaticJoint")) {
						OWLObjectProperty prismaticallyConnectedTo = factory.getOWLObjectProperty("knowrob:prismaticallyConnectedTo", pm);
						manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(prismaticallyConnectedTo, parent, child));
					}
					
					// set rigid connection between joint and parent/child resp.
					OWLObjectProperty connectedTo = factory.getOWLObjectProperty("knowrob:connectedTo-Rigidly", pm);
						manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(connectedTo, obj_inst, child));
						manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(connectedTo, obj_inst, parent));
				}
			}
			
		} catch (Exception e) {
			ontology = null;
			e.printStackTrace();
		}

		return ontology;
	}


	
	/**
	 * Create the address part of a semantic map. It contains a hierarchy of individuals, e.g. City-Street-Building-Floor-Room.
	 * 
	 * @param component String[] of the form ["knowrob:RoomInAConstruction", "knowrob:roomNumber", "3010"]
	 * @param child Optional reference to a spatial child of the current object, e.g. a room instance if the current element 
	 *         is a floor; null if the current element is on the lowest level above the SemanticEnvironmentMap instance
	 * @param sem_map_inst {@link OWLIndividual} of a SemanticEnvironmentMap 
	 * @param ontology {@link OWLOntology} to which the axioms will be added
	 * @return
	 */
	public OWLNamedIndividual createAndLinkAddressPart(String[] component, OWLNamedIndividual child, OWLNamedIndividual sem_map_inst, String namespace, OWLOntology ontology) {
		
		// create individual 
		OWLClass obj_class = factory.getOWLClass(component[0], pm);
		OWLNamedIndividual obj_inst = factory.getOWLNamedIndividual("map:"+edu.tum.cs.ias.knowrob.owl.OWLIndividual.getOWLIndividualOfClass(obj_class.getIRI().toString()).getShortName(), pm);
		manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(obj_class, obj_inst));
		
		
		// link lowest level with 'describedInMap'
		
		OWLObjectProperty describedInMap = factory.getOWLObjectProperty("knowrob:describedInMap", pm);
		OWLObjectProperty properPhysicalParts = factory.getOWLObjectProperty("knowrob:properPhysicalParts", pm);
		
		if(child==null) {
			manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(describedInMap, obj_inst, sem_map_inst));
			
		} else { // link higher levels with 'properPhysicalParts'
			manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(properPhysicalParts, obj_inst, child));
		}
		
		// set properties (roomNumber, label, etc)
		OWLDataProperty property = factory.getOWLDataProperty(component[1],  pm);
		if(property!=null)
			manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(property, obj_inst, component[2]));
		
		return obj_inst;
	}


	
	/**
	 * Create the OWL description for an object, including the object instance with its dimension,
	 * a pose instance where the object has been detected, and a SemanticMapPerception instance 
	 * linking object, pose, and detection time
	 * 
	 * @param map_obj   ObjectInstance input data
	 * @param timestamp OWLIndividual for the time when map_obj has been perceived
	 * @param ontology  Ontology to which the axioms are to be added
	 */
	public OWLNamedIndividual createSemObjectInstanceDescription(ObjectInstance map_obj, OWLNamedIndividual timestamp, OWLOntology ontology) {

		// create object instance
		OWLNamedIndividual obj_inst = createObjectInst(map_obj, ontology);

		// create pose matrix instance
		OWLNamedIndividual pose_inst = createPoseInst(map_obj.getPoseMatrix(), manager, factory, pm, ontology);

		// create perception instance
		createPerceptionInst("knowrob:SemanticMapPerception", obj_inst, pose_inst, timestamp, ontology);

		return obj_inst;
	}


	
	/**
	 * Create an instance of a knowrob:SemanticEnvironmentMap
	 * 
	 * @param map_id 
	 * @param namespace 
	 * @param ontology  Ontology to which the axioms are to be added
	 * @return 			Created instance of a SemanticEnvironmentMap
	 */
	public OWLNamedIndividual createSemMapInst(String namespace, String map_id, OWLOntology ontology) {

		OWLClass sem_map_class = factory.getOWLClass("knowrob:SemanticEnvironmentMap", pm);
		OWLNamedIndividual sem_map_inst = factory.getOWLNamedIndividual(namespace + map_id, pm);
		manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(sem_map_class, sem_map_inst));

		return sem_map_inst;
	}

	
	
	/**
	 * Generate an instance of the object class indicated by map_obj.type and link it to its parent object
	 * 
	 * @param map_obj   ObjectInstance input data
	 * @param ontology  Ontology to which the axioms are to be added
	 * @return 			Created instance of the respective object
	 */
	public OWLNamedIndividual createObjectInst(ObjectInstance map_obj, OWLOntology ontology) {

		OWLNamedIndividual obj_inst = factory.getOWLNamedIndividual("map:"+map_obj.getShortName(), pm);
		
		for(edu.tum.cs.ias.knowrob.owl.OWLClass t : map_obj.getTypes()) {
			OWLClass obj_class = factory.getOWLClass("knowrob:"+t.getShortName(), pm);
			manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(obj_class, obj_inst));	
		}
	
		
		// write all normal data properties contained in the properties hashmap	
		for(String prop : map_obj.getDataProperties().keySet()) {
			for(String val : map_obj.getDataPropValues(prop)) {


				if( prop.endsWith("depthOfObject") || 
					prop.endsWith("widthOfObject") || 
					prop.endsWith("heightOfObject") )
					continue;
				
				
				OWLDataProperty property = factory.getOWLDataProperty("knowrob:" + prop.split("#")[1],  pm);
				if(property!=null)
					manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(property, obj_inst, val));
				
			}
		}
			
		// create hinge-specific properties
		if(map_obj instanceof JointInstance) {
			
			// set direction for prismatic joints
			if(map_obj.hasType("PrismaticJoint")) {
				
				OWLNamedIndividual dir_vec = createDirVector(((JointInstance) map_obj).direction, manager, factory, pm, ontology);
				
				OWLObjectProperty direction = factory.getOWLObjectProperty("knowrob:direction", pm);
				manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(direction, obj_inst, dir_vec));
			}
		}
		
		return obj_inst;
	}

	
	
	/**
	 * Create an instance of a knowrob:TimePoint for time t
	 * 
	 * @param t         Time to be translated into a TimePoint
	 * @param ontology  Ontology to which the axioms are to be added
	 * @return 			Created instance of a TimePoint
	 */
	public OWLNamedIndividual createTimePointInst(long stamp, OWLOntology ontology) {

		OWLNamedIndividual time_inst = factory.getOWLNamedIndividual("map:timepoint_"+stamp, pm);
		OWLClass time_class = factory.getOWLClass("knowrob:TimePoint", pm);
		manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(time_class, time_inst));

		return time_inst;
	}

	
	
	/**
	 * Create a RotationMatrix3D with all m_ij components set according to the pose of the map_obj
	 * 
	 * @param map_obj   ObjectInstance input data
	 * @param ontology  Ontology to which the axioms are to be added
	 * @return 			Created instance of a RotationMatrix3D
	 */
	public static OWLNamedIndividual createPoseInst(Matrix4d pose, OWLOntologyManager manager, OWLDataFactory factory, DefaultPrefixManager pm, OWLOntology ontology) { 

		// create pose matrix instance
		OWLClass pose_class = factory.getOWLClass("knowrob:RotationMatrix3D", pm);
		OWLNamedIndividual pose_inst = factory.getOWLNamedIndividual(OWLThing.getUniqueID("knowrob:RotationMatrix3D"), pm);
		manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(pose_class, pose_inst));

		
		// set pose properties
		for(int i=0;i<4;i++) {
			for(int j=0;j<4;j++) {
				OWLDataProperty prop = factory.getOWLDataProperty("knowrob:m"+i+j, pm);
				manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(prop,  pose_inst, pose.getElement(i,j)));
			}
		}

		return pose_inst;
	}	

	/**
	 * Create a Vector with all i components set according to the direction of the map_joint
	 * 
	 * @param dir_vec   Input data vector
	 * @param ontology  Ontology to which the axioms are to be added
	 * @return 			Created instance of a Vector
	 */
	public static OWLNamedIndividual createDirVector(Vector3d dir_vec, OWLOntologyManager manager, OWLDataFactory factory, DefaultPrefixManager pm, OWLOntology ontology) { 


		// create vector instance
		OWLClass vec_class = factory.getOWLClass("knowrob:Vector", pm);
		OWLNamedIndividual vec_inst = factory.getOWLNamedIndividual(OWLThing.getUniqueID("knowrob:Vector"), pm);
		manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(vec_class, vec_inst));

		// set vector dimensions
		OWLDataProperty vecX = factory.getOWLDataProperty("knowrob:vectorX", pm);
		OWLDataProperty vecY = factory.getOWLDataProperty("knowrob:vectorY", pm);
		OWLDataProperty vecZ = factory.getOWLDataProperty("knowrob:vectorZ", pm);
		
		manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(vecX,  vec_inst, dir_vec.x));
		manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(vecY,  vec_inst, dir_vec.y));
		manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(vecZ,  vec_inst, dir_vec.z));

		return vec_inst;
	}
	
	/**
	 * Create an instance of a SemanticMapPerception linking objects to poses and times
	 * 
	 * @param type      Type of the perception, e.g. "knowrob:SemanticMapPerception"
	 * @param obj_inst  The object that was detected
	 * @param pose_inst Pose where the object was detected
	 * @param timestamp Time when the object was detected
	 * @param ontology  Ontology to which the axioms are to be added
	 * @return 			Created instance of SemanticMapPerception
	 */
	public OWLNamedIndividual createPerceptionInst(String type, OWLNamedIndividual obj_inst, OWLNamedIndividual pose_inst, OWLNamedIndividual timestamp, OWLOntology ontology) {

		// create perception instance
		OWLClass perc_class = factory.getOWLClass(type, pm);
		OWLNamedIndividual perc_inst = factory.getOWLNamedIndividual(
				OWLThing.getUniqueID(type), pm);
		manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(perc_class, perc_inst));

		// link to the object instance and the pose instance
		OWLObjectProperty objectActedOn = factory.getOWLObjectProperty("knowrob:objectActedOn", pm);
		OWLObjectProperty eventOccursAt = factory.getOWLObjectProperty("knowrob:eventOccursAt", pm);

		manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(objectActedOn, perc_inst, obj_inst));
		manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(eventOccursAt, perc_inst, pose_inst));

		// set time stamp
		OWLObjectProperty startTime = factory.getOWLObjectProperty("knowrob:startTime", pm);
		manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(startTime,  perc_inst, timestamp));

		return perc_inst;
	}


	
	/**
	 * Read a semantic map from an OWL file into a HashMap representation, using the object
	 * identifier as key and the object pose description as value.
	 * 
	 * @param filename Name of the OWL file to be imported
	 * @return HashMap with the object identifiers as key and the ObjectInstance data structures as values
	 */
	static public HashMap<String, ObjectInstance> readObjectInstanceFromOWL(String filename) {
		
		HashMap<String, ObjectInstance> objects = new HashMap<String, ObjectInstance>();
    	OWLOntology ont = null;
    	try {
    		
			OWLOntologyManager manager = OWLManager.createOWLOntologyManager();
			OWLDataFactory factory = manager.getOWLDataFactory();
			DefaultPrefixManager pm = OWLImportExport.PREFIX_MANAGER;
			OWLClass semanticMapPerception = factory.getOWLClass("knowrob:SemanticMapPerception", pm);
			OWLClass rotationmatrix3d = factory.getOWLClass("knowrob:RotationMatrix3D", pm);
			OWLClass timepoint = factory.getOWLClass("knowrob:TimePoint", pm);
			
			ont = OWLFileUtils.loadOntologyFromFile(filename);
			
			if(ont!=null) {

				// iterate over objects, add to objects hashmap 
				for(OWLNamedIndividual inst : ont.getIndividualsInSignature()) {
										
					Set<OWLClassExpression> types = inst.getTypes(ont);
					
					if(types.contains(semanticMapPerception)) {
						
						Map<OWLObjectPropertyExpression, Set<OWLIndividual>> perc_props = 
							inst.getObjectPropertyValues(ont);
						
						OWLObjectProperty objectActedOn = factory.getOWLObjectProperty("knowrob:objectActedOn", pm);
						OWLObjectProperty eventOccursAt = factory.getOWLObjectProperty("knowrob:eventOccursAt", pm);

						Set<OWLIndividual> objs  = perc_props.get(objectActedOn);
						Set<OWLIndividual> poses = perc_props.get(eventOccursAt);

						
						// assuming there is only one object and one pose per perception:
						if(objs != null && poses != null) {
							for(OWLIndividual obj : objs) {
								for(OWLIndividual pose : poses) {

									String iri = obj.toStringID();

									// create map object
									ObjectInstance cur = ObjectInstance.getObjectInstance(iri);

									// get types
									for(OWLClassExpression c: obj.getTypes(ont)) {
										cur.addType(edu.tum.cs.ias.knowrob.owl.OWLClass.getOWLClass(c.asOWLClass().toStringID()));
									}

									// special treatment for MapJoints
									if(cur.hasType("HingedJoint") || cur.hasType("PrismaticJoint")) {
										cur = JointInstance.getMapJoint(iri);
										for(OWLClassExpression c: obj.getTypes(ont)) {
											cur.addType(edu.tum.cs.ias.knowrob.owl.OWLClass.getOWLClass(c.asOWLClass().toStringID()));
										}
									}


									// iterate over all data properties and read them to the internal representation
									Map<OWLDataPropertyExpression, Set<OWLLiteral>> data_props = 
											obj.getDataPropertyValues(ont);

									for(OWLDataPropertyExpression prop : data_props.keySet()) {
										for(OWLLiteral d : data_props.get(prop)) {
											cur.addDataPropValue(prop.asOWLDataProperty().toStringID(), d.getLiteral());
										}
									}


									// iterate over all object properties and read them to the internal representation
									Map<OWLObjectPropertyExpression, Set<OWLIndividual>> obj_props = 
											obj.getObjectPropertyValues(ont);

									for(OWLObjectPropertyExpression prop: obj_props.keySet()) {
										for(OWLIndividual d : obj_props.get(prop)) {
											cur.addObjPropValue(prop.asOWLObjectProperty().toStringID(), d.toStringID());
										}
									}


									// get dimensions
									OWLDataProperty width  = factory.getOWLDataProperty("knowrob:widthOfObject", pm);
									OWLDataProperty depth  = factory.getOWLDataProperty("knowrob:depthOfObject", pm);
									OWLDataProperty height = factory.getOWLDataProperty("knowrob:heightOfObject", pm);

									if(data_props.get(depth) != null) {
										for(OWLLiteral d : data_props.get(depth)) {
											cur.getDimensions().x = Double.valueOf(d.getLiteral());
										}
									}

									if(data_props.get(width) != null) {
										for(OWLLiteral w : data_props.get(width)) {
											cur.getDimensions().y = Double.valueOf(w.getLiteral());
										}
									}

									if(data_props.get(height) != null) {
										for(OWLLiteral h : data_props.get(height)) {
											cur.getDimensions().z = Double.valueOf(h.getLiteral());
										}
									}

									// read hinge-specific properties
									if(cur.hasType("HingedJoint") || cur.hasType("PrismaticJoint")) {

										if(cur.hasType("PrismaticJoint")) {

											OWLObjectProperty direction = factory.getOWLObjectProperty("knowrob:direction", pm);

											if(obj_props.containsKey(direction)) {
												for(OWLIndividual dir : obj_props.get(direction)) {

													Map<OWLDataPropertyExpression, Set<OWLLiteral>> vec_props = 
															dir.getDataPropertyValues(ont);

													OWLDataProperty vectorx  = factory.getOWLDataProperty("knowrob:vectorX", pm);
													OWLDataProperty vectory  = factory.getOWLDataProperty("knowrob:vectorY", pm);
													OWLDataProperty vectorz  = factory.getOWLDataProperty("knowrob:vectorZ", pm);

													if(vec_props.containsKey(vectorx)) {
														for(OWLLiteral x : vec_props.get(vectorx)) {
															((JointInstance) cur).direction.x = Double.valueOf(x.getLiteral());
														}
													}
													if(vec_props.containsKey(vectory)) {
														for(OWLLiteral y : vec_props.get(vectory)) {
															((JointInstance) cur).direction.y = Double.valueOf(y.getLiteral());
														}
													}
													if(vec_props.containsKey(vectorz)) {
														for(OWLLiteral z : vec_props.get(vectorz)) {
															((JointInstance) cur).direction.z = Double.valueOf(z.getLiteral());
														}
													}
												}
											}
										}
									}


									// get pose elements
									Map<OWLDataPropertyExpression, Set<OWLLiteral>> matrix_elems = 
											pose.getDataPropertyValues(ont);

									for(int i=0;i<4;i++) {
										for(int j=0;j<4;j++){
											OWLDataProperty m_ij = factory.getOWLDataProperty("knowrob:m"+i+j, pm);
											Set<OWLLiteral> elem = matrix_elems.get(m_ij);

											for(OWLLiteral e : elem) {
												cur.getPoseMatrix().setElement(i, j, Double.valueOf(e.getLiteral()) );	
											}


										}
									}
									objects.put(cur.getShortName(), cur);

								}
							}
						}
					}
				}
				
				
				// add objects that have not been perceived (e.g. street, building)
				for(OWLNamedIndividual inst : ont.getIndividualsInSignature()) {
					
					String iri = inst.toStringID();
					
					if(objects.containsKey(OWLThing.getShortNameOfIRI(iri)))
						continue;
					
					// skip those types we have already  
					if( inst.getTypes(ont).contains(semanticMapPerception) ||
						inst.getTypes(ont).contains(rotationmatrix3d) ||
						inst.getTypes(ont).contains(timepoint))
						continue;

					
					// create map object
					ObjectInstance cur = ObjectInstance.getObjectInstance(iri);
					
					// get types
					for(OWLClassExpression c: inst.getTypes(ont)) {
						cur.addType(edu.tum.cs.ias.knowrob.owl.OWLClass.getOWLClass(c.asOWLClass().toStringID()));
					}
					
					// iterate over all data properties and read them to the internal representation
					Map<OWLDataPropertyExpression, Set<OWLLiteral>> data_props = 
						inst.getDataPropertyValues(ont);

					for(OWLDataPropertyExpression prop : data_props.keySet()) {
						for(OWLLiteral d : data_props.get(prop)) {
							cur.addDataPropValue(prop.asOWLDataProperty().toStringID(), d.getLiteral());
						}
					}

					// iterate over all object properties and read them to the internal representation
					Map<OWLObjectPropertyExpression, Set<OWLIndividual>> obj_props = 
						inst.getObjectPropertyValues(ont);

					for(OWLObjectPropertyExpression prop: obj_props.keySet()) {
						for(OWLIndividual d : obj_props.get(prop)) {
							cur.addObjPropValue(prop.asOWLObjectProperty().toStringID(), d.toStringID());
						}
					}
					objects.put(cur.getShortName(), cur);
				}
			
				
				
				
				// link objects to their physical parts
				for(OWLNamedIndividual inst : ont.getIndividualsInSignature()) {

					Set<OWLClassExpression> types = inst.getTypes(ont);

					if(types.contains(semanticMapPerception)) {

						Map<OWLObjectPropertyExpression, Set<OWLIndividual>> perc_props = 
							inst.getObjectPropertyValues(ont);

						OWLObjectProperty objectActedOn = factory.getOWLObjectProperty("knowrob:objectActedOn", pm);

						Set<OWLIndividual> objs  = perc_props.get(objectActedOn);

						if(objs != null) {
							for(OWLIndividual obj : objs) {

								// get proper physical parts
								Map<OWLObjectPropertyExpression, Set<OWLIndividual>> obj_props = 
										obj.getObjectPropertyValues(ont);

								OWLObjectProperty parts = factory.getOWLObjectProperty("knowrob:properPhysicalParts", pm);
								if(obj_props.containsKey(parts)) {
									for(OWLIndividual p : obj_props.get(parts)) {
										ObjectInstance part = objects.get(OWLThing.getShortNameOfIRI(p.toStringID()));
										if(part!=null) {
											objects.get(OWLThing.getShortNameOfIRI(obj.toStringID())).addPhysicalPart(part);
										}
									}
								}
							}
						}
					}
				}
				
				// read parent and child
				for(OWLNamedIndividual inst : ont.getIndividualsInSignature()) {

					Set<OWLClassExpression> types = inst.getTypes(ont);

					if(types.contains(semanticMapPerception)) {

						Map<OWLObjectPropertyExpression, Set<OWLIndividual>> perc_props = 
							inst.getObjectPropertyValues(ont);

						OWLObjectProperty objectActedOn = factory.getOWLObjectProperty("knowrob:objectActedOn", pm);

						Set<OWLIndividual> objs  = perc_props.get(objectActedOn);


						if(objs!=null) {
							for(OWLIndividual obj : objs) {

								Map<OWLObjectPropertyExpression, Set<OWLIndividual>> obj_props = 
										obj.getObjectPropertyValues(ont);
								OWLObjectProperty connectedTo = factory.getOWLObjectProperty("knowrob:connectedTo-Rigidly", pm);

								if(obj_props.containsKey(connectedTo)) {

									ObjectInstance cur = objects.get(OWLThing.getShortNameOfIRI(obj.toStringID()));

									for(OWLIndividual rel : obj_props.get(connectedTo)) {

										ObjectInstance connectedObj = objects.get(OWLThing.getShortNameOfIRI(rel.toStringID()));

										if(connectedObj!=null) {

											// connectedObj is a child if it is also contained 
											// in the physicalParts list of the current object
											if(connectedObj.hasPhysicalPart(cur)) {
												((JointInstance) cur).parent = connectedObj;
											} else {
												((JointInstance) cur).child = connectedObj;
											}

										}

									}
								}
							}
						}

					}
				}
			}
			
		} catch (OWLOntologyCreationException e) {
			e.printStackTrace();
		}
		return objects;
	}

	
	
	/**
	 * Initialization of the mapping between object types that are sent via 
	 * the ROS service and concepts of the KnowRob ontology.
	 */
	protected static void readKnowRobObjectClasses() {

		try {
			
			// Create ontology manager, data factory, and prefix manager
			OWLOntologyManager manager = OWLManager.createOWLOntologyManager();
			OWLDataFactory factory = manager.getOWLDataFactory();
			DefaultPrefixManager pm = PREFIX_MANAGER;

			// Find ros package holding the knowrob ontology
			String knowrob_pkg = RosUtilities.rospackFind(KNOWROB_PKG);
			String knowrob_owl = knowrob_pkg + "/" + KNOWROB_OWL;


			// Load the knowrob ontology  
			OWLOntology ont = manager.loadOntologyFromOntologyDocument(new File(knowrob_owl));

			// Retrieve only subclasses of SpatialThing-Localized
			OWLReasoner reasoner = new StructuralReasoner(ont, new SimpleConfiguration(), BufferingMode.NON_BUFFERING);
			OWLClass spatialThing = factory.getOWLClass("knowrob:SpatialThing-Localized", pm);
			NodeSet<OWLClass> ns = reasoner.getSubClasses(spatialThing, false);

			java.util.Set<Node<OWLClass>>  set = ns.getNodes();       

			// Iterate over all subclasses and put them into the mapping hashmap
			for(Node<OWLClass> n : set) {
				OWLClass c = (OWLClass) n.getRepresentativeElement();

				String iri = c.toStringID().replaceAll(KNOWROB, "knowrob:");
				String key = c.toStringID().substring(c.toStringID().lastIndexOf('#') + 1).toLowerCase();

				rosToKnowrob.put(key, iri);
			}
			// to support backward compatibility (should be removed)
			rosToKnowrob.put("hinge", "knowrob:HingedJoint");
			rosToKnowrob.put("knob",  "knowrob:ControlKnob");
			rosToKnowrob.put("horizontal_plane", "knowrob:CounterTop");

		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}


	/**
	 * Debug method: print all object types imported from KnowRob
	 */
	protected static void printObjectTypes() {

		for(Object o : rosToKnowrob.values()) {
			System.out.println(((String)o).replaceAll("knowrob:",""));
		}
	}


}

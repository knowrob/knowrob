package edu.tum.cs.ias.knowrob;

import java.io.*;
import java.util.Date;
import java.util.HashMap;
import java.text.SimpleDateFormat;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.io.RDFXMLOntologyFormat;
import org.semanticweb.owlapi.model.*;
import org.semanticweb.owlapi.util.DefaultPrefixManager;
import org.semanticweb.owlapi.vocab.PrefixOWLOntologyFormat;

import ros.*;
import ros.pkg.mod_semantic_map.srv.*;
import ros.pkg.mod_semantic_map.msg.*;


/**
 * 
 * Interface between the CoP cognitive perception system (http://www.ros.org/wiki/cop)
 * and the RoboEarth database.
 * 
 * Provides ROS services to upload CoP object models to the RoboEarth database as well
 * as to retrieve object models from the database and add them to CoP.
 * 
 * @author Moritz Tenorth, tenorth@cs.tum.edu
 *
 */

public class SemanticMapToOWL {


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
	public final static String IAS_MAP = "http://ias.cs.tum.edu/kb/ccrl2_semantic_map.owl#";
	
	
	// Prefix manager
	public final static DefaultPrefixManager PREFIX_MANAGER = new DefaultPrefixManager(IAS_MAP);
	static {
		PREFIX_MANAGER.setPrefix("knowrob:", KNOWROB);
		PREFIX_MANAGER.setPrefix("ias_map:", IAS_MAP);
		PREFIX_MANAGER.setPrefix("owl:", OWL);
		PREFIX_MANAGER.setPrefix("rdfs:", RDFS);
	}
	
	// mapping ROS-KnowRob identifiers
	protected static final HashMap<String, String> rosToKnowrob = new HashMap<String, String>();
	static {
		rosToKnowrob.put("cupboard",     "knowrob:Cupboard");
		rosToKnowrob.put("drawer",       "knowrob:Drawer");
		rosToKnowrob.put("oven",         "knowrob:Oven");
		rosToKnowrob.put("refrigerator", "knowrob:Refrigerator");
		rosToKnowrob.put("dishwasher",    "knowrob:Dishwasher");
		rosToKnowrob.put("table",        "knowrob:Table");
		rosToKnowrob.put("countertop",   "knowrob:CounterTop");
		rosToKnowrob.put("sink",         "knowrob:Sink");
		rosToKnowrob.put("door",         "knowrob:Door");
		rosToKnowrob.put("hinge",        "knowrob:HingedJoint");
		rosToKnowrob.put("handle",       "knowrob:Handle");
		rosToKnowrob.put("knob",         "knowrob:ControlKnob");
		
		// TODO: add a concept for horizontal planes to knowrob
		rosToKnowrob.put("horizontal_plane", "knowrob:CounterTop");
	}
	
	HashMap<Integer, OWLNamedIndividual> idToInst; 
	
	OWLDataFactory factory;
	OWLOntologyManager manager;
	DefaultPrefixManager pm;
	
	int inst_counter=0;	// counter to create unique instance identifiers
	
	////////////////////////////////////////////////////////////////////////////////
	// ROS stuff
	
	static Boolean rosInitialized = false;
	static Ros ros;
	static NodeHandle n;
	
	////////////////////////////////////////////////////////////////////////////////

		


	/**
	 * Constructor. Advertises the needed ROS services.
	 * @param ros reference to rosjava
	 * @param n the node handle
	 * @throws RosException if advertising ROS services failed
	 */
	public SemanticMapToOWL() {

		idToInst = new HashMap<Integer, OWLNamedIndividual>();
	
		try {
			
			initRos();
			
			n.advertiseService("/generate_owl_map", new GenerateSemanticMapOWL(), new ConvertToOwlCallback());
			ros.spin();
			
		} catch (RosException e) {
			e.printStackTrace();	
		}
		
	}

	
	/**
	 * Thread-safe ROS initialization
	 */
	protected static void initRos() {

    	ros = Ros.getInstance();

		if(!Ros.getInstance().isInitialized()) {
	    	ros.init("knowrob_semantic_map_to_owl");
		}
		n = ros.createNodeHandle();

	}
	
	/**
	 * 
	 * Callback class for querying the Web for the object type of a barcode
	 * 
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 *
	 */
	class ConvertToOwlCallback implements ServiceServer.Callback<GenerateSemanticMapOWL.Request, GenerateSemanticMapOWL.Response> {
		
		@Override
		public GenerateSemanticMapOWL.Response call(GenerateSemanticMapOWL.Request req) {

			GenerateSemanticMapOWL.Response res = new GenerateSemanticMapOWL.Response();
			res.owlmap="";

			if (req.map != null && req.map.objects.length>0) {
				
				OWLOntology owlmap = buildOWLMapDescription(req.map);
				res.owlmap = saveOntologytoString(owlmap, owlmap.getOWLOntologyManager().getOntologyFormat(owlmap));
				
			}
			
			return res;
		}
	}
	
	
	
	/**
	 * 
	 * @param ros_map
	 * @return
	 */
	protected OWLOntology buildOWLMapDescription(SemMap ros_map) {
		
		OWLOntology ontology = null;
		idToInst.clear();
		try {
			
			// Create ontology manager and data factory
			manager = OWLManager.createOWLOntologyManager();
			factory = manager.getOWLDataFactory();
			
			// Get prefix manager using the base IRI of the JoystickDrive ontology as default namespace
			pm = PREFIX_MANAGER;

			// Create empty OWL ontology
			ontology = manager.createOntology(IRI.create(IAS_MAP));
			manager.setOntologyFormat(ontology, new RDFXMLOntologyFormat());
			
			// Import KnowRob ontology
			OWLImportsDeclaration oid = factory.getOWLImportsDeclaration(IRI.create(KNOWROB));
			AddImport addImp = new AddImport(ontology,oid);
			manager.applyChange(addImp);

			
			// create SemanticMap object in the ontology
			createSemMapInst(ontology);
			
			// create time point 
			OWLNamedIndividual time_inst = createTimePointInst(ros_map.header.stamp, ontology);
			
			
			// iterate over all objects and create the respective OWL representations
			for(SemMapObject ros_obj : ros_map.objects) {
				createObjectDescription(ros_obj, time_inst, ontology);
			}
			
			SimpleDateFormat sdf = new SimpleDateFormat("yy-MM-dd_HH-mm-ss-SSS");
			String outfile = "ias_semantic_map_"+sdf.format(new Date())+".owl";
			saveOntologyToFile(ontology, outfile);

		} catch (Exception e) {
			ontology = null;
			e.printStackTrace();
		}
		
		return ontology;
		
	}
	
	
	/**
	 * Create the OWL description for an object, including the object instance with its dimension,
	 * a pose instance where the object has been detected, and a perception instance linking object,
	 * pose, and detection time
	 * 
	 * @param ros_obj   SemMapObject input data
	 * @param timestamp OWLIndividual for the time when ros_obj has been perceived
	 * @param ontology  Ontology to which the axioms are to be added
	 */
	protected void createObjectDescription(SemMapObject ros_obj, OWLNamedIndividual timestamp, OWLOntology ontology) {
				
		// create object instance
		OWLNamedIndividual obj_inst = createObjectInst(ros_obj, ontology);

		// create pose matrix instance
		OWLNamedIndividual pose_inst = createPoseInst(ros_obj, ontology);
		
		// create perception instance
		createPerceptionInst(obj_inst, pose_inst, timestamp, ontology);
		
		// remember mapping of ROS object ID and OWL instance (for being able to link to parents)
		this.idToInst.put(ros_obj.id, obj_inst);
				
	}
	
	/**
	 * Create an instance of a knowrob:SemanticEnvironmentMap
	 * 
	 * @param ontology  Ontology to which the axioms are to be added
	 * @return 			Created instance of a SemanticEnvironmentMap
	 */
	protected OWLNamedIndividual createSemMapInst(OWLOntology ontology) {

		OWLClass sem_map_class = factory.getOWLClass("knowrob:SemanticEnvironmentMap", pm);
		OWLNamedIndividual sem_map_inst = factory.getOWLNamedIndividual(
				                                  instForClass("ias_map:SemanticEnvironmentMap"), pm);
		manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(sem_map_class, sem_map_inst));
		
		this.idToInst.put(0, sem_map_inst);
		
		return sem_map_inst;
	}
	

	/**
	 * Create an instance of a knowrob:TimePoint for time t
	 * 
	 * @param t         Time to be translated into a TimePoint
	 * @param ontology  Ontology to which the axioms are to be added
	 * @return 			Created instance of a TimePoint
	 */
	protected OWLNamedIndividual createTimePointInst(ros.communication.Time stamp, OWLOntology ontology) {
		
		OWLNamedIndividual time_inst = factory.getOWLNamedIndividual("ias_map:timepoint_"+stamp.secs, pm);
		OWLClass time_class = factory.getOWLClass("knowrob:TimePoint", pm);
		manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(time_class, time_inst));
		
		return time_inst;
	}
	
	
	/**
	 * Generate an instance of the object class indicated by ros_obj.type and link it to its parent object
	 * 
	 * @param ros_obj   SemMapObject input data
	 * @param ontology  Ontology to which the axioms are to be added
	 * @return 			Created instance of the respective object
	 */
	protected OWLNamedIndividual createObjectInst(SemMapObject ros_obj, OWLOntology ontology) {
		
		// create object instance
		OWLClass obj_class = factory.getOWLClass(rosToKnowrob.get(ros_obj.type), pm);
		OWLNamedIndividual obj_inst = factory.getOWLNamedIndividual(
										      instForClass(rosToKnowrob.get(ros_obj.type)), pm);
		manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(obj_class, obj_inst));
		
		// set object dimensions
		OWLDataProperty width  = factory.getOWLDataProperty("knowrob:widthOfObject",  pm);
		OWLDataProperty depth  = factory.getOWLDataProperty("knowrob:depthOfObject",  pm);
		OWLDataProperty height = factory.getOWLDataProperty("knowrob:heightOfObject", pm);
		
		manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(width,  obj_inst, ros_obj.width));
		manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(depth,  obj_inst, ros_obj.depth));
		manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(height, obj_inst, ros_obj.height));
		
		
		
		// link to parent objects
		OWLNamedIndividual parent = idToInst.get(ros_obj.partOf);
		if(parent!=null) {
			
			// select linking property based on the object type
			if(ros_obj.type.equals("cupboard") ||
			   ros_obj.type.equals("drawer") ||
			   ros_obj.type.equals("oven") ||
			   ros_obj.type.equals("refrigerator") ||
			   ros_obj.type.equals("dishwasher") ||
			   ros_obj.type.equals("table") ||
			   ros_obj.type.equals("countertop") ||
			   ros_obj.type.equals("horizontal_plane") ||
			   ros_obj.type.equals("sink")) {
			
				// top-level object, link to map
				OWLObjectProperty describedInMap = factory.getOWLObjectProperty("knowrob:describedInMap", pm);
				manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(describedInMap, obj_inst, parent));

				
			} else if(ros_obj.type.equals("door")) {
				
				// doors are both part of parent and hinged to it
				OWLObjectProperty properPhysicalParts = factory.getOWLObjectProperty("knowrob:properPhysicalParts", pm);
				manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(properPhysicalParts, parent, obj_inst));
				
				OWLObjectProperty hingedTo = factory.getOWLObjectProperty("knowrob:hingedTo", pm);
				manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(hingedTo, obj_inst, parent));

			} else {
				
				// other objects are only part of their parents
				OWLObjectProperty properPhysicalParts = factory.getOWLObjectProperty("knowrob:properPhysicalParts", pm);
				manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(properPhysicalParts, parent, obj_inst));
				
			}
			
		} else {
			ros.logError("GenerateSemanticMapOWL: Did not find parent object with ID " + ros_obj.partOf);
		}
		
		return obj_inst;
	}
	
	
	
	/**
	 * Create a RotationMatrix3D with all m_ij components set according to the pose of the ros_obj
	 * 
	 * @param ros_obj   SemMapObject input data
	 * @param ontology  Ontology to which the axioms are to be added
	 * @return 			Created instance of a RotationMatrix3D
	 */
	protected OWLNamedIndividual createPoseInst(SemMapObject ros_obj, OWLOntology ontology) { 
	
		// create pose matrix instance
		OWLClass pose_class = factory.getOWLClass("knowrob:RotationMatrix3D", pm);
		OWLNamedIndividual pose_inst = factory.getOWLNamedIndividual(
										      instForClass("ias_map:RotationMatrix3D"), pm);
		manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(pose_class, pose_inst));
		
		System.out.println("\n\n"+ros_obj.id + " " + ros_obj.type);
		
		// set pose properties
		for(int i=0;i<4;i++) {
			for(int j=0;j<4;j++) {
				
				System.out.println("m"+i+j+"="+ros_obj.pose[4*i+j]);
				
				OWLDataProperty prop = factory.getOWLDataProperty("knowrob:m"+i+j, pm);
				manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(prop,  pose_inst, ros_obj.pose[4*i+j]));
			}
		}
		
		return pose_inst;
	}	


	/**
	 * Create an instance of a SemanticMapPerception linking objects to poses and times
	 * @param obj_inst  The object that was detected
	 * @param pose_inst Pose where the object was detected
	 * @param timestamp Time when the object was detected
	 * @param ontology  Ontology to which the axioms are to be added
	 * @return 			Created instance of SemanticMapPerception
	 */
	protected OWLNamedIndividual createPerceptionInst(OWLNamedIndividual obj_inst, OWLNamedIndividual pose_inst, OWLNamedIndividual timestamp, OWLOntology ontology) {
		
		// create perception instance
		OWLClass perc_class = factory.getOWLClass("knowrob:SemanticMapPerception", pm);
		OWLNamedIndividual perc_inst = factory.getOWLNamedIndividual(
										      instForClass("ias_map:SemanticMapPerception"), pm);
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
	 * Create a unique instance identifier from a class string
	 * @param cl 	Class string
	 * @return 		Instance identifier (class string plus index)
	 */
	protected String instForClass(String cl) {
		cl.replaceAll("knorob", "ias_map");
		return cl+(inst_counter++);
	}
	

	
	
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// TODO: copied from RoboEarth -- find a nicer solution, e.g. with a dedicated OWL library package 
//

	/**
	 * Saves an OWL ontology to an output stream in a given ontology format.
	 * @param ontology ontology to be saved
	 * @param stream output stream
	 * @param format desired ontology format (@see OWLOntologyFormat)
	 * @return <tt>true</tt> - if saving was successfully completed<br>
	 * <tt>false</tt> - otherwise
	 */
	public static boolean saveOntologyToStream(OWLOntology ontology, OutputStream stream, OWLOntologyFormat format) {
		
		boolean ok = false;
		
		if (stream != null) {

			try {
				
				// Get hold of the ontology manager
				OWLOntologyManager manager = ontology.getOWLOntologyManager();

				// By default ontologies are saved in the format from which they were loaded.
				// We can get information about the format of an ontology from its manager
				OWLOntologyFormat currFormat = manager.getOntologyFormat(ontology);

				if (currFormat.equals(format)) {
					
					// Save a copy of the ontology to the given stream.
					manager.saveOntology(ontology, stream);
					
				} else {

					// Some ontology formats support prefix names and prefix IRIs. When we save the ontology in
					// the new format we will copy the prefixes over so that we have nicely abbreviated IRIs in
					// the new ontology document
					if (format.isPrefixOWLOntologyFormat() && currFormat.isPrefixOWLOntologyFormat()) {
						((PrefixOWLOntologyFormat)format).copyPrefixesFrom(currFormat.asPrefixOWLOntologyFormat());
					}
					manager.saveOntology(ontology, format, stream);
					
				}

				ok = true;
				
			} catch (Exception e) {
				System.out.println("Could not save ontology: " + e.getMessage());
			}
			
		}
		
		return ok;
	}
	
	
	/**
	 * Saves an OWL ontology to a String object.
	 * @param ontology ontology to be saved
	 * @param format desired ontology format (@see OWLOntologyFormat)
	 * @return String object containing the string representation of the ontology
	 */
	public static String saveOntologytoString(OWLOntology ontology, OWLOntologyFormat format) {
		
		String s = null;
		ByteArrayOutputStream os = new ByteArrayOutputStream(4096);
		
		if (saveOntologyToStream(ontology, os, format)) {
			try {
				s = new String(os.toByteArray(), "UTF-8");
			} catch (UnsupportedEncodingException e) {
				System.out.println("UTF-8 encoding is unsupported: " + e.getMessage());
			}			
		}

		return s;
		
	}
	
	

	/**
	 * Saves an OWL ontology to a file.
	 * @param ontology ontology to be saved
	 * @param file name of target file
	 * @return <tt>true</tt> - if saving was successfully completed<br>
	 * <tt>false</tt> - otherwise
	 */
	public static boolean saveOntologyToFile(OWLOntology ontology, String file) {

		boolean ok = false;

		try {
			OWLOntologyFormat format = ontology.getOWLOntologyManager().getOntologyFormat(ontology);
			ok = saveOntologyToFile(ontology, file, format);			
		} catch (NullPointerException e) {
			System.out.println("Could not save ontology: null pointer argument found\n" + e.getMessage());
		}
		
		return ok; 
		
	}

	
	/**
	 * Saves an OWL ontology to a file in a given ontology format.
	 * @param ontology ontology to be saved
	 * @param file name of target file
	 * @param format desired ontology format (@see OWLOntologyFormat)
	 * @return <tt>true</tt> - if saving was successfully completed<br>
	 * <tt>false</tt> - otherwise
	 */
	public static boolean saveOntologyToFile(OWLOntology ontology, String file, OWLOntologyFormat format) {
		
		boolean ok = false;
		
		try {
			
			// Build File object
			File f = new File(file);
			
			// Get hold of the ontology manager
			OWLOntologyManager manager = ontology.getOWLOntologyManager();

			// By default ontologies are saved in the format from which they were loaded.
			// We can get information about the format of an ontology from its manager
			OWLOntologyFormat currFormat = manager.getOntologyFormat(ontology);

			// The Document IRI, where the file should be saved
			IRI documentIRI = IRI.create(f.toURI());
			
			if (currFormat.equals(format)) {
				
				// Save a local copy of the ontology.
				manager.saveOntology(ontology, documentIRI);
				
			} else {

				// Some ontology formats support prefix names and prefix IRIs. When we save the ontology in
				// the new format we will copy the prefixes over so that we have nicely abbreviated IRIs in
				// the new ontology document
				if (format.isPrefixOWLOntologyFormat() && currFormat.isPrefixOWLOntologyFormat()) {
					((PrefixOWLOntologyFormat)format).copyPrefixesFrom(currFormat.asPrefixOWLOntologyFormat());
				}
				manager.saveOntology(ontology, format, documentIRI);
				
			}

			ok = true;
			
		} catch (Exception e) {
			System.out.println("Could not save ontology: " + e.getMessage());
		}
		
		return ok;
	}

	
	
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	
	public static void main(String[] args) {

		new SemanticMapToOWL();
		
	}

	
}

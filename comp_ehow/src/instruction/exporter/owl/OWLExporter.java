package instruction.exporter.owl;

import instruction.configuration.ConfigurationManager;
import instruction.exceptions.InstructionException;
import instruction.importer.PlanImporter;
import instruction.opencyc.OpenCyc20;
import instruction.semanticObjects.Instruction;
import instruction.semanticObjects.ObjectX;
import instruction.semanticObjects.Preposition;
import instruction.semanticObjects.Word;
import instruction.wrapper.LocalFileWrapper;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.StringWriter;
import java.net.UnknownHostException;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;

import edu.stanford.smi.protege.exception.OntologyLoadException;
import edu.stanford.smi.protegex.owl.ProtegeOWL;
import edu.stanford.smi.protegex.owl.model.OWLClass;
import edu.stanford.smi.protegex.owl.model.OWLModel;
import edu.stanford.smi.protegex.owl.model.OWLNamedClass;
import edu.stanford.smi.protegex.owl.model.OWLObjectProperty;
import edu.stanford.smi.protegex.owl.model.OWLSomeValuesFrom;
import edu.stanford.smi.protegex.owl.model.ProtegeInstance;
import edu.stanford.smi.protegex.owl.model.RDFSClass;
import edu.stanford.smi.protegex.owl.writer.rdfxml.rdfwriter.OWLModelWriter;

public class OWLExporter {

	static {
		ConfigurationManager.loadSettings();
	}
	
	public static final String KITCHEN_ONTOLOGY_URI = ConfigurationManager.getPathKnowRob();
	public static final String ABSTRACT_ACTION_CLASS = "knowrob:Action";
	public static final String ABSTRACT_OBJECT_CLASS = "knowrob:SpatialThing-Localized";
	public static final String ABSTRACT_OBJECT_PROPERTY = "knowrob:spatiallyRelated";

	private static Map<String, OWLNamedClass> actionStore = new HashMap<String, OWLNamedClass>();
	private static Map<String, OWLNamedClass> objectStore = new HashMap<String, OWLNamedClass>();
	private static Map<String, OWLObjectProperty> propertyStore = new HashMap<String, OWLObjectProperty>();

	private static int locationCounter = 1;

	private PlanImporter importer;

	public OWLExporter() {
		
		try {
			this.initOWLConverter();
			
		} catch (UnknownHostException e) {
			e.printStackTrace();
		} catch (InstructionException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	
	/**
	 * Look up instructions for the command 'howto' and translate them into a formal OWL representation
	 * 
	 * @param howto Command like 'set a table'
	 * @return String containing a formal OWL representation of the translated instruction
	 */
	public String convertHowtoToOWLOntology(String howto) {

		try {

			// Convert Howto
			String filePath = ConfigurationManager.getPathHowtos() + "/"
					+ howto.replace(" ", "_");

			if (new File(filePath).exists()) {
				LocalFileWrapper wrapper = new LocalFileWrapper();
				wrapper.load(filePath);
				importer.setWrapper(wrapper);
				importer.parseInstructions();

				importer.recognizeAndDisambiguateInstructions();

				String owlContent = convertInstructionsToOWL(importer
						.getInstructions(), importer.getTitle(), howto);

				return owlContent;
			}

		} catch (InstructionException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		} catch (Exception e) {
			e.printStackTrace();
		}

		return null;
	}

	
	
	/**
	 * Look up instructions for the command 'howto' and translate them into a formal OWL representation
	 * 
	 * Save the result to a file and return the file name (required for the Prolog integration)
	 * 
	 * @param howto Command like 'set a table'
	 * @return File name of the temporary file
	 */
	public String convertHowtoToOWLFile(String howto) {

		try {

			// Convert Howto
			String owlContent = convertHowtoToOWLOntology(howto);

			// Create temporary owl file
			String owlFileName = howto.replace(" ", "_") + ".owl";
			File owlFile = new File(new File(".").getAbsolutePath()	+ "/" + owlFileName);

			if (!owlFile.exists())
				owlFile.createNewFile();
			
			System.out.println("Writing OWL file...");
			FileWriter writer = new FileWriter(owlFile);
			writer.write(owlContent);
			writer.close();
			
			return owlFile.getAbsolutePath();


		} catch (IOException e) {
			e.printStackTrace();
		} catch (Exception e) {
			e.printStackTrace();
		}

		return null;
	}
	
	
	
	
	
	/**
	 * Initialize the converter, including initialization of the OpenCyc
	 * interface and parsing the WordNet ontology.
	 * 
	 * @return
	 * @throws InstructionException
	 * @throws Exception
	 * @throws UnknownHostException
	 * @throws IOException
	 */
	private PlanImporter initOWLConverter() throws InstructionException,
			Exception, UnknownHostException, IOException {
		
		readAvailableActions();
		readAvailableObjects();
		readAvailableObjectProperties();

		ConfigurationManager.loadSettings();
		ConfigurationManager
				.setPathDisambiguator("./etc/disambiguator.xml");
		ConfigurationManager.setPathHowtos("./howtos");

		importer = new PlanImporter();
		importer.initialize();

		System.out.println("Initializing Plan-Importer...");
		Map<String, List<String>> mappings = ConfigurationManager
				.getMappings();
		Set<String> synsets = mappings.keySet();
		for (Iterator<String> i = synsets.iterator(); i.hasNext();) {
			String synset = i.next();
			List<String> concepts = mappings.get(synset);
			for (Iterator<String> j = concepts.iterator(); j.hasNext();) {
				OpenCyc20.getInstance().addMapping(synset, j.next());
			}
		}
		importer.getDisambiguator().load(
				ConfigurationManager.getPathDisambiguator());
		System.out.println("Plan-Importer initialized.");
		return importer;
	}

	
	
	
	/**
	 * Convert a list of instructions into OWL
	 * 
	 * @param inst List of Instruction instances to be converted
	 * @param title Title of the howto
	 * @param howtoName Name of the howto
	 * @return OWL representation of the task
	 * @throws InstructionException
	 */
	
	private String convertInstructionsToOWL(List<Instruction> inst,
			Instruction title, String howtoName) throws InstructionException {

		try {
			OWLModel owlModel = ProtegeOWL.createJenaOWLModelFromURI("http://ias.cs.tum.edu/kb/ehow_input.owl");
			
			int actionCounter = 1;

			// Create ComplexTask

			// Create an easy to read task name composed by the howto name
			String[] howtoTokens = howtoName.split(" ");
			String task = "";
			Set<ProtegeInstance> subTasks = new HashSet<ProtegeInstance>();

			for (String t : howtoTokens)
				task += t.substring(0, 1).toUpperCase() + t.substring(1);

			OWLNamedClass complexTask = owlModel.createOWLNamedSubclass(task,
					actionStore.get("Action"));

			// add the howto name as a label so that the plan can be found from Prolog
			complexTask.addLabel(howtoName, null);
			
			for (Iterator<Instruction> i = inst.iterator(); i.hasNext();) {
				Instruction in = i.next();

				String action = in.getAction().getAction().getCycConcepts()
						.get(0);

				OWLNamedClass owlClass = actionStore.get(action);

				if (owlClass != null) {

					// Create action subclass
					OWLNamedClass actionClass = owlModel
							.createOWLNamedSubclass(action + actionCounter++, owlClass);

					// Set the restriction on the "subEvents" property of the
					// complex task
					OWLSomeValuesFrom value = owlModel.createOWLSomeValuesFrom(
							propertyStore.get("subEvents"), actionClass);

					// Set the restriction on the "objectActedOn" property
					List<ObjectX> objects = in.getObjects();
					Set<RDFSClass> objectRestrictions = new HashSet<RDFSClass>();

					objectRestrictions.add(owlClass);

					for (Iterator<ObjectX> i2 = objects.iterator(); i2.hasNext();) {
						
						ObjectX o = i2.next();
						String objectDesignator = o.getName().get(0).getCycConcepts().get(0);

						String mappedName = objectDesignator;
						if (objectStore.get(mappedName) != null) {
							OWLSomeValuesFrom objectActedOn = owlModel.createOWLSomeValuesFrom(propertyStore
																		.get("objectActedOn"), objectStore
																		.get(mappedName));
							objectRestrictions.add(objectActedOn);
						} else
							System.err.println("Unkown Object Class: " + mappedName);
					}

					if (in.getPrepositions().size() > 0) {
						Preposition p = in.getPrepositions().get(0);
						if (p.getObjects().size() > 0) {
							OWLNamedClass locations = getSpatialPostconditions(
									p.getObjects().get(0), owlModel);
							if (locations != null && (p.getPrepositions().contains(new Word( Word.TYPE_PREPOSITION, "from")) ||
									p.getPrepositions().contains(new Word( Word.TYPE_PREPOSITION, "off"))))
								objectRestrictions.add(owlModel
										.createOWLSomeValuesFrom(propertyStore
												.get("fromLocation"), locations));
							else if (locations != null)
								objectRestrictions.add(owlModel
										.createOWLSomeValuesFrom(propertyStore
												.get("toLocation"), locations));
						}
					}

					// Add the restrictions
					actionClass.addEquivalentClass(owlModel
							.createOWLIntersectionClass(objectRestrictions));
					// Add the new action class as subEvent to the ComplexTask
					subTasks.add(value);
				} else
					System.err.println("Unknown Action Class: " + action);
			}

			complexTask.addEquivalentClass(owlModel
					.createOWLIntersectionClass(subTasks));

			StringWriter writer = new StringWriter();

			OWLModelWriter modelWwriter = new OWLModelWriter(owlModel, owlModel
					.getTripleStoreModel().getActiveTripleStore(), writer);

			modelWwriter.write();
			writer.close();

			return writer.toString();

		} catch (Exception e) {
			e.printStackTrace();
			throw new InstructionException("Could not create OWL Ontology: "
					+ e.getMessage());
		}
	}

	

	/**
	 * Read action classes from the KnowRob ontology
	 * 
	 * @throws InstructionException
	 */
	@SuppressWarnings("unchecked")
	private static void readAvailableActions() throws InstructionException {

		try {
			OWLModel owlModel = ProtegeOWL
					.createJenaOWLModelFromURI(KITCHEN_ONTOLOGY_URI);

			OWLNamedClass actionClass = owlModel
					.getOWLNamedClass(ABSTRACT_ACTION_CLASS);

			actionStore.put(actionClass.getLocalName(), actionClass);

			Collection<OWLNamedClass> classes = owlModel
					.getUserDefinedOWLNamedClasses();

			for (Iterator<OWLNamedClass> it = classes.iterator(); it.hasNext();) {
				OWLNamedClass clazz = it.next();
				if (clazz.getSuperclasses(true).contains(actionClass)) {
					actionStore.put(clazz.getLocalName(), clazz);
				}
			}

		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Read object classes from the KnowRob ontology
	 * 
	 * @throws InstructionException
	 */
	@SuppressWarnings("unchecked")
	private static void readAvailableObjects() throws InstructionException {

		try {

			OWLModel owlModel = ProtegeOWL
					.createJenaOWLModelFromURI(KITCHEN_ONTOLOGY_URI);

			OWLClass actionClass = owlModel
					.getOWLNamedClass(ABSTRACT_OBJECT_CLASS);

			Collection<OWLNamedClass> classes = owlModel
					.getUserDefinedOWLNamedClasses();

			for (Iterator<OWLNamedClass> it = classes.iterator(); it.hasNext();) {
				OWLNamedClass clazz = it.next();
				if (clazz.getSuperclasses(true).contains(actionClass)) {

					objectStore.put(clazz.getLocalName(), clazz);
				}
			}


		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Read object properties from the KnowRob ontology
	 * 
	 * @throws InstructionException
	 */
	@SuppressWarnings("unchecked")
	private static void readAvailableObjectProperties()
			throws InstructionException {

		try {

			OWLModel owlModel = ProtegeOWL
					.createJenaOWLModelFromURI(KITCHEN_ONTOLOGY_URI);


			Collection<OWLObjectProperty> properties = owlModel
					.getUserDefinedOWLObjectProperties();

			for (Iterator<OWLObjectProperty> it = properties.iterator(); it
					.hasNext();) {
				OWLObjectProperty prop = it.next();
				propertyStore.put(prop.getLocalName(), prop);
			}

		} catch (OntologyLoadException e) {
			e.printStackTrace();
		}
	}

	
	/**
	 * Return spatial post-conditions for the ObjectX obj
	 * @param obj
	 * @param owlModel
	 * @return
	 */
	public static OWLNamedClass getSpatialPostconditions(ObjectX obj, OWLModel owlModel) {

		List<Preposition> prep = null;

		prep = obj.getPrepositions();

		if (prep.size() > 0) {
			Preposition p = prep.get(0);

			List<ObjectX> objects = p.getObjects();

			if (objects.size() > 0) {
				// go recursively through the objects
				OWLNamedClass toLocationClass = getSpatialPostconditions(
						objects.get(0), owlModel);

				if (toLocationClass != null) {

					// Create a new "Location" subclass
					String sub = "Place" + locationCounter++;
					OWLNamedClass sup = objectStore.get("Place");
					OWLNamedClass locationClass = owlModel.createOWLNamedSubclass(sub, sup);
					Set<RDFSClass> locationRestrictions = new HashSet<RDFSClass>();
					locationRestrictions.add(objectStore.get("Place"));

					String objectPredicate = obj.getName().get(0)
							.getCycConcepts().get(0);

					// map the preposition to one of the "spatiallyRelated"
					// properties
					OWLObjectProperty spatialRelation = null;
					if (objectPredicate.equalsIgnoreCase("FrontSide"))
						spatialRelation = propertyStore
								.get("inFrontOf-Generally");
					
					else if (objectPredicate.equalsIgnoreCase("TopSide"))
						spatialRelation = propertyStore.get("aboveOf");
					
					else if (objectPredicate.equalsIgnoreCase("BottomSide"))
						spatialRelation = propertyStore.get("belowOf");

					else if (objectPredicate.equalsIgnoreCase("LeftSide"))
						spatialRelation = propertyStore.get("toTheLeftOf");

					else if (objectPredicate.equalsIgnoreCase("RightSide"))
						spatialRelation = propertyStore.get("toTheRightOf");

					else if (objectPredicate.equalsIgnoreCase("Side"))
						spatialRelation = propertyStore.get("toTheSideOf");

					else if (objectPredicate
							.equalsIgnoreCase("CenterOfGeographicalRegion"))
						spatialRelation = propertyStore.get("center");

					if (spatialRelation != null)
						locationRestrictions.add(owlModel
								.createOWLSomeValuesFrom(spatialRelation,
										toLocationClass));
					else
						return null;
					locationClass.addEquivalentClass(owlModel
							.createOWLIntersectionClass(locationRestrictions));

					return locationClass;
				} else
					return null;
			} else
				return null;
		} else
			return objectStore
					.get(obj.getName().get(0).getCycConcepts().get(0));

	}

	public static void main(String[] args) {
		OWLExporter owl = new OWLExporter();
		String filePath = owl.convertHowtoToOWLOntology("set a table");
		System.out.println(filePath);
	}
}

package edu.tum.cs.ias.knowrob.prolog;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Vector;

import edu.tum.cs.ias.knowrob.owl.OWLThing;



/**
 * Prolog query wrapper classes for reading and writing information 
 * from/to the belief state in the UNR execution engine.
 * 
 * These classes are to serve as convenience methods to facilitate
 * the access to information in the belief state without having to
 * deal with Prolog statements. 
 * 
 * @author Moritz Tenorth, tenorth@atr.jp
 *
 */
/**
 * @author tenorth
 *
 */
public class PrologQueryUtils {

	
	
	public static void setActionVisHighlight(String action_class) {
		try {
			action_class = PrologInterface.removeSingleQuotes(action_class);
			PrologInterface.executeQuery("planvis_clear_highlight(_), planvis_highlight('"+ action_class +"', _)");
		} catch (Exception e) {}
	}
	
	public static void clearActionVisHighlight() {
		try {
			PrologInterface.executeQuery("planvis_clear_highlight(_)");
		} catch (Exception e) {}
	}

	public static void startActionVis(String recipe_class) {
		recipe_class = PrologInterface.removeSingleQuotes(recipe_class);
		PrologInterface.executeQuery("planvis_create(P), planvis_load('"+ recipe_class +"', _)");
	}
	
	public static void loadActionIntoVis(String recipe_class) {
		recipe_class = PrologInterface.removeSingleQuotes(recipe_class);
		PrologInterface.executeQuery("planvis_load('"+ recipe_class +"', _)");
	}

	public static void parseOwlFile(String filename) {
		PrologInterface.executeQuery("owl_parser:owl_parse('"+ filename +"', false, false, true)");
	}
	
	
	/**
	 * Read the number of instances that exist for one class, e.g. to
	 * check if there are at least N instances
	 * 
	 * @param objClass Identifier of an OWL class such as 'http://ias.cs.tum.edu/kb/knowrob.owl#Cup'
	 * @return Number of instances of objClass that exist in the knowledge base
	 */
	public static int readNumObjectInstances(String objClass) {
		
		objClass = PrologInterface.removeSingleQuotes(objClass);

		HashMap<String, Vector<String>> obj_insts = 
			PrologInterface.executeQuery("owl_individual_of(Inst, '" + objClass + "')");

		if(obj_insts!=null && obj_insts.get("Inst").size()>0) {

			return obj_insts.get("Inst").size();
		}

		return 0;
	}	
	
	
	/**
	 * Select all instances of a class
	 * 
	 * @param owlClass Identifier of an OWL class such as 'http://ias.cs.tum.edu/kb/knowrob.owl#Cup'
	 * @return Vector of OWL identifiers of all object instances 
	 */
	public static Vector<String> readAllInstancesOfClass(String owlClass) {
		
		owlClass = PrologInterface.removeSingleQuotes(owlClass);
		
		HashMap<String, Vector<String>> obj_insts = 
			PrologInterface.executeQuery("owl_individual_of(Inst, '" + owlClass + "')");

		if(obj_insts!=null && obj_insts.get("Inst").size()>0) {

			return obj_insts.get("Inst");
		}
		return null;
	}
	
	
	/**
	 * Read the latest detection of an object/person/... instance
	 * 
	 * @param objInst OWL identifier of an object instance such as 'http://ias.cs.tum.edu/kb/knowrob.owl#Cup123'
	 * @return OWL identifier of the latest detection of objInst
	 */
	public static String readLatestDetectionOfObjectInst(String objInst) {

		objInst = PrologInterface.removeSingleQuotes(objInst);
		
		HashMap<String, Vector<String>> detections = 
			PrologInterface.executeQuery("latest_detection_of_instance('" + objInst + "', LatestDetection)");

		if(detections!=null && detections.get("LatestDetection").size()>0) {
			return detections.get("LatestDetection").get(0);
		}
		return null;
	}

	
	/**
	 * Read the latest detection of an object/person/... class
	 * 
	 * @param objClass Identifier of an OWL class such as 'http://ias.cs.tum.edu/kb/knowrob.owl#Cup'
	 * @return OWL identifier of the latest detection of any instance of objClass
	 */
	public static String readLatestDetectionOfObjectClass(String objClass) {

		objClass = PrologInterface.removeSingleQuotes(objClass);
		
		HashMap<String, Vector<String>> detections = 
			PrologInterface.executeQuery("latest_detection_of_type('" + objClass + "', LatestDetection)");

		if(detections!=null && detections.get("LatestDetection").size()>0) {
			return detections.get("LatestDetection").get(0);
		}
		return null;
	}
	
	
	/**
	 * Read all information for an instance (owl_has)
	 * 
	 * @param owlInst Identifier of an OWL instance such as 'http://ias.cs.tum.edu/kb/knowrob.owl#Cup123'
	 * @return HashMap of the form <String, Vector<String>> containing the OWL properties (key "P") and their values (key "O")
	 */
	public static HashMap<String, Vector<String>> readInformationForInstance(String owlInst) {

		owlInst = PrologInterface.removeSingleQuotes(owlInst);
		
		HashMap<String, Vector<String>> values = 
			PrologInterface.executeQuery("owl_has('" + owlInst + "', P, O)");
		
		return values;
	}
	
	
	/**
	 * Read all information for a class (class_properties)
	 * 
	 * @param owlClass Identifier of an OWL class such as 'http://ias.cs.tum.edu/kb/knowrob.owl#Cup'
	 * @return HashMap of the form <String, Vector<String>> containing the OWL properties (key "P") and their values (key "O")
	 */
	public static HashMap<String, Vector<String>> readInformationForClass(String owlClass) {

		owlClass = PrologInterface.removeSingleQuotes(owlClass);
		
		HashMap<String, Vector<String>> values = 
			PrologInterface.executeQuery("class_properties('" + owlClass + "', P, O)");
		
		return values;
	}
	
	public static Vector<String> readTypesOfInstance(String owlInst) {

		owlInst = PrologInterface.removeSingleQuotes(owlInst);
		
		HashMap<String, Vector<String>> types = 
			PrologInterface.executeQuery("owl_has('" + owlInst + "', rdf:type, T)");
		
		if(types!=null && types.get("T").size()>0) {
			return types.get("T");
		}
		return new Vector<String>();
	}


	/**
	 * Read the main action class for the command to be performed (using the forCommand specification)
	 * 
	 * @param command The command the recipe is to execute, e.g. 'serve a drink'
	 * @return The OWL class describing the recipe, e.g. roboearth:'ServeADrink'
	 * @throws IOException 
	 */
	public static String readRecipeClass(String command) {

		HashMap<String, Vector<String>> recipe_classes = 
			PrologInterface.executeQuery("rdf(RecipeClass, rdfs:label, literal(like('*"+command+"*'), _))");

		if(recipe_classes.get("RecipeClass").size()==0) {
			return null;
		} 
		return recipe_classes.get("RecipeClass").firstElement();
	}

	
	
	/**
	 * Read prior actions performed in the task
	 * 
	 * @param currentActInst OWL identifier of an action instance such as 'http://ias.cs.tum.edu/kb/knowrob.owl#Pick123'
	 * @return Vector of OWL identifiers of the previous actions in the current task
	 */
	public static Vector<String> readPrevActionInTask(String currentActInst) {

		currentActInst = PrologInterface.removeSingleQuotes(currentActInst);
		
		HashMap<String, Vector<String>> values =
			PrologInterface.executeQuery("rdf_reachable(Prev, 'http://ias.cs.tum.edu/kb/knowrob.owl#nextAction', '" + currentActInst + "')");
		
		if(values!=null && values.get("Prev").size()>0) {
			return values.get("Prev");
		}
		return null;
	}
	
	
	/**
	 * Read output 'prop' of an earlier action 'actionClass' performed in the task
	 * 
	 * @param currentActInst OWL identifier of an action instance such as 'http://ias.cs.tum.edu/kb/knowrob.owl#Pick123'
	 * @param prevActionClass Type of the previous action whose output is to be read
	 * @param prop Property of that action whose value is to be read
	 * @return Value of prop for an instance of actionClass
	 */
	public static String readOutputOfPrevAction(String currentActInst, String prevActionClass, String prop) {

		prevActionClass = PrologInterface.removeSingleQuotes(prevActionClass);
		prop = OWLThing.getShortNameOfIRI(PrologInterface.removeSingleQuotes(prop));
		Vector<String> prev_acts = readPrevActionInTask(currentActInst);
		prevActionClass = OWLThing.getOWLThingByShortname(prevActionClass).getIRI();
		
		for(String prev_act : prev_acts) {
			
			HashMap<String, Vector<String>> values =  // TODO: check that that action is in the same task!
				PrologInterface.executeQuery(
						"owl_individual_of('" + PrologInterface.removeSingleQuotes(prev_act) + "', '" + prevActionClass + "'), " +
						"rdf_has('" + PrologInterface.removeSingleQuotes(prev_act) + "', 'http://ias.cs.tum.edu/kb/knowrob.owl#" + prop + "' , Val)");
//			"rdf_has(prev_act, 'http://ias.cs.tum.edu/kb/knowrob.owl#" + prop + "' , Val)");
			
			if(values!=null && values.get("Val").size()>0) {
				return PrologInterface.stripLiteralType(values.get("Val").firstElement());
			}
		}
		return null;
	}
	
	
	
	
	/**
	 * Read the XY-position of a given point instance
	 * 
	 * TODO: switch from the hacky vector format to RLS or something cleaner and better-defined
	 * 
	 * @param pointInst OWL identifier of a point such as 'http://ias.cs.tum.edu/kb/knowrob.owl#Point123'
	 * @return two-element vector [x,y]
	 */
	public static Vector<Float> readPointXYPose(String pointInst) {

		pointInst = PrologInterface.removeSingleQuotes(pointInst);
		
		if(PrologInterface.executeQuery("owl_individual_of('" + pointInst + "', 'http://ias.cs.tum.edu/kb/knowrob.owl#Point2D')") != null) {

			Vector<Float> res = new Vector<Float>();
			HashMap<String, Vector<String>> obj_pose = 
				PrologInterface.executeQuery("rdf_has('" + pointInst + "', 'http://ias.cs.tum.edu/kb/knowrob.owl#xCoord', X), " +
						"rdf_has('" + pointInst + "', 'http://ias.cs.tum.edu/kb/knowrob.owl#yCoord', Y)");

			if(obj_pose!=null && obj_pose.get("X").size()>0 && obj_pose.get("X").size()>0) {

				res.add(Float.valueOf(PrologInterface.stripLiteralType(obj_pose.get("X").get(0))));   // x coord
				res.add(Float.valueOf(PrologInterface.stripLiteralType(obj_pose.get("Y").get(0))));   // y coord
				return res;
			}
		}
		return null;
	}
	
	
	/**
	 * Read the XY-position plus AZ-orientation of a given point instance
	 * 
	 * TODO: switch from the hacky vector format to RLS or something cleaner and better-defined
	 * 
	 * @param pointInst OWL identifier of a point such as 'http://ias.cs.tum.edu/kb/knowrob.owl#Point123'
	 * @return three-element vector [x,y,az]
	 */
	public static Vector<Float> readPointXYAZPose(String pointInst) {

		pointInst = PrologInterface.removeSingleQuotes(pointInst);
		
		if(PrologInterface.executeQuery("owl_individual_of('" + pointInst + "', 'http://ias.cs.tum.edu/kb/knowrob.owl#Point2D')") != null) {

			Vector<Float> res = new Vector<Float>();
			HashMap<String, Vector<String>> obj_pose = 
				PrologInterface.executeQuery("rdf_has('" + pointInst + "', 'http://ias.cs.tum.edu/kb/knowrob.owl#xCoord', X), " +
						"rdf_has('" + pointInst + "', 'http://ias.cs.tum.edu/kb/knowrob.owl#yCoord', Y)," +
						"rdf_has('" + pointInst + "', 'http://ias.cs.tum.edu/kb/knowrob.owl#orientationAZ', AZ)");

			if(obj_pose!=null && obj_pose.get("X").size()>0 && obj_pose.get("X").size()>0 && obj_pose.get("AZ").size()>0) {

				res.add(Float.valueOf(PrologInterface.stripLiteralType(obj_pose.get("X").get(0))));   // x coord
				res.add(Float.valueOf(PrologInterface.stripLiteralType(obj_pose.get("Y").get(0))));   // y coord
				res.add(Float.valueOf(PrologInterface.stripLiteralType(obj_pose.get("AZ").get(0))));  // orientation
				return res;
			}
		}
		return null;
	}
	

	/**
	 * Read the 6D pose matrix of a given object instance
	 * 
	 * 'Object' is thereby defined in a very wide sense (knowrob:'EnduringThing-Localized')
	 *  and also includes people and robots.
	 * 
	 * TODO: switch from the hacky vector format to RLS or something cleaner and better-defined
	 * 
	 * @param objInst OWL identifier of an object instance such as 'http://ias.cs.tum.edu/kb/knowrob.owl#Cup123'
	 * @return 16-element vector with the row-based pose matrix [m00,m01,m02,m03,m10,...]
	 */
	public static Vector<Float> readObjectPose(String objInst) {

		objInst = PrologInterface.removeSingleQuotes(objInst);
		
		if(PrologInterface.executeQuery("owl_individual_of('" + objInst + "', 'http://ias.cs.tum.edu/kb/knowrob.owl#EnduringThing-Localized')") != null) {

			Vector<Float> res = new Vector<Float>();
			HashMap<String, Vector<String>> obj_pose = 
				PrologInterface.executeQuery("current_object_pose('" + objInst + "', Pose)");

			if(obj_pose.get("Pose").size()>0) {

				ArrayList<String[]> pose = PrologInterface.dottedPairsToArrayList(obj_pose.get("Pose").get(0));

				for(String[] elem : pose) {
					res.add(Float.valueOf(elem[0]));
				}
				return res;
			}
		}
		return null;
	}


	/**
	 * Read the 6D pose matrix of the first instance of the class objClass that can be found.
	 * 
	 * 'Object' is thereby defined in a very wide sense (knowrob:'EnduringThing-Localized')
	 *  and also includes people and robots.
	 * 
	 * TODO: switch from the hacky vector format to RLS or something cleaner and better-defined
	 * 
	 * @param objClass OWL identifier of an object class such as 'http://ias.cs.tum.edu/kb/knowrob.owl#Cup'
	 * @return 16-element vector with the row-based pose matrix [m00,m01,m02,m03,m10,...]
	 */
	public static Vector<Float> readObjectClassPose(String objClass) {

		objClass = PrologInterface.removeSingleQuotes(objClass);
		
		if(PrologInterface.executeQuery("owl_subclass_of('" + objClass + "', 'http://ias.cs.tum.edu/kb/knowrob.owl#EnduringThing-Localized')") != null) {

			Vector<Float> res = new Vector<Float>();
			
			// read pose of first instance and set as target position
			HashMap<String, Vector<String>> obj_inst = 
				PrologInterface.executeQuery("owl_individual_of(Ind, '" + objClass + "')");

			if(obj_inst!=null && obj_inst.get("Ind")!=null && obj_inst.get("Ind").size()>0) {

				HashMap<String, Vector<String>> obj_pose = 
					PrologInterface.executeQuery("current_object_pose(" + obj_inst.get("Ind").get(0) + ", Pose)");

				if(obj_pose.get("Pose").size()>0) {

					ArrayList<String[]> pose = PrologInterface.dottedPairsToArrayList(obj_pose.get("Pose").get(0));

					for(String[] elem : pose) {
						res.add(Float.valueOf(elem[0]));
					}
					return res;

				}
			}
		}
		return null;
	}
	
	
	/**
	 * Checks if an OWL class exists in KnowRob (by checking if it is derived from another class 
	 * using a subClassOf definition)
	 * 
	 * @param owlClass Identifier of an OWL class such as 'http://ias.cs.tum.edu/kb/knowrob.owl#Cup'
	 * @return true if class exists
	 */
	public static boolean owlClassExists(String owlClass) {

		owlClass = PrologInterface.removeSingleQuotes(owlClass);
		
		HashMap<String, Vector<String>> values = 
			PrologInterface.executeQuery("rdfs_subclass_of('" + owlClass + "', Super), \\+ rdf_equal('" + owlClass + "',Super)");
		
		if(values!=null && values.get("Super")!=null && values.get("Super").size()>0) {
			
			// there is a superclass different from the class itself -> class exists
			return true;
		}
		return false;
	}
	
	
	/**
	 * Create an instance of the current action, assert startTime
	 * 
	 * @param currentActClass OWL identifier of the current action class such as 'http://ias.cs.tum.edu/kb/knowrob.owl#PickingUpAnObject'
	 */
	public static String createActionInst(String currentActClass) {

		currentActClass = PrologInterface.removeSingleQuotes(currentActClass);
		
		String inst = createOWLInst(currentActClass);
		
		HashMap<String, Vector<String>> st_time = 
			PrologInterface.executeQuery("get_timepoint(NOW), " +
										 "rdf_assert(" + inst + ", 'http://ias.cs.tum.edu/kb/knowrob.owl#startTime', NOW)");

		if(st_time !=null && st_time.get("NOW").size() > 0) {
			return inst;
		}
		return null;
	}
	
	
	/**
	 * Sets the endTime of an action to the current time point
	 *  
	 * @param actionInst Action instance whose end time is to be set
	 */
	public static void assertActionEndTime(String actionInst) {

		actionInst = PrologInterface.removeSingleQuotes(actionInst);
		
		PrologInterface.executeQuery("get_timepoint(NOW), " +
				"rdf_assert('" + actionInst + "', 'http://ias.cs.tum.edu/kb/knowrob.owl#endTime', NOW)");
	}
	
	
	/**
	 * Links action to the higher-level task instance and to the previous action in the task
	 *  
	 * @param actionInst Action instance to be integrated
	 * @param recipeinst Higher-level task that actionInst is part of 
	 */
	public static void linkActionInTaskContext(String actionInst, String recipeInst) {

		actionInst = PrologInterface.removeSingleQuotes(actionInst);
		recipeInst = PrologInterface.removeSingleQuotes(recipeInst);
		
		
		// find subAction of recipeInst that does not yet have a nextAction
		// (which should be the end of the sequence)
		
		HashMap<String, Vector<String>> prev_action = PrologInterface.executeQuery(
				"rdf_has('" + recipeInst + "', 'http://ias.cs.tum.edu/kb/knowrob.owl#subAction', SUB)," +
				" not(rdf_has(SUB, 'http://ias.cs.tum.edu/kb/knowrob.owl#nextAction', _))");

		// if there is any, assert the current action as nextAction (now being the end of the sequence)
		if(prev_action !=null && prev_action.get("SUB").size() > 0) {
			assertObjectPropertyForInst(prev_action.get("SUB").get(0), "'http://ias.cs.tum.edu/kb/knowrob.owl#nextAction'", actionInst);
		}

		// set actionInst as subAction of recipeInst
		assertObjectPropertyForInst(recipeInst, "'http://ias.cs.tum.edu/kb/knowrob.owl#subAction'", actionInst);
	}


	/**
	 * Assert a set of properties for an instance (given as HashMap propName--Vector<values>)
	 * 
	 * @param instOWL identifier of an instance such as 'http://ias.cs.tum.edu/kb/knowrob.owl#Cup123'
	 * @param propVal HashMap<String, Vector<String> of the form propName --> Vector(propValues)
	 */
	public static void assertPropertiesForInstance(String inst, HashMap<String, Vector<String>> propVal) {

		inst = PrologInterface.removeSingleQuotes(inst);
		
		for(String prop: propVal.keySet()) {
			prop = PrologInterface.removeSingleQuotes(prop);
			for(String val : propVal.get(prop)) {
				val = PrologInterface.removeSingleQuotes(val);
				PrologInterface.executeQuery("rdf_assert('" + inst + "', '"+prop+"', '" + val + "')");
			}
		}
	}	

	
	/**
	 * Assert a single property for an instance
	 * 
	 * @param inst OWL identifier of an instance such as 'http://ias.cs.tum.edu/kb/knowrob.owl#Cup123'
	 * @param prop OWL identifier of a property such as 'http://ias.cs.tum.edu/kb/knowrob.owl#on-Physical'
	 * @param val OWL identifier of an instance such as 'http://ias.cs.tum.edu/kb/knowrob.owl#Cup123'
	 */
	public static void assertObjectPropertyForInst(String inst, String prop, String val) {

		inst = PrologInterface.removeSingleQuotes(inst);
		prop = PrologInterface.removeSingleQuotes(prop);
		val = PrologInterface.removeSingleQuotes(val);
		
		PrologInterface.executeQuery("rdf_assert('" + inst + "', '"+prop+"', '" + val + "')");
	}
	

	
	/**
	 * Assert a single property for an instance
	 * 
	 * @param inst OWL identifier of an instance such as 'http://ias.cs.tum.edu/kb/knowrob.owl#Cup123'
	 * @param prop OWL identifier of a property such as 'http://ias.cs.tum.edu/kb/knowrob.owl#on-Physical'
	 * @param val String value to be asserted
	 * @param type Type of val (usually an XSD type such as 'http://www.w3.org/2001/XMLSchema#string'
	 */
	public static void assertDataPropertyForInst(String inst, String prop, String val, String type) {

		inst = PrologInterface.removeSingleQuotes(inst);
		prop = PrologInterface.removeSingleQuotes(prop);
		val = PrologInterface.removeSingleQuotes(val);
		type = PrologInterface.removeSingleQuotes(type);
		
		PrologInterface.executeQuery("rdf_assert('" + inst + "', '"+prop+"', literal(type('" + type + "', '" + val + "')))");
	}
	
	
	
	/**
	 * Creates an OWL class restriction in Prolog
	 * 
	 * @param owlClass  Object class to which the restriction is to be added
	 * @param prop 	     Property to be restricted
	 * @param value     Value definition to be set
	 * @param restrType Type of restriction (owl:someValuesFrom, owl:allValuesFrom or owl:hasValue)
	 * @param sourceRef String to identify the source module of the created triples
	 * @return          Identifier of the generated restriction
	 */
	public static String createRestriction(String owlClass, String prop, String value, String restrType, String sourceRef) {

		owlClass  = PrologInterface.removeSingleQuotes(owlClass);
		prop      = PrologInterface.removeSingleQuotes(prop);
		value     = PrologInterface.removeSingleQuotes(value);
		restrType = PrologInterface.removeSingleQuotes(restrType);
		sourceRef = PrologInterface.removeSingleQuotes(sourceRef);
		
		HashMap<String, Vector<String>> restr = 
			PrologInterface.executeQuery("create_restr('"+owlClass+"', '" + prop + "', '" + value + "', '" + restrType + "', '" + sourceRef + "', Restr)");

		if(restr !=null && restr.get("Restr")!=null && restr.get("Restr").size() > 0) {
			return restr.get("Restr").firstElement();
		}
		return null;
	}
	
	
	/**
	 * Add a subClassOf axiom for a class
	 * 
	 * @param subClass OWL identifier of the subclass such as 'http://ias.cs.tum.edu/kb/knowrob.owl#Cup'
	 * @param superClass OWL identifier of the superclass such as 'http://ias.cs.tum.edu/kb/knowrob.owl#Container'
	 */
	public static void assertSubClassOf(String subClass, String superClass) {

		subClass = PrologInterface.removeSingleQuotes(subClass);
		superClass = PrologInterface.removeSingleQuotes(superClass);
		
		PrologInterface.executeQuery("rdf_assert('" + subClass + "', rdfs:subClassOf, '" + superClass + "')");
	}
	
	
	/**
	 * Create an instance of an OWL Class
	 * 
	 * @param owlClass The class to be instantiated
	 * @return
	 */
	public static String createOWLInst(String owlClass) {

		owlClass = PrologInterface.removeSingleQuotes(owlClass);
		
		HashMap<String, Vector<String>> inst = 
			PrologInterface.executeQuery("rdf_instance_from_class('" + owlClass + "', ObjInst)");

		if(inst !=null && inst.get("ObjInst").size() > 0) {

			return inst.get("ObjInst").get(0);
		}
		return null;
	}
	
	
	/**
	 * Create an instance of a state transition
	 * 
	 * @param owlClass The class to be instantiated
	 * @return
	 */
	public static String createStateTransition(String fromState, String toState, String causedBy) {

		fromState = PrologInterface.removeSingleQuotes(fromState);
		toState   = PrologInterface.removeSingleQuotes(toState);
		
		// check if there is already a transition with the defined from- and to-classes
		HashMap<String, Vector<String>> t_insts = 
			PrologInterface.executeQuery("owl_individual_of(T, 'http://ias.cs.tum.edu/kb/knowrob.owl#IntrinsicStateChangeEvent'), " +
										 "rdf_has(T, 'http://ias.cs.tum.edu/kb/knowrob.owl#fromState', '" + fromState + "'), " +
									 	 "rdf_has(T, 'http://ias.cs.tum.edu/kb/knowrob.owl#toState', '" + toState + "'), " +
									 	 "rdf_has(T, 'http://ias.cs.tum.edu/kb/knowrob.owl#causedBy', '" + causedBy+ "')");

		// return existing instance if there is one with the respective arguments
		if(t_insts !=null && t_insts.get("T").size() > 0) {
			return t_insts.get("T").get(0);
			
		} else {
			
			// create new transition otherwise:
			String t_inst = PrologQueryUtils.createOWLInst("http://ias.cs.tum.edu/kb/knowrob.owl#IntrinsicStateChangeEvent");
			
			assertObjectPropertyForInst(t_inst, "http://ias.cs.tum.edu/kb/knowrob.owl#fromState", fromState);
			assertObjectPropertyForInst(t_inst, "http://ias.cs.tum.edu/kb/knowrob.owl#toState",   toState);
			assertObjectPropertyForInst(t_inst, "http://ias.cs.tum.edu/kb/knowrob.owl#causedBy",  causedBy); // TODO: is this value already in the UNR Actions OWL namespace?
			return t_inst;
		}
	}
	
	
	/**
	 * Clear state transitions asserted for an action (i.e. the transitions between sub-actions)
	 * 
	 * @param owlClass The action whose sub-action transitions are to be cleared 
	 */
	public static void clearActionStateTransitions(String action) {

		action = PrologInterface.removeSingleQuotes(action);
		
		HashMap<String, Vector<String>> t_insts = PrologInterface.
				executeQuery("class_properties('" + action + "', 'http://ias.cs.tum.edu/kb/knowrob.owl#stateTransition', T)");

		if(t_insts !=null && t_insts.get("T").size() > 0) {
			
			for(String t : t_insts.get("T")) {

				// remove restriction on this transition
				PrologInterface.executeQuery("findall(R, rdf_has(R, 'http://www.w3.org/2002/07/owl#hasValue', " + t + "), Rs), " +
						"member(Restr, Rs), rdf_retractall(Restr, _, _), rdf_retractall(_, 'http://www.w3.org/2000/01/rdf-schema#subClassOf', Restr)");

				// remove transition properties and transition itself
				PrologInterface.executeQuery("rdf_retractall(" + t + ", _, _)");

			}
		}
	}
	
	
	/**
	 * Create an instance of an object/person incl. pose (and detection of the pose)
	 * 
	 * @param objClass
	 * @param perceptionMethod 
	 * @return
	 */
	public static String createObjInstWithPose(String objClass, String perceptionMethod, Vector<Float> pose) {

		objClass = PrologInterface.removeSingleQuotes(objClass);
		perceptionMethod = PrologInterface.removeSingleQuotes(perceptionMethod);
		
		String q = "";
		
		if(pose.size()==2) {
			q="create_object_perception('" + objClass + "', [1.0,0.0,0.0," + 
				pose.get(0) + ",0.0,1.0,0.0," + pose.get(1) + ",0.0,0.0,1.0," +
				"0.0,0.0,0.0,0.0,1.0], ['" + perceptionMethod + "'], ObjInst)";
		} else if(pose.size()==16) {
			q="create_object_perception('" + objClass + "', ["+
			pose.get(0)  +","+ pose.get(1)  +","+ pose.get(2)  +","+ pose.get(3)  +","+
			pose.get(4)  +","+ pose.get(5)  +","+ pose.get(6)  +","+ pose.get(7)  +","+
			pose.get(8)  +","+ pose.get(9)  +","+ pose.get(10) +","+ pose.get(11) +","+
			pose.get(12) +","+ pose.get(13) +","+ pose.get(14) +","+ pose.get(15) +
			"], ['" + perceptionMethod + "'], ObjInst)";			
		}

		HashMap<String, Vector<String>> obj_id = PrologInterface.executeQuery(q);
		if(obj_id != null && obj_id.get("ObjInst").size() > 0) {

			return obj_id.get("ObjInst").get(0);
		}
		return null;
	}
	
	
	
	/**
	 * Set the pose of an instance by adding a detection to it
	 * 
	 * @param objInst
	 * @param perceptionMethod
	 * @param pose
	 */
	public static void setObjPose(String objInst, String perceptionMethod, Vector<Float> pose) {

		objInst = PrologInterface.removeSingleQuotes(objInst);
		perceptionMethod = PrologInterface.removeSingleQuotes(perceptionMethod);
		String q = "";
		
		if(pose.size()==2) {
			q="create_perception_instance(['"+perceptionMethod+"'], Perception),"+
		    "set_object_perception('"+objInst+"', Perception),"+
		    "set_perception_pose(Perception, [1.0,0.0,0.0," + pose.get(0) + 
		    ",0.0,1.0,0.0," + pose.get(1) + ",0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0]).";
			
		} else if(pose.size()==16) {
			q="create_perception_instance(['"+perceptionMethod+"'], Perception),"+
		    "set_object_perception('"+objInst+"', Perception),"+
		    "set_perception_pose(Perception, ["+
			pose.get(0)  +","+ pose.get(1)  +","+ pose.get(2)  +","+ pose.get(3)  +","+
			pose.get(4)  +","+ pose.get(5)  +","+ pose.get(6)  +","+ pose.get(7)  +","+
			pose.get(8)  +","+ pose.get(9)  +","+ pose.get(10) +","+ pose.get(11) +","+
			pose.get(12) +","+ pose.get(13) +","+ pose.get(14) +","+ pose.get(15) +
			"])";			
		}

		PrologInterface.executeQuery(q);
	}
	
	
	/**
	 * Check if 'objActOn' fulfills the condition 'condition' (if objActOn is an
	 * individual) or if any of its individuals fulfills the condition (it objActOn
	 * is a class)
	 * 
	 * @param condition OWL Class specification (usually based on restrictions)
	 * @param objActOn OWL Class or individual
	 * @return true if objActOn is an individual of condition
	 */
	public static boolean evaluateCondition(String condition, String objActOn) {

		condition = PrologInterface.removeSingleQuotes(condition);
		objActOn = PrologInterface.removeSingleQuotes(objActOn);
		
		// covers the cases that 
		// - objectActedOn is class: check if there is an instance of that class that fulfills the condition
		// - objectActedOn is instance: check if that instance fulfills the condition
		
		HashMap<String, Vector<String>> cond_check = 
			PrologInterface.executeQuery("owl_individual_of('" + objActOn+ "', '" + condition + "');" +
					"owl_individual_of(Inst, '" + objActOn + "'), owl_individual_of(Inst, '" + condition + "')");	

		if(cond_check !=null) {

			return true;
		}
		return false;
	}

	
	/**
	 * Convenience method to read the (first) instance of a SemanticEnvironmentMap
	 * @param streetName 
	 * @param streetNumber 
	 * @param floorNumber 
	 * @param roomNumber 
	 * 
	 * @return IRI of the first instance of a SemanticEnvironmentMap found in the knowledge base
	 */
	public static String getSemanticMapInstance(String roomNumber, String floorNumber, String streetNumber, String streetName) {

		String q = 	"owl_individual_of(Map, 'http://ias.cs.tum.edu/kb/knowrob.owl#SemanticEnvironmentMap')";
		
		if(roomNumber!=null && !roomNumber.isEmpty()) {
			roomNumber = PrologInterface.removeSingleQuotes(roomNumber);
			q+= ",owl_has(R, 'http://ias.cs.tum.edu/kb/knowrob.owl#describedInMap', Map)," +
			     "owl_has(R, 'http://ias.cs.tum.edu/kb/knowrob.owl#roomNumber', literal(type(_,'"+roomNumber+"')))";
		}
		
		if(floorNumber!=null && !floorNumber.isEmpty()) {
			floorNumber = PrologInterface.removeSingleQuotes(floorNumber);
			q+= ",owl_has(S1, 'http://ias.cs.tum.edu/kb/knowrob.owl#properPhysicalParts', R)," +
				 "rdf_has(R,  'http://ias.cs.tum.edu/kb/knowrob.owl#floorNumber', literal(type(_, '"+floorNumber+"')))";
		}
		
		if(streetNumber!=null && !streetNumber.isEmpty()) {
			streetNumber = PrologInterface.removeSingleQuotes(streetNumber);
			q+= ",owl_has(S1, 'http://ias.cs.tum.edu/kb/knowrob.owl#properPhysicalParts', R)," +
				"rdf_has(R, 'http://ias.cs.tum.edu/kb/knowrob.owl#streetNumber', literal(type(_, '"+streetNumber+"')))";
		}
		
		if(streetName!=null && !streetName.isEmpty()) {
			streetName = PrologInterface.removeSingleQuotes(streetName);
			q+= ",owl_has(S1, 'http://ias.cs.tum.edu/kb/knowrob.owl#properPhysicalParts', R)," +
				"rdf_has(S2, rdfs:label, literal(like('*"+streetName+"*'), _))";
		}
		
		HashMap<String, Vector<String>> map_insts = PrologInterface.executeQuery(q);
		
		if(map_insts!=null && map_insts.get("Map").size()>0) {
			return map_insts.get("Map").firstElement();
		}
		return null;
	}

	
	public static void writeActionToOWLFile(String actionClass, String file) {
		PrologInterface.executeQuery("export_action('" + actionClass + "', '" + file + "')");
	}
	
	
	public static void deleteObjectInstance(String iri) {
		iri = PrologInterface.removeSingleQuotes(iri);
		PrologInterface.executeQuery("delete_object_information('" + iri + "')");
	}

	public static void deleteObjectInstanceWithChildren(String iri) {
		iri = PrologInterface.removeSingleQuotes(iri);
		PrologInterface.executeQuery("delete_object_information_recursive('" + iri + "')");
	}
}

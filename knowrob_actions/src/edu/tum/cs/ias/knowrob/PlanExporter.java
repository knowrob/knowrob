package edu.tum.cs.ias.knowrob;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Vector;

import com.google.common.base.CaseFormat;
import com.google.common.base.Joiner;

import net.sf.json.*;
import edu.tum.cs.ias.knowrob.prolog.PrologInterface;
import edu.tum.cs.ias.knowrob.utils.ros.RosUtilities;

public class PlanExporter {


    public static final String MAPPING_CONFIG_FILE = "etc/knowrob2cpl.json";

    /**
     * List of all action instances of the plan in the correct order, e.g.
     * "action1", "action2", ...
     */
    private ArrayList<String> plan;

    /**
     * mapping from KnowRob concepts to CPL identifiers
     */
    private JSONObject knowrobToCPL;
    private HashMap<String, String> designators;
    private Vector<String> designator_order;
    private int inst_counter = 100;

    public PlanExporter() throws IOException {

        designators = new HashMap<String, String>();
        designator_order = new Vector<String>();
        knowrobToCPL = readCplMapping(RosUtilities.rospackFind("knowrob_actions") + "/" +MAPPING_CONFIG_FILE);
        plan = new ArrayList<String>();
    }

    public String exportPlanToCPL(String plan_name) {
    	
        plan_name = PrologInterface.addSingleQuotes(plan_name);
		String res = "";
		
		HashMap<String, Vector<String>> plan_steps = 
			PrologInterface.executeQuery("plan_subevents("+ plan_name +", Steps)");

		
		for(String dottedpairs : (plan_steps.get("Steps"))) {
		    

		    for(String[] actions: PrologInterface.dottedPairsToArrayList(dottedpairs)) {
		        
		        for(String action : new ArrayList<String>(Arrays.asList(actions))) {
		    
		            HashMap<String, Vector<String>> params = 
		                PrologInterface.executeQuery("class_properties("+action+", Prop, Val)");

		            String obj_desig="", loc_desig="", act_desig="", device="", bodypart="";
		            String cplAction = knowrobToCpl(action);

		            for(int i=0;i<params.get("Prop").size();i++) {

		                String prop = PrologInterface.removeSingleQuotes(params.get("Prop").get(i));
		                String val  = PrologInterface.removeSingleQuotes(params.get("Val").get(i));


		                if(prop.endsWith("objectActedOn")) {

		                    obj_desig = objectDesignatorFromOWLclass(val);


		                } else if(prop.endsWith("toLocation") || prop.endsWith("fromLocation")) {

	                		// location specification
		                	loc_desig = locationDesignatorFromOWLclass(val);
		                    
		                	// trajectory specification for reaching motions
	                		act_desig = actionDesignatorFromOWLclass(action, prop, loc_desig);
	                		
		                } else if(prop.endsWith("deviceUsed")) {
		                	device = val;
		                	
		                } else if(prop.endsWith("bodyPartsUsed")) {
		                	bodypart = val;
		                }

		            }
		            
		            // create action goal
		            String action_spec = "";
		            if(cplAction.endsWith("object-in-hand")) {
		            	
		            	if(bodypart!=null && bodypart.contains("left")) {
		            		action_spec = "(achieve `(object-in-hand ,"+ obj_desig +" :left))";
		            	} else {
		            		action_spec = "(achieve `(object-in-hand ,"+ obj_desig +" :right))";
		            	}

		            	
		            } else if(cplAction.endsWith("object-placed-at")) {
		            	
		            	if(bodypart!=null && bodypart.contains("left")) {
		            		action_spec = "(achieve `(object-placed-at ,"+ obj_desig +" ,"+ loc_desig +" :left))";
		            	} else {
		            		action_spec = "(achieve `(object-placed-at ,"+ obj_desig +" ,"+ loc_desig +" :right))";
		            	}

		            	
		            } else if(cplAction.endsWith("arm-parked")) {
		                action_spec = "(achieve `(arm-parked ,"+device+"))";

		                
		            } else if(cplAction.endsWith("open-gripper")) {

		            	String act_desig2 = "unhand-action"+(inst_counter++);
	            		String act_desig2_content = "";
		            	if(bodypart!=null && bodypart.contains("left")) {
		            		act_desig2_content = "("+act_desig2+" (action `((to open-gripper) (side :left))))\n";
		            	} else {
		            		act_desig2_content = "("+act_desig2+" (action `((to open-gripper) (side :right))))\n";
		            	}
	            		
	            		action_spec = "(achieve `(arms-at ," + act_desig2 + "))";
	            		
	            		if(!designator_order.contains(act_desig2))
	            			designator_order.add(act_desig2);
	            		designators.put(act_desig2, act_desig2_content);
	            	      
		            } else if(cplAction.endsWith("close-gripper")) {

		            	String act_desig2 = "unhand-action"+(inst_counter++);
	            		String act_desig2_content = "";
		            	if(bodypart!=null && bodypart.contains("left")) {
		            		act_desig2_content = "("+act_desig2+" (action `((to close-gripper) (side :left))))\n";
		            	} else {
		            		act_desig2_content = "("+act_desig2+" (action `((to close-gripper) (side :right))))\n";
		            	}
	            		
	            		action_spec = "(achieve `(arms-at ," + act_desig2 + "))";
	            		
	            		if(!designator_order.contains(act_desig2))
	            			designator_order.add(act_desig2);
	            		designators.put(act_desig2, act_desig2_content);

	            		
		            } else if(cplAction.endsWith("arms-at")) {
		                action_spec = "(achieve `(arms-at ,"+ act_desig +"))";

	            		
		            } else if(cplAction.endsWith("looking-at")) {
		                action_spec = "(achieve `(looking-at ,"+ loc_desig +"))";

		                
		            } else if(cplAction.endsWith("perceive-all")) {
		                action_spec = "(perceive-all ,"+ obj_desig +")";

		                
		            } else if(cplAction.endsWith("perceive")) {
		            	action_spec = "(perceive ,"+ obj_desig +")"; 

		            	
		            }  else if(cplAction.endsWith("loc")) {
		            	action_spec = "(at-location ("+ loc_desig +"))"; // TODO: extend the closing parenthesis around all actions to be performed here 
		            }

		            plan.add(action_spec);

		        }

		    }
		}
        

		ArrayList<String> orderedDesigValues = new ArrayList<String>();
		
		for(String key : designator_order)
			orderedDesigValues.add(designators.get(key));


		// export to string
		res += "(def-top-level-plan " + lispify(plan_name) + " () \n";
		res += "(with-designators (\n      ";
		res += Joiner.on("\n      ").join(orderedDesigValues);
		
		res += ")\n\n";
		
		res += Joiner.on("\n").join(plan);
		
		res += "))";
		
		return res;
	}

    
    
    
    /**
     * Create an object designator based on an OWL class specification
     * 
     * @param objdef OWL class specification of an object
     * @return       ID of the generated designator
     */
    private String objectDesignatorFromOWLclass(String objdef) {

        // create the appropriate object designators
        HashMap<String, Vector<String>> types = PrologInterface
                .executeQuery("rdf_has(" + PrologInterface.addSingleQuotes(objdef) + ", rdf:type, T)");

        String obj_type = "";
        String obj_inst = "";
        String obj_desig = "";
        
        // first case: objdef is an instance
        if (types.get("T") != null && types.get("T").size() > 0) {
        	
            obj_type = knowrobToCpl(types.get("T").firstElement());
            obj_inst = knowrobToCpl(objdef);
            obj_desig = "(" + lispify(obj_inst) + " (object `((name " + knowrobToCpl(obj_inst) + ") (type "+lispify(obj_type)+"))))";

        } else { // second case: objdef is a class
            
            obj_type = knowrobToCpl(objdef);
            obj_inst = knowrobToCpl(instanceFromClass(obj_type));
            obj_desig = "(" + lispify(obj_inst) + " (object  `((type " + knowrobToCpl(obj_type) + "))))";
        }

        if(!designator_order.contains(obj_inst))
        	designator_order.add(obj_inst);
        
        designators.put(obj_inst, obj_desig);
        
        return obj_inst;
    }

    
    
    
    
    private String locationDesignatorForObject(String objClass) {
    	
    	// first create object designator:
    	String obj_desig = objectDesignatorFromOWLclass(objClass);
//    	
//    	String loc_desig="";
//    	loc_desig = "(location-" + obj_desig + " (location `((of ," + obj_desig + "))))";
//    	
//    	if(!designator_order.contains(obj_desig))
//    		designator_order.add(obj_desig);
//    	designators.put("location-" + obj_desig, loc_desig);
//    	return "location-" + obj_desig;
    	
    	return obj_desig;
    }
    
    
    
    
    
    /**
     * Create a location designator based on an OWL class specification
     * 
     * @param loc
     *            OWL class specification of a location
     * @return ID of the generated designator
     */
    private String locationDesignatorFromOWLclass(String loc) {
        
        HashMap<String, Vector<String>> params = PrologInterface
                .executeQuery("class_properties(" + PrologInterface.addSingleQuotes(loc) + ", Prop, Val); " +
                		      "owl_has("+PrologInterface.addSingleQuotes(loc)+", Prop, Val)");
        
        
        loc = knowrobToCpl(loc);
        
        // recursively build the location designator
        String loc_desig = "(" + loc + " ";
        loc_desig += "(location `(";
        
        
        if(params!=null && params.get("Prop")!=null) {
        
            for (int i = 0; i < params.get("Prop").size(); i++) {
    
                String prop = PrologInterface.removeSingleQuotes(params.get("Prop").get(i));
                String val = params.get("Val").get(i);
    
                // check if referring to object instance or another location
                HashMap<String, Vector<String>> objtype = PrologInterface
                        .executeQuery("rdf_has("+PrologInterface.addSingleQuotes(val)+", rdf:type, Type)");
                
                boolean isObject = true;
                if(objtype!=null && objtype.get("Type")!=null) {
                	for (int k = 0; k < objtype.get("Type").size(); k++) {
                		
                		String type = PrologInterface.removeSingleQuotes(objtype.get("Type").get(k));
                		if( (type.endsWith("Point3D")) || (type.endsWith("Point2D"))) {
                			isObject = false;
                		}
                	}
                }
                
                String desig = "";                               
                if ((prop.endsWith("in-ContGeneric")
                        || prop.endsWith("in-UnderspecifiedContainer")
                        || prop.endsWith("into-UnderspecifiedContainer") 
                        || prop.endsWith("from-UnderspecifiedLocation"))) {
                	
                    if(isObject) {
                    	desig = knowrobToCpl(locationDesignatorForObject(val));
                    } else {
                    	desig = knowrobToCpl(locationDesignatorFromOWLclass(val));
                    }
                    
                    loc_desig += "(in ," + desig + ")";
                    
                } else if (prop.endsWith("to-UnderspecifiedLocation")
                        || prop.endsWith("on-Physical")
                        || prop.endsWith("aboveOf")
                        || prop.endsWith("inCenterOf")) {
                    if(isObject) {
                    	desig = knowrobToCpl(locationDesignatorForObject(val));
                    } else {
                    	desig = knowrobToCpl(locationDesignatorFromOWLclass(val));
                    }
                    
                    loc_desig += "(on ," + desig + ")";
    
                } else if (prop.endsWith("inReachOf") ||
                        prop.endsWith("inFrontOf-Generally")) {
                    if(isObject) {
                    	desig = knowrobToCpl(locationDesignatorForObject(val));
                    } else {
                    	desig = knowrobToCpl(locationDesignatorFromOWLclass(val));
                    }
                    
                    loc_desig += "(to reach) (side :right) (loc ," + desig + ")";
    
                } else if (prop.endsWith("visibleFrom")) {
                    if(isObject) {
                    	desig = knowrobToCpl(locationDesignatorForObject(val));
                    } else {
                    	desig = knowrobToCpl(locationDesignatorFromOWLclass(val));
                    }
                    
                    loc_desig += "(to see ," + desig + ")";
    
                } else if (prop.endsWith("orientation")) {
                    if(isObject) {
                    	desig = knowrobToCpl(locationDesignatorForObject(val));
                    } else {
                    	desig = knowrobToCpl(locationDesignatorFromOWLclass(val));
                    }
                    
                    loc_desig += "(pose ," + desig + ")";
                }
    
            }
        }
        loc_desig += ")))";

        if(!designator_order.contains(loc))
        	designator_order.add(loc);
        designators.put(loc, loc_desig);
        return loc;
    }

    
    
    private String actionDesignatorFromOWLclass(String action, String prop, String loc_desig) {
    	
    	String cplAction = knowrobToCpl(action);
    	String act_desig = cplAction+(inst_counter++);
    	String act_descr="";
    	act_descr += "("+act_desig+" (action `(";
        
    	// check action type: only Reaching
    	
    	if(cplAction.endsWith("arms-at")) {
    		
        	act_descr += "(to trajectory) (pose ," + loc_desig + ")";

        	// check if side is set
        	HashMap<String, Vector<String>> params = 
                    PrologInterface.executeQuery("class_properties("+action+", Prop, Val)");

    		int idx = params.get("Prop").lastIndexOf(PrologInterface.
    				addSingleQuotes("http://ias.cs.tum.edu/kb/knowrob.owl#bodyPartsUsed"));
    		
            if(idx>-1) {
            	if(params.get("Val").get(idx).contains("right") ||
            			params.get("Val").get(idx).contains("Right")) {
            		act_descr+=" (side :right)";
            	} else if(params.get("Val").get(idx).contains("left") ||
            			params.get("Val").get(idx).contains("Left")) {
            		act_descr+=" (side :left)";	
            	}
            }
            act_descr+=")))";

            if(!designator_order.contains(act_desig))
            	designator_order.add(act_desig);
        	designators.put(act_desig, act_descr);

		}
		return act_desig;		
		
    }
    
    /**
     * Convert KnowRob identifier to CPL
     * 
     * If there is a mapping defined in the knowrobToCPL HashMap, this
     * mapping will be used. Otherwise, the methods returns a 'lispified'
     * string, i.e. a string that is converted from CamelCase to 
     * lower-case-hyphenated 
     * 
     * @param val KnowRob identifier
     * @return Lispified identifier that can be used in CPL
     */
    private String knowrobToCpl(String val) {
        
        if(knowrobToCPL.containsKey(PrologInterface.valueFromIRI(val))) {
            return knowrobToCPL.getString(PrologInterface.valueFromIRI(val));
            
        } else {
            
            // check whether there are definitions for superclasses, take the first one
            HashMap<String, Vector<String>> supercl = PrologInterface.
                    executeQuery("owl_subclass_of(" + PrologInterface.addSingleQuotes(val) + ", Super)");
            
            for(String sup : supercl.get("Super")) {
                if(knowrobToCPL.containsKey(PrologInterface.valueFromIRI(sup))) {
                    return knowrobToCPL.getString(PrologInterface.valueFromIRI(sup));
                }
            }
            return lispify(val);
        }
    }
    
    /**
     * 'Lispify' a string: convert from Java CamelCaseSyntax into
     *  hyphenated-lower-case-syntax.
     *  
     * @param pl String to be lispified
     * @return Resulting string in lowercase, hyphenated style
     */
    private String lispify(String pl) {
        
        // extract value from IRI
        String val = PrologInterface.valueFromIRI(PrologInterface.removeSingleQuotes(pl));
        val = CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_HYPHEN, val);
        val = val.replaceAll("--", "_");
        
        return val;
    }
    
    
    
    /**
     * Read JSON-formatted configuration file to initialize the mapping between
     * KnowRob classes and CPL identifiers
     * 
     * @param configFile
     * @throws java.io.IOException
     */

    public JSONObject readCplMapping(String configFile)
            throws java.io.IOException {

        byte[] buffer = new byte[(int) new File(configFile).length()];
        BufferedInputStream f = null;
        try {
            f = new BufferedInputStream(new FileInputStream(configFile));
            f.read(buffer);
        } finally {
            if (f != null)
                try {
                    f.close();
                } catch (IOException ignored) {
                }
        }

        return (JSONObject) JSONSerializer.toJSON(new String(buffer));
    }

    
    /**
     * Create unique identifier from a class description
     * 
     * @param cl
     *            Class description
     * @return Identifier: class description plus numeric ID
     */
    private String instanceFromClass(String cl) {
        return (cl + (inst_counter++)).toLowerCase();
    }

    
    /**
     * Main function for testing purposes
     * 
     * @param args
     */
    public static void main(String[] args) {
        
        try {
            
            PrologInterface.initJPLProlog("knowrob_actions");
            PlanExporter pl = new PlanExporter();
            String plan = pl.exportPlanToCPL("http://www.roboearth.org/kb/roboearth.owl#ServeADrink");
            System.out.println(plan);
            
        } catch (IOException e) {
            e.printStackTrace();
        }
        
    }
    
}

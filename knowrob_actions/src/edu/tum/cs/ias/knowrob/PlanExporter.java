package edu.tum.cs.ias.knowrob;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Vector;
import org.apache.commons.lang.StringUtils;
import net.sf.json.*;
import edu.tum.cs.ias.knowrob.prolog.PrologInterface;

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
    private int inst_counter = 100;

    public PlanExporter() throws IOException {

        designators = new HashMap<String, String>();
        knowrobToCPL = readCplMapping(MAPPING_CONFIG_FILE);
    }

    public String exportPlanToCPL(String plan_name) {
		
		String res = "";

		HashMap<String, Vector<String>> plan_steps = 
			PrologInterface.executeQuery("plan_subevents("+plan_name+", Steps)");

		
		for(String action : plan_steps.get("Steps")) {

			HashMap<String, Vector<String>> params = 
				PrologInterface.executeQuery("class_properties("+action+", Prop, Val)");
			
			String obj_desig="", loc_desig="";
			
			for(int i=0;i<params.get("Prop").size();i++) {
				
				String prop = params.get("Prop").get(i);
				String val  = params.get("Val").get(i);
				
				
				if(prop.endsWith("objectActedOn")) {
					
				    obj_desig = objectDesignatorFromOWLclass(val);
					
				} else if(prop.endsWith("toLocation") || prop.endsWith("fromLocation")) {
					
					Vector<String> props = params.get("Prop");
					loc_desig = locationDesignatorFromOWLclass(val, 
					        props.get(props.lastIndexOf("objectActedOn"))); 
				}
			}
			
			
			// create action goal
			String rplAction = knowrobToCPL.getString(action);
			
			String action_spec = "";
			if(rplAction.endsWith("object-in-hand")) {
			    action_spec = "(achieve (object-in-hand "+ obj_desig +" ?side))";
			    
            } else if(rplAction.endsWith("object-placed-at")) {
                action_spec = "(achieve (object-placed-at "+ obj_desig +" "+ loc_desig +"))";
                
            } else if(rplAction.endsWith("arm-parked")) {
                action_spec = "(achieve (arm-parked ?side))";
                
            } else if(rplAction.endsWith("arms-at")) {
                action_spec = "(achieve (arms-at ?traj))";
                
            } else if(rplAction.endsWith("looking-at")) {
                action_spec = "(achieve (looking-at "+ loc_desig +"))";
                
            } else if(rplAction.endsWith("perceive-all")) {
                action_spec = "(perceive-all "+ obj_desig +")";
                
            } else if(rplAction.endsWith("perceive")) {
                action_spec = "(perceive "+ obj_desig +")"; 
            } 

			plan.add(action_spec);

//			String plan = "(def-top-level-plan ehow-make-pancakes1 ()" +
//			
//				    "(with-designators (" +
//				    
//				    "      (pancake2 (an object '((type pancake))))" + 
//				    "      (mixforbakedgoods2 (some stuff '((type pancake-mix))))" + 
//				    "      (refrigerator2 (an object '((type refrigerator))))" + 
//				    "      (cookingvessel2 (an object '((type pan))))" + 
//				    "      (dinnerplate2 (an object '((type plate))))" + 
//				    
//				    "      (location1 (a location `((in ,refrigerator2)" + 
//				    "                               (of ,mixforbakedgoods2))))" + 
//				    "      (location2 (a location `((in ,cookingvessel2)" + 
//				    "                               (of ,mixforbakedgoods2))))" + 
//				    "      (location0 (a location `((on ,dinnerplate2)" + 
//				    "                               (for ,pancake2)))))" + 
//				  
//				  "(achieve `(object-in-hand ,mixforbakedgoods2))" + 
//				  "(achieve `(container-content-transfilled" + 
//				  "                              ,mixforbakedgoods2" + 
//				  "                              ,cookingvessel2))" + 
//				  "(sleep 180)" + 
//				  "(achieve `(object-flipped ,pancake2))" + 
//				  "(sleep 180)" + 
//				  "(achieve `(loc ,pancake2 ,location0)) )))";
//			

		}


		// export to string
		res += "(def-top-level-plan " + plan_name + " () \n";
		res += "(with-designators (";
		
		res += StringUtils.join(designators.values(), "\n");
		
		res += ")";
		
		res += StringUtils.join(plan, "\n");
		
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
                .executeQuery("rdf_has(" + objdef + ", rdf:type, T)");

        String obj_type = "";
        String obj_inst = "";

        String obj_desig = "";
        
        // first case: objdef is an instance
        if (!types.get("T").isEmpty()) {
            
            obj_type = knowrobToCPL.getString(types.get("T").firstElement());
            obj_inst = objdef;
            
            obj_desig = "(" + obj_inst + " (an object '((name " + obj_inst + ")) ))";

        } else { // second case: objdef is a class
            
            obj_type = knowrobToCPL.getString(objdef);
            obj_inst = instanceFromClass(obj_type);
            
            obj_desig = "(" + obj_inst + " (an object '((type " + obj_type + ")) ))";
        }
        // TODO: (part-of <object-desig>)

        designators.put(obj_inst, obj_desig);
        return obj_inst;
    }

    
    
    
    /**
     * Create a location designator based on an OWL class specification
     * 
     * @param loc
     *            OWL class specification of a location
     * @return ID of the generated designator
     */
    private String locationDesignatorFromOWLclass(String loc, String obj_desig) {

        // recursively build the location designator
        String loc_desig = "(" + loc + " ";

        HashMap<String, Vector<String>> params = PrologInterface
                .executeQuery("class_properties(" + loc + ", Prop, Val)");

        for (int i = 0; i < params.get("Prop").size(); i++) {

            String prop = params.get("Prop").get(i);
            String val = params.get("Val").get(i);


            loc_desig += "(a location `(";
            
            if ((prop.endsWith("in-ContGeneric")
                    || prop.endsWith("in-UnderspecifiedContainer")
                    || prop.endsWith("into-UnderspecifiedContainer") 
                    || prop.endsWith("from-UnderspecifiedLocation"))) {

                loc_desig += "(in ,"
                        + locationDesignatorFromOWLclass(val, obj_desig)
                        + ") (of ," + obj_desig + ")";

            } else if (prop.endsWith("to-UnderspecifiedLocation")
                    || prop.endsWith("on-Physical")) {
                loc_desig += "(on ,"
                        + locationDesignatorFromOWLclass(val, obj_desig)
                        + ") (for ," + obj_desig + ")";

            } else if (prop.endsWith("inReachOf")) {
                loc_desig += "(to reach ," + objectDesignatorFromOWLclass(val) + ")";

            } else if (prop.endsWith("visibleFrom")) {
                loc_desig += "(to see ," + objectDesignatorFromOWLclass(val) + ")";

            } else if (prop.endsWith("orientation")) {
                loc_desig += "(pose ," + objectDesignatorFromOWLclass(val) + ")";

            } else {
                loc_desig += "(name ," + val + ")";
            }

        }
        loc_desig += "))";

        designators.put(loc, loc_desig);
        return loc_desig;
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

}

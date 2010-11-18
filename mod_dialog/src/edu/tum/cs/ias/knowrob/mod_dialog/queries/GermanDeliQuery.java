package edu.tum.cs.ias.knowrob.mod_dialog.queries;

import java.util.HashMap;
import java.util.Vector;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import de.tum.in.fipm.kipm.gui.visualisation.applets.CommunicationVisApplet;
import edu.tum.cs.ias.knowrob.json_prolog.PrologValue;
import edu.tum.cs.ias.knowrob.mod_dialog.DialogModule;



public class GermanDeliQuery extends SimpleTopLevelQuery {
	
	public GermanDeliQuery(DialogModule mod) {
		super(mod);
	}

	@Override
	public String match(String q) {

		Matcher matcher = Pattern.compile("^(([R|r]osie)?[,]* would you make us some pancakes\\?)").matcher(q);
		if(matcher.find()) {
			
			// visualize communication
			String shortquery = "rdf_triple(knowrob:forCommand, A, 'make pancakes').\n";
	        CommunicationVisApplet.visualizeCommunication(shortquery, null, "rosie.png", "wikihow.png");

			dialog_module.setVoice("dfki-prudence");
			
			String res = "A = 'http://ias.cs.tum.edu/kb/knowrob.owl#MakePancakesUsingPancakeMix'.\n";
			
			dialog_module.ehow_window.setVisible( true );
			
			// remember plan identifier
			this.dialog_module.setCurrentObject("knowrob:'MakePancakesUsingPancakeMix'");
			
			// visualize communication
	        CommunicationVisApplet.visualizeCommunication(null, res, "rosie.png", "wikihow.png");

			return "Sure, let me look up instructions on the web.";
		}
		

		matcher = Pattern.compile("^([W|w]hat is this\\?)").matcher(q);
		if(matcher.find()) {
			
			
			dialog_module.setVoice("dfki-prudence");
			
			
			String query = "latest_perception_of_type('http://ias.cs.tum.edu/kb/germandeli.owl#GermanDeliObject', A), " +
					"rdf_has(A, knowrob:objectActedOn, Obj), rdf_has(Obj, rdf:type, ObjT)";
	        CommunicationVisApplet.visualizeCommunication(query, null, "", "rosie.png");
	        
	        HashMap<String, Vector<PrologValue>> qres = DialogModule.executeJSONPrologQuery(query);


	        String obj = ""; 
	        String type = "";
	        	
	        if(qres==null)
	        	return null;
	        
	        if(qres.containsKey("Obj") && qres.containsKey("ObjT")) {
	        	obj = qres.get("Obj").toString(); obj=obj.substring(1, obj.length()-1);

				this.dialog_module.setCurrentObject(obj);
	        	type = qres.get("ObjT").toString(); type=type.substring(1, type.length()-1);
	        } else {
	        	return null;
	        } 

	        
			if(obj.contains("#")) {
				obj=DialogModule.removeSingleQuotes(obj).split("#")[1];
			}
			if(type.contains("#")) {
				type=DialogModule.removeSingleQuotes(type).split("#")[1];
			}
						

			String res = "It is a "+type+".\n";
			
			
	        CommunicationVisApplet.visualizeCommunication(null, "Obj="+obj+", ObjT="+type, "", "rosie.png");

			return res;
		}
		
		
		
		
		matcher = Pattern.compile("^([W|w]hat do you need for that\\?)").matcher(q);
		if(matcher.find()) {
			
			q=this.dialog_module.getCurrentObject();
			
			// visualize communication
			
			String obj = "Mondamin_Pfannkuchen_Teig_Mix_200g_Pancake_Mix";
			String query = "owl_direct_subclass_of('http://ias.cs.tum.edu/kb/germandeli.owl#"+obj+"', R)," +
					"owl_has(R, owl:onProperty, germandeli:productID)," +
					"owl_has(R, owl:hasValue, literal(type(xsd:string, ID)))";
			String printquery = "owl_direct_subclass_of(http://ias.cs.tum.edu/kb/germandeli.owl#"+obj+", R), \n" +
			"owl_has(R, owl:onProperty, germandeli:productID), \n" +
			"owl_has(R, owl:hasValue, literal(type(xsd:string, ID))).";
	        CommunicationVisApplet.visualizeCommunication(printquery, null, null, "rosie.png");
	        
			dialog_module.setVoice("dfki-prudence");
			
	        String id = "";
	        HashMap<String, Vector<PrologValue>> qres = DialogModule.executeJSONPrologQuery(query);
	        if(qres.containsKey("ID")) {
	        	id = qres.get("ID").toString(); id=id.substring(1, id.length()-1);
	        }

			dialog_module.showImageInNewWindow("/work/germandeli/images/"+id+".jpg");

			String res = "I need some pancake mix like this one I found on the German Deli website.\n";
			
			// remember object
			this.dialog_module.setCurrentObject("knowrob:'PancakeMix'");
			
			// visualize communication
	        CommunicationVisApplet.visualizeCommunication(null, res, "", "rosie.png");

			return res;
		}
		
		
		
		matcher = Pattern.compile("^([C|c]an you recognize it\\?)").matcher(q);
		if(matcher.find()) {
			
			q=this.dialog_module.getCurrentObject();
			
			// visualize communication
			
			String obj = "Mondamin_Pfannkuchen_Teig_Mix_200g_Pancake_Mix";
			String query = "owl_direct_subclass_of('http://ias.cs.tum.edu/kb/germandeli.owl#"+obj+"', R)," +
			"owl_has(R, owl:onProperty, germandeli:productID)," +
			"owl_has(R, owl:hasValue, literal(type(xsd:string, ID)))";
			String printquery = "owl_direct_subclass_of(http://ias.cs.tum.edu/kb/germandeli.owl#"+obj+", R), \n" +
			"owl_has(R, owl:onProperty, germandeli:productID), \n" +
			"owl_has(R, owl:hasValue, literal(type(xsd:string, ID))).";
	        CommunicationVisApplet.visualizeCommunication(printquery, null, null, "rosie.png");

			dialog_module.setVoice("dfki-prudence");
			
	        String id = "";
	        HashMap<String, Vector<PrologValue>> qres = DialogModule.executeJSONPrologQuery(query);
	        if(qres.containsKey("ID")) {
	        	id = qres.get("ID").toString(); id=id.substring(1, id.length()-1);
	        }
			dialog_module.showImageInNewWindow("/work/germandeli/images_with_keypoints/"+id+".jpg");
			
			
			String res = "Yes, using SIFT features.\n"; 
			CommunicationVisApplet.visualizeCommunication(null, res, "", "rosie.png");
			
			return res;
		}
		
		
		
		matcher = Pattern.compile("^([W|w]here would you search for it\\?)").matcher(q);
		if(matcher.find()) {
			
			q=this.dialog_module.getCurrentObject();
			dialog_module.setVoice("dfki-prudence");
			
			
			String query = "storagePlaceForBecause(StLoc, knowrob:'PancakeMix', Why), highlight_object(StLoc, _)";
	        CommunicationVisApplet.visualizeCommunication(query, null, "", "rosie.png");
	        
	        HashMap<String, Vector<PrologValue>> qres = DialogModule.executeJSONPrologQuery(query);


	        String loc = ""; 
	        String why = "";
	        	
	        if(qres.containsKey("StLoc") && qres.containsKey("Why")) {
				loc = qres.get("StLoc").toString(); loc=loc.substring(1, loc.length()-1);
				why = qres.get("Why").toString(); why=why.substring(1, why.length()-1);
	        } else {
	        	return null;
	        } 

	        
			if(loc.contains("#")) {
				loc=DialogModule.removeSingleQuotes(loc).split("#")[1];
			}
			if(why.contains("#")) {
				why=DialogModule.removeSingleQuotes(why).split("#")[1];
			}
						

			String res = "In "+loc+", because it is "+why+".\n";
			
	        CommunicationVisApplet.visualizeCommunication(null, "StLoc="+loc+", Why="+why, "", "rosie.png");

			return res;
		}
		
		
		
		matcher = Pattern.compile("^([C|c]an you get the pancake mix from the fridge\\?)").matcher(q);
		if(matcher.find()) {
			

	        String query = "returnMissingCapsForAction(srdl_action:'GetObjectFromRefrigerator',  rosie:'TUM_Rosie_RobotInstance1', Missing)";

			// visualize communication
	        CommunicationVisApplet.visualizeCommunication(query, null, "", "rosie.png");
	        dialog_module.setVoice("dfki-prudence");
	        
	        
			HashMap<String, Vector<PrologValue>> qres = DialogModule.executeJSONPrologQuery(query);

			String res = "No.\n";
			if(qres==null) {
				return "Yes.\n";
			} else {
				
				if(qres!=null && qres.containsKey("Missing")&&qres.get("Missing").get(0).islist()) {
					
					
					// visualize communication
			        CommunicationVisApplet.visualizeCommunication(null,"Missing = "+qres.get("Missing").get(0).toString(), "", "rosie.png");

					
					Vector<PrologValue> caps = (Vector<PrologValue>)qres.get("Missing").get(0).getValue();

					res = "No, because I am missing the capabilities for ";
					for(int i=0;i<caps.size();i++) {
					
						if(caps.get(i)==null)
							continue;
						
						String cap=caps.get(i).toString();
						if(cap.contains("#")) {
							cap=DialogModule.removeSingleQuotes(cap).split("#")[1];
						}
						res+=cap;
						
						if(i==caps.size()-2) {
							res+=", and ";
						} else if (i==caps.size()-1) {
							res+=".\n";
						} else { 
							res+=" ";
						}
					}
					
				}
			}
			
			return res;
		}
		

		matcher = Pattern.compile("^([J|j]ames show me a location that can be reached by rosie)").matcher(q);
		if(matcher.find()) {
			
			q=this.dialog_module.getCurrentObject();
			dialog_module.setVoice("dfki-prudence");
			
			
			// visualize communication
			String query = "(visualize-location-designator ((on table) (name kitchen-island) (in reach)) ?_)";
	        CommunicationVisApplet.visualizeCommunication(query, null, "", "pr2.jpg");

	        DialogModule.executeLispPrologQuery(query);
	        
			
			String res = "This is the distribution.\n"; 
			CommunicationVisApplet.visualizeCommunication(null, res, "", "pr2.jpg");

			return res;
		}

		
		matcher = Pattern.compile("^([J|j]ames[,]* can you get the pancake mix from the fridge\\?)").matcher(q);
		if(matcher.find()) {
			
	        
	        // if query fails no capabilities are missing, therefore 'Yes'
	        String query = "returnMissingCapsForAction(srdl_action:'GetObjectFromRefrigerator',  pr2:'PR2_Instance1', Missing)";

			// visualize communication
	        CommunicationVisApplet.visualizeCommunication(query, null, "", "pr2.jpg");
	        dialog_module.setVoice("dfki-obadiah");

	        
			HashMap<String, Vector<PrologValue>> qres = DialogModule.executeJSONPrologQuery(query);

			String res = "No.\n";
			if(qres==null) {
				CommunicationVisApplet.visualizeCommunication(null,"Missing=[]", "", "pr2.jpg");
				return "Yes.\n";			
			}
	        
			// visualize communication
	        CommunicationVisApplet.visualizeCommunication(null, "No.", "", "pr2.jpg");

			return res;
		}

		
		
		matcher = Pattern.compile("^([J|j]ames[,]* where can you put it down\\?)").matcher(q);
		if(matcher.find()) {
			
			// visualize communication
			String shortquery = "James, where can you put it down?";
	        CommunicationVisApplet.visualizeCommunication(shortquery, null, "", "pr2.jpg");
			dialog_module.setVoice("dfki-obadiah");
			
			
			String res = "Right over here.\n";
			
			// visualize communication
	        CommunicationVisApplet.visualizeCommunication(null, res, "", "pr2.jpg");

			return res;
		}


		
		return null;
	}

	

}

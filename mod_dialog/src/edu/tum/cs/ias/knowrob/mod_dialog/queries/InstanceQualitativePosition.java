package edu.tum.cs.ias.knowrob.mod_dialog.queries;

import java.util.HashMap;
import java.util.Vector;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import de.tum.in.fipm.kipm.gui.visualisation.applets.CommunicationVisApplet;


import edu.tum.cs.ias.knowrob.json_prolog.PrologValue;
import edu.tum.cs.ias.knowrob.mod_dialog.DialogModule;

public class InstanceQualitativePosition extends SimpleTopLevelQuery {

	public InstanceQualitativePosition(DialogModule mod) {
		super(mod);
	}

	@Override
	public String match(String q) {
		

		Matcher matcher = Pattern.compile("([w|W]here is ([a-zA-Z0-9]*?)\\?)").matcher(q);
		if(matcher.find()) {
			q=matcher.group(2);
			q=DialogModule.toProlog(q);
			dialog_module.setCurrentObject(q);

			// visualize communication
			String shortquery = "rdf_has(Prop, subPropertyOf, topologicalRelations)," +
			"rdf_triple(A, "+q+", O)";
	        CommunicationVisApplet.visualizeCommunication(shortquery, null, null, "rosie.png");
	
			
			String query = "rdf_has(Prop, rdfs:subPropertyOf, knowrob:topologicalRelations)," +
			"rdf_triple(A, "+q+", O)";

			HashMap<String, Vector<PrologValue>> res = DialogModule.executeJSONPrologQuery(query);

			if(res.get("Prop")==null || res.get("O")==null)
				return null;
			
			String prop = res.get("Prop").toString();
			String obj  = res.get("O").toString();
			
			// visualize communication
	        CommunicationVisApplet.visualizeCommunication(null, prop + ": " + obj, null, "rosie.png");
			
			return prop + " "+obj+".\n"; 
		}
		
		return null;
	}

}

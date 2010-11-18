package edu.tum.cs.ias.knowrob.mod_dialog.queries;

import java.util.HashMap;
import java.util.Vector;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.tum.cs.ias.knowrob.json_prolog.PrologValue;
import edu.tum.cs.ias.knowrob.mod_dialog.DialogModule;

public class SimilarClass extends SimpleTopLevelQuery {

	public SimilarClass(DialogModule mod) {
		super(mod);
	}

	@Override
	public String match(String q) {
		
		Matcher matcher = Pattern.compile("([d|D]o you know anything similar to ([a-zA-Z0-9]*?)\\?)").matcher(q);
		if(matcher.find()) {
			q=matcher.group(2);
			dialog_module.setCurrentObject(DialogModule.toProlog(q));
			
//			String query = "comp_similarity:rdf_most_similar('http://ias.cs.tum.edu/kb/knowrob.owl#Cup', 'http://ias.cs.tum.edu/kb/knowrob.owl#HumanScaleObject', 4, NMostSim)";
			String query = "comp_similarity:rdf_most_similar("+DialogModule.toProlog(q)+", knowrob:'HumanScaleObject', 4, NMostSim)";

			HashMap<String, Vector<PrologValue>> res = DialogModule.executeJSONPrologQuery(query);

			Vector<PrologValue> obj  = (Vector<PrologValue>)res.get("NMostSim");
			
			
			q=DialogModule.toEnglish(q);
			String r = "Yes, for instance ";
			
			if(obj!=null && obj.size()>0) {
				PrologValue res_list = obj.get(0);
			}
			
			for(int i=0;i<obj.size();i++) {

				String val=obj.get(i).toString();
				
				if(val.contains("#")) {
					val=DialogModule.removeSingleQuotes(val).split("#")[1];
				}
				
				r+= DialogModule.toEnglish(val);

				if(i==obj.size()-2) {
					r+=", and ";
				} else if (i==obj.size()-1) {
					r+=".";
				} else { 
					r+=", ";
				}
			}
		}
		
		return null;
	}

}

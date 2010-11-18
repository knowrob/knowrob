package edu.tum.cs.ias.knowrob.mod_dialog.queries;

import java.util.HashMap;
import java.util.Vector;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.tum.cs.ias.knowrob.json_prolog.PrologValue;
import edu.tum.cs.ias.knowrob.mod_dialog.DialogModule;

public class InstanceEdible extends SimpleTopLevelQuery {

	public InstanceEdible(DialogModule mod) {
		super(mod);
	}

	@Override
	public String match(String q) {
		
		Matcher matcher = Pattern.compile("([c|C]an you eat it\\?)").matcher(q);
		if(matcher.find()) {
			
			q=this.dialog_module.getCurrentObject();
			String query = "owl_subclass_of('"+q+"', knowrob:'Food'); owl_individual_of('"+q+"', knowrob:'Food')";
			
			HashMap<String, Vector<PrologValue>> res = DialogModule.executeJSONPrologQuery(query);

			if(res==null) {
				return "No.\n";				
			} else {
				return "Yes.\n";
			}
		}

		matcher = Pattern.compile("([c|C]an you drink it\\?)").matcher(q);
		if(matcher.find()) {
			
			q=this.dialog_module.getCurrentObject();
			String query = "owl_subclass_of('"+q+"', knowrob:'Drink'); owl_individual_of('"+q+"', knowrob:'Drink')";

			HashMap<String, Vector<PrologValue>> res = DialogModule.executeJSONPrologQuery(query);

			if(res==null) {
				return "No.\n";				
			} else {
				return "Yes.\n";
			} 
		}
		
		return null;
	}
	
}

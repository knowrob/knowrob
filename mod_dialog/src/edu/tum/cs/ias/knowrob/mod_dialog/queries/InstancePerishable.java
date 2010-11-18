package edu.tum.cs.ias.knowrob.mod_dialog.queries;

import java.util.HashMap;
import java.util.Vector;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.tum.cs.ias.knowrob.json_prolog.PrologValue;
import edu.tum.cs.ias.knowrob.mod_dialog.DialogModule;

public class InstancePerishable extends SimpleTopLevelQuery {

	public InstancePerishable(DialogModule mod) {
		super(mod);
	}

	@Override
	public String match(String q) {
		
		Matcher matcher = Pattern.compile("([i|I]s it perishable\\?)").matcher(q);
		if(matcher.find()) {
			
			q=this.dialog_module.getCurrentObject();
			String query = "owl_subclass_of("+q+", knowrob:'Perishable'); owl_individual_of("+q+", knowrob:'Perishable')";

			HashMap<String, Vector<PrologValue>> res = DialogModule.executeJSONPrologQuery(query);

			if(res==null) {
				return "No.\n";				
			} else {
				return "Yes.\n";
			} 
		}		
		matcher = Pattern.compile("([i|I]s it heat sensitive\\?)").matcher(q);
		if(matcher.find()) {
			
			q=this.dialog_module.getCurrentObject();
			String query = "owl_subclass_of('"+q+", knowrob:'HeatSensitive'); owl_individual_of("+q+", knowrob:'HeatSensitive')";

			HashMap<String, Vector<PrologValue>> res = DialogModule.executeJSONPrologQuery(query);

			if(res==null) {
				return "No.\n";				
			} else {
				return "Yes.\n";
			} 
		}	
		matcher = Pattern.compile("([i|I]s it frozen\\?)").matcher(q);
		if(matcher.find()) {
			
			q=this.dialog_module.getCurrentObject();
			String query = "owl_subclass_of('"+q+", knowrob:'Frozen'); owl_individual_of("+q+", knowrob:'Frozen')";

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

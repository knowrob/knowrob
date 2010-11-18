package edu.tum.cs.ias.knowrob.mod_dialog.queries;

import java.util.HashMap;
import java.util.Vector;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.tum.cs.ias.knowrob.json_prolog.PrologValue;
import edu.tum.cs.ias.knowrob.mod_dialog.DialogModule;

public class InstanceProperties extends SimpleTopLevelQuery {

	public InstanceProperties(DialogModule mod) {
		super(mod);
	}

	@Override
	public String match(String q) {
		

		Matcher matcher = Pattern.compile("([w|W]hat do you know about it\\?)").matcher(q);
		if(matcher.find()) {
			
			q=this.dialog_module.getCurrentObject();
			
			String query = "(rdf_has("+q+", P, O); (rdf_has("+q+", rdf:type, T), rdf_has(T, P, O)))";
			System.err.println(query);
			
			HashMap<String, Vector<PrologValue>> res = DialogModule.executeJSONPrologQuery(query);

			
			if(res!=null && res.containsKey("P") && res.containsKey("O")) {
				
				Vector<PrologValue> prop = res.get("P");
				Vector<PrologValue> obj  = res.get("O");

				q=DialogModule.toEnglish(q);
				String r = q.substring(0,1).toUpperCase() + q.substring(1,q.length()) +" ";
				for(int i=0;i<prop.size();i++) {
				
					if(prop.get(i)==null || obj.get(i)==null)
						continue;
					
					String pr=prop.get(i).toString();
					String val=obj.get(i).toString();
					
					if(pr.contains("#")) {
						pr=DialogModule.removeSingleQuotes(pr).split("#")[1];
					}
					
					if(val.startsWith("literal(type('http://www.w3.org/2001/XMLSchema#float")) {
						val=DialogModule.removeSingleQuotes(val).substring(56, val.length()-3);
					} else if(val.contains("#")) {
						val=DialogModule.removeSingleQuotes(val).split("#")[1];
					}
					r+= DialogModule.toEnglish(pr) +" "+ DialogModule.toEnglish(val);
	
					if(i==prop.size()-2) {
						r+=", and ";
					} else if (i==prop.size()-1) {
						r+=".";
					} else { 
						r+=", ";
					}
				}
				return r+"\n";
			} 
		}
			
		matcher = Pattern.compile("([w|W]hat do you know about ([a-zA-Z0-9]*?)\\?)").matcher(q);
		if(matcher.find()) {
			
			q=matcher.group(2);
			dialog_module.setCurrentObject(DialogModule.toProlog(q));
			
			String query = "rdf_has("+DialogModule.toProlog(q)+", P, O)";
			
			HashMap<String, Vector<PrologValue>> res = DialogModule.executeJSONPrologQuery(query);

			
			if(res!=null && res.containsKey("P") && res.containsKey("O")) {
				
				Vector<PrologValue> prop = res.get("P");
				Vector<PrologValue> obj  = res.get("O");

				q=DialogModule.toEnglish(q);
				String r = q.substring(0,1).toUpperCase() + q.substring(1,q.length()) +" ";
				for(int i=0;i<prop.size();i++) {
				
					if(prop.get(i)==null || obj.get(i)==null)
						continue;
					
					String pr=prop.get(i).toString();
					String val=obj.get(i).toString();
					
					if(pr.contains("#")) {
						pr=DialogModule.removeSingleQuotes(pr).split("#")[1];
					}
					
					if(val.startsWith("literal(type('http://www.w3.org/2001/XMLSchema#float")) {
						val=DialogModule.removeSingleQuotes(val).substring(56, val.length()-3);
					} else if(val.contains("#")) {
						val=DialogModule.removeSingleQuotes(val).split("#")[1];
					}
					r+= DialogModule.toEnglish(pr) +" "+ DialogModule.toEnglish(val);
	
					if(i==prop.size()-2) {
						r+=", and ";
					} else if (i==prop.size()-1) {
						r+=".";
					} else { 
						r+=", ";
					}
				}
				return r+"\n";
			} 
		}

		
		return null;
	}

}

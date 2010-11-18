package edu.tum.cs.ias.knowrob.mod_dialog.queries;

import java.util.HashMap;
import java.util.Vector;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.tum.cs.ias.knowrob.json_prolog.PrologValue;
import edu.tum.cs.ias.knowrob.mod_dialog.DialogModule;

public class SimilarInstancePerception extends SimpleTopLevelQuery {

	public SimilarInstancePerception(DialogModule mod) {
		super(mod);
	}

	@Override
	public String match(String q) {
		
//		Matcher matcher = Pattern.compile("([w|W]here exs ([a-zA-Z0-9]*?)\\?)").matcher(q);
//		if(matcher.find()) {
//			q=matcher.group(2);
//
//			String query = "";
//
//			HashMap<String, Vector<PrologValue>> res = DialogModule.executeQuery(query);
//
//			String x = res.get("X").toString(); x=x.substring(1, x.length()-1);
//			return "No.\n"; 
//		}
		
		return null;
	}
}

package edu.tum.cs.ias.knowrob.mod_dialog.queries;

import edu.tum.cs.ias.knowrob.mod_dialog.DialogModule;

public abstract class SimpleTopLevelQuery extends DialogQuery {

	public SimpleTopLevelQuery(DialogModule mod) {
		super(mod);
	}
	
	@Override
	public String process(String q) {
		if(haveTopLevelState()) {
			return match(q);	
		}		
		return null;
	}
	
	public abstract String match(String q);
}

package edu.tum.cs.ias.knowrob.mod_dialog.queries;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.tum.cs.ias.knowrob.mod_dialog.DialogModule;

public abstract class DialogQuery {
	
	protected DialogModule dialog_module;
	protected Matcher matcher;
	
	public DialogQuery(DialogModule mod) {
		this.dialog_module = mod;
	}
	
	public boolean haveState(String state) {
		return dialog_module.getCurrentState().equals(state);
	}
	
	public boolean haveTopLevelState() {
		return haveState(DialogModule.TOP_LEVEL_STATE);
	}
	
	public void setState(String state) {
		dialog_module.setState(state);
	}
	
	public void setTopLevelState() {
		dialog_module.setTopLevelState();
	}
	
	public abstract String process(String q) throws Exception;
	
	protected boolean match(String regex, String s) {
		matcher = Pattern.compile(regex).matcher(s);
		return matcher.matches();
	}
}

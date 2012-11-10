package edu.tum.cs.ias.knowrob.vis.applets;

import java.util.HashMap;
import java.util.Map;

import controlP5.Button;
import controlP5.ControlEvent;
import controlP5.ControlListener;
import controlP5.ControlP5;
import controlP5.ControlWindow;
import controlP5.ListBox;
import controlP5.ListBoxItem;
import edu.tum.cs.ias.knowrob.vis.actions.ActionTransition;
import edu.tum.cs.ias.knowrob.vis.actions.FsmActionResult;
import edu.tum.cs.ias.knowrob.vis.themes.GreyTheme;


/**
 * Displays a dialog for searching for and downloading action
 * recipes from RoboEarth.
 *  
 * @author tenorth
 *
 */
public class TransitionPropertiesEditor {

	public ControlP5 controlP5;
	ControlWindow controlWindow;
	DialogListener dialogListener;
	
	private ActionTransition transition;
	
	String recipe_owl_file = "";

	public PlanVisAppletFsm app;

	private Map<Integer, ListBoxItem> id2button;
	ListBoxItem cur = null;
	ListBox props;
	
	/**
	 * Constructor
	 * 
	 * @param app Reference to the TeleopInterfaceMain instance
	 * @param gui Reference to the TeleopInterfaceGui instance 
	 * @param guiElem Instance of the GuiElemFactory
	 */
	public TransitionPropertiesEditor(PlanVisAppletFsm applet, ActionTransition t) {
		
		this.app = applet;
		transition = t;
		
		controlP5 = new ControlP5(app);
		GreyTheme.applyStyle(controlP5);
		
		controlWindow = controlP5.addControlWindow("controlP5window",200,200,350,150)
								 .setBackground(app.color(50))
								 .setUpdateMode(ControlWindow.NORMAL)
								 .setTitle("Edit state transitions");

		controlP5.addTextlabel("from action", "From action: " + transition.getFrom().getLabel(), 20, 30 );
		controlP5.addTextlabel("to action",   "To action: " + transition.getFrom().getLabel(),   20, 65 );
		
		Button b = GreyTheme.applyStyle(controlP5.addButton("apply", 23, 220, 30, 80, 20)).moveTo(controlWindow);
		
		
		props = controlP5.addListBox("Transition type", 10, 30, 200, 125);
		props.setBarHeight(15).setItemHeight(15);
		props.setColorValue(app.color(120,40,40));
		GreyTheme.applyStyle(props, 15).moveTo(controlWindow);

		id2button = new HashMap<Integer, ListBoxItem>();
		int props_idx = 0;
		for(FsmActionResult r : FsmActionResult.values()) { // TODO: read from / convert to OWL (via method in FsmActionResult?)
			
			ListBoxItem i = props.addItem(r.toString(), props_idx);

			if(transition.getCause().endsWith(i.getName())) {
				i.setColorBackground(120);
				cur = i;
			}
			
			id2button.put(props_idx, i);
			props_idx++;
		}
				
		dialogListener = new DialogListener();

		b.addListener(dialogListener);
		props.addListener(dialogListener);

	}


	/** 
	 * Custom control event listener attached to the control elements in 
	 * this window.
	 * 
	 * Otherwise, the control events would be sent to the listener that
	 * is part of the main window.
	 * 
	 * @author tenorth
	 *
	 */
	public class DialogListener implements ControlListener {

		public void controlEvent(ControlEvent ev) {
			try {
				
				if (ev != null && ev.isGroup()) {
					
					if(ev.getGroup().getName().equals("Transition type")) {
						
						if(cur!=null)
							cur.setColorBackground(80);
						
						cur = id2button.get((int)ev.getValue());
						cur.setColorBackground(120);
						
					}
					
				} else if (ev.isController()) {
					
					if(ev.getController().getName().equals("apply")) {
						
						transition.setCause(FsmActionResult.toOWLIdentifier(
												FsmActionResult.fromString(
														id2button.get((int)props.getValue()).getName())));
						controlWindow.hide();
						return;
					}
				}
				
			} catch (Exception e ) {
				e.printStackTrace();
			}
		}
	}
	
}

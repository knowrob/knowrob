package edu.tum.cs.ias.knowrob.vis.gui.applets;

import java.awt.Color;
import java.awt.Frame;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import controlP5.Button;
import controlP5.ControlEvent;
import controlP5.ControlGroup;
import controlP5.ControlP5;
import controlP5.ListBox;
import controlP5.ListBoxItem;
import controlP5.Textfield;
import edu.tum.cs.ias.knowrob.vis.actions.Action;


import processing.core.PApplet;
import processing.core.PFont;


public class EditActionPropertiesApplet  extends PApplet implements MouseListener, MouseMotionListener, IClassSelectionCallback {

	private static final long serialVersionUID = 7695328948788620463L;

	public ControlP5 controlP5;
	
	/**
	 * Font for drawing text
	 */
	private PFont dejavuFont;
	
	/**
	 * Reference to the OwlClassSelectorApplet used for selecting an OWL class from an ontology
	 */
	OwlClassSelectorApplet class_selector;
	
	/**
	 * Reference to an iAddActionCallback; usually the parent that started this editor
	 */
	private iAddActionCallback callback;
	

	/**
	 * submit button
	 */
	private Button submit;
	
	/**
	 * List of properties defined for the current action
	 */
	private ListBox action_props;
	
	/**
	 * List of applicable properties, from which the user can select which ones to define
	 */
	private ListBox props;

	/**
	 * currently selected {@link ListBoxItem} in the action_props {@link ListBox}
	 */
	private ListBoxItem currentActionProp;
	
	/**
	 * currently selected {@link ListBoxItem} in the props {@link ListBox}
	 */
	private ListBoxItem currentProp;

	/** 
	 * Mapping from the integer ID (which will be provided by click events) to the 
	 * corresponding {@link ListBoxItem} in the action_props list
	 */
	private Map<Integer, ListBoxItem> actionpropId2item;

	/** 
	 * Mapping from the integer ID (which will be provided by click events) to the 
	 * corresponding {@link ListBoxItem} in the props list
	 */
	private Map<Integer, ListBoxItem> propId2item;
	
	/**
	 * Height of a row in the action_props {@link ListBox}
	 */
	private int listBoxHeight = 20;
	
	/**
	 * Applet completely initialized
	 */
	private boolean initialized = false;
	
	/**
	 * Flag indicating if new action is created or existing action is edited
	 */
	private boolean editing = false;

	
	

	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	//  Init/drawing methods
	// 
	
	
	@Override
	public void setup()
	{
		size(500, 500);

		if (this.frame != null)
		{
			this.frame.setTitle("Edit action properties");
			this.frame.setBackground(new Color(40, 40, 40));
		}
		
		dejavuFont = createFont("DejaVu Sans",12);
		textFont(dejavuFont);
		
		frameRate(20);	
		background(40);
				
		initControlP5();
	}


	/**
	 * Draw the content of the 
	 */
	@Override
	public void draw() {

		background(40);
		controlP5.draw();
	}




	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	//  Public getter/setter interface
	// 
	
	
	/**
	 * Set the action identifier
	 * 
	 * @param id  IRI of the current action 
	 */
	public void setIdentifier(String id) {
		// TODO: do not set GUI element values directly
		((Textfield) this.controlP5.getController("identifier")).setValue(id);
	}
	
	/**
	 * Get the action identifier
	 * 
	 * @return id  IRI of the current action 
	 */
	public String getIdentifier() {
		// TODO: do not set GUI element values directly
		return ((Textfield) this.controlP5.getController("identifier")).getStringValue();
	}
	
	/**
	 * Set the super-class of the currently edited action
	 *  
	 * @param cl IRI of the super-class
	 */
	public void setActionClass(String cl) {
		// TODO: do not set GUI element values directly
		Textfield t = ((Textfield) this.controlP5.getController("class")).setValue(cl);
		
	}
	
	/**
	 * Get the super-class of the currently edited action
	 *  
	 * @return cl IRI of the super-class
	 */
	public String getActionClass() {
		// TODO: do not set GUI element values directly
		return ((Textfield) this.controlP5.getController("class")).getStringValue();
	}
	

	/**
	 * Read all values from the props Map to the action_props list, which
	 * is cleared beforehand.
	 * 
	 * @param props Mapping from the property identifier to a list of values
	 */
	public void setActionProperties(Map<String, List<String>> props) {
		
		action_props.clear();
		addActionProperties(props);
	}
	
	
	/**
	 * Read all values from the props Map to the action_props list in addition
	 * to the existing ones
	 * 
	 * @param props Mapping from the property identifier to a list of values
	 */
	public void addActionProperties(Map<String, List<String>> props) {
		
		for(String prop : props.keySet()) {
			for(String val : props.get(prop)) {
				
				if(prop.endsWith("type")) {
					this.setActionClass(val);
				} else {
					addActionPropItem(prop, val);
				}
			}
		}
	}

	
	/** 
	 * Add a new property/value pair to the list of action properties
	 *  
	 * @param prop String denoting the property to be added
	 * @param val  String denoting the value to be assigned to the property
	 */
	public void addActionPropItem(String prop, String val) {
		addActionPropItem(prop + " -- " + val);
	}
		
	
	/**
	 * Add a new item to the action_props {@link ListBox}.
	 * 
	 * For internal use, use the public {@see addActionPropItem} instead.
	 * 
	 * @param name String used for the item, composed of its property and value
	 */
	private void addActionPropItem(String name) {
		
		int idx = 0;
		if(actionpropId2item.size()>0)
			idx = Collections.max(actionpropId2item.keySet()) + 1;
		
		ListBoxItem item = actionpropId2item.put(idx, applyStyle(action_props.addItem(name, idx)));
		setCurrentActionPropItem(item);
		
		// adjust height of the action properties ListBox and position of the submit button
		if(submit.getPosition().y < this.height-50) {
			
			listBoxHeight += 20;
			action_props.setHeight(listBoxHeight);
			submit.setPosition(submit.getPosition().x, submit.getPosition().y + 20);
		}
	}
	
	
	/**
	 * Remove a {@link ListBoxItem} from the action_props {@link ListBox}
	 * 
	 * @param item Reference to a {@link ListBoxItem} that is to be removed
	 */
	private void removeActionPropItem(ListBoxItem item) {
		
		if(item!=null) {
			
			action_props.removeItem(item.getName());
			currentActionProp = null;
			actionpropId2item.remove(item.getValue());
			
			// adapt layout
			listBoxHeight += 20;
			action_props.setHeight(listBoxHeight);
			submit.setPosition(submit.getPosition().x, submit.getPosition().y - 20);
		}
	}

	/**
	 * set the currently selected item in the action properties {@link ListBox}
	 * 
	 * @param item Reference to the item to be set as 'selected'
	 */
	private void setCurrentActionPropItem(ListBoxItem item) {
		
		if(currentActionProp!=null) {
			currentActionProp.setColorBackground(color(80));
		}

		// also allow 'null' as item -> reset highlight
		currentActionProp = item;

		if(item!=null) {
			item.setColorBackground(color(120));
		}
	}
	
	
	public boolean isEditing() {
		return editing;
	}
	
	public void setEditing(boolean e) {
		
		this.editing = e;
		if(editing) {
			Textfield id = ((Textfield) controlP5.getController("identifier"));
			controlP5.addTextlabel("edited identifier", "Editing action: '" +id.getText() +"'", (int)id.getPosition().x, (int)id.getPosition().y);
			id.setVisible(false);
			
			controlP5.getController("submit").setCaptionLabel("update action");
		}
	}
	
	public boolean isInitialized() {
		return initialized;
	}
	
	

	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	//  Event handling
	// 

	/**
	 * Handle controlP5 mouse events
	 */
	public void controlEvent(ControlEvent ev) {


		if(ev.isController()) {
			

			// open class selection dialog	
			if(ev.getController().getName().equals("select class")) {
				OWLClassSelect f = new OWLClassSelect();
				f.setClassSelectedCallback(this); 
			}


			// add action property
			if(ev.getController().getName().equals("add property") || ev.getController().getName().equals("value")) {
				addActionPropItem(propId2item.get((int)props.getValue()).getName(), ((Textfield) controlP5.getController("value")).getText());
				
				// reset text field and properties box
				((Textfield) controlP5.getController("value")).setText("");
				
				if(currentProp!=null)
					currentProp.setColorBackground(color(80));
				
				currentProp = null;
			}

			
			// remove action property
			if(ev.getController().getName().equals("remove property")) {
				if(currentActionProp!=null) {
					removeActionPropItem(currentActionProp);
					currentActionProp = null;
				}
			}
			
			// submit action to parent object
			if(ev.getController().getName().equals("submit")) {
				
				String id = ((Textfield) controlP5.getController("identifier")).getText();
				Action act = new Action(id, id);

				for(ListBoxItem i : actionpropId2item.values()) {
					
					if(i!=null) {
						String[] prop_val = i.getName().split(" -- ", 2);
						act.setProperty(prop_val[0], prop_val[1]);
					}			
				}
				
				act.setProperty("type", ((Textfield) controlP5.getController("class")).getText());
				
				
				if(editing)
					this.callback.updateAction(act);
				else 
					this.callback.addAction(act);
				
				this.frame.setVisible(false);
			}
		}
		
		
		if(ev.isGroup()) {
			
			if(ev.getGroup().getName().equals("Action properties")) {
				
				ListBoxItem item = actionpropId2item.get((int)ev.getValue());
				setCurrentActionPropItem(item);
			}
			
			
			if(ev.getGroup().getName().equals("Properties")) {
				
				ListBoxItem item = propId2item.get((int)ev.getValue());
				
				if(item!=null) {
					if(currentProp!=null)
						currentProp.setColorBackground(color(80));
					
					currentProp = item;
					item.setColorBackground(color(120));
				}
			}
		}
		
	}

	

	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	//  Callback methods
	// 
	
	/**
	 * Implementation of the {@link IClassSelectionCallback} interface; accept
	 * OWL classes selected by external components.
	 * 
	 * @param cl IRI of the class that has been selected in the external program
	 */
	@Override
	public void owlClassSelected(String cl) {
		setActionClass(cl);
	}

	/**
	 * 
	 * @param cb Set 
	 */
	public void setAddActionCallback(iAddActionCallback cb) {
		this.callback = cb;
	}
	
	
	/**
	 * Wrapper class around an OWL class selector applet in a new window
	 *  
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 */
	public class OWLClassSelect extends Frame {
		private static final long serialVersionUID = -7174804883131977878L;

		public OWLClassSelect() {
	        setBounds(100,100,800,600);
	        class_selector = new OwlClassSelectorApplet();
	        class_selector.frame = this;
	        add(class_selector);
	        class_selector.init();
			this.setVisible(true);
	    }

		public void setClassSelectedCallback(IClassSelectionCallback cb) {
			class_selector.setClassSelectedCallback(cb); 
		}
	}
	
	
	

	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	//  GUI initialization and styling
	// 

	/**
	 * Initialize the controlP5 library and create GUI elements
	 */
	private void initControlP5() {

		controlP5 = new ControlP5(this);
		
		controlP5.setColorForeground(color(180))
		.setColorCaptionLabel(color(240))
		.setColorBackground(color(80))
		.setColorActive(color(200));
		
		controlP5.addTextfield("identifier", 20, 20, 300, 20).setAutoClear(false).setFocus(true);
				
		controlP5.addTextfield("class", 20, 60, 300, 20).setAutoClear(false);
		controlP5.addButton("select class", 1, 330, 60, 35, 20).setCaptionLabel("select");

		submit = controlP5.addButton("submit", 1, 355, 275, 65, 20).setCaptionLabel("add to task");
		
		
		ControlGroup<?> new_prop = controlP5.addGroup("Add properties", 20, 120, 400);
		new_prop.setBarHeight(15);
		new_prop.getCaptionLabel().getStyle().marginTop = 3;
		new_prop.getValueLabel().getStyle().marginTop = 3; 
		new_prop.setBackgroundHeight(105);
		new_prop.setBackgroundColor(color(50));
		
		
		// TODO read properties from KnowRob
		props = controlP5.addListBox("Properties", 10, 30, 200, 75);
		props.setBarHeight(15).setItemHeight(15);
		props.setColorValue(color(120,40,40));
		applyStyle(props).moveTo(new_prop);

		int props_idx = 0;
		propId2item = new HashMap<Integer, ListBoxItem>();
		
		propId2item.put(props_idx, applyStyle(props.addItem("objectActedOn", props_idx)).setText("objectActedOn"));
		props_idx++;
		propId2item.put(props_idx, applyStyle(props.addItem("toLocation",    props_idx++)).setText("toLocation"));
		props_idx++;
		propId2item.put(props_idx, applyStyle(props.addItem("fromLocation",  props_idx++)).setText("fromLocation"));
		props_idx++;
		propId2item.put(props_idx, applyStyle(props.addItem("bodyPartUsed",  props_idx++)).setText("bodyPartsUsed"));
		props_idx++;
		
		// TODO: create mapping from displayed names to IRIs for properties and (possibly) values
		
		
		
		controlP5.addTextfield("value", 230, 15, 160, 20).setAutoClear(true).moveTo(new_prop);

		controlP5.addButton("add property", 2, 230, 55, 33, 20).moveTo(new_prop).setCaptionLabel(" add");
		controlP5.addButton("remove property", 2, 280, 55, 50, 20).moveTo(new_prop).setCaptionLabel(" remove");
		
		
		action_props = controlP5.addListBox("Action properties", 20, 260, 400, listBoxHeight).setBarHeight(15).setItemHeight(20);
		applyStyle(action_props);
		
		
		actionpropId2item = new HashMap<Integer, ListBoxItem>();
		setCurrentActionPropItem(null);
		
		this.initialized   = true;
		
	}
	

	/**
	 * Apply the default styling to a ListBoxItem
	 * 
	 * @param b The item to be styled
	 * @return the same button instance with updated properties
	 */
	private ListBoxItem applyStyle(ListBoxItem b) {
		b.setColorBackground(color(80));
		return b;
	}
	/**
	 * Apply the default styling to a ListBox
	 * 
	 * @param b The list to be styled
	 * @return the same list instance with updated properties
	 */
	private ListBox applyStyle(ListBox lb) {
		
		lb.setColorBackground(color(120));
		lb.getCaptionLabel().getStyle().marginTop = 3;
		lb.getValueLabel().getStyle().marginTop = 3; 
		
		return lb;
	}

	

	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	//  Main method
	// 
	
	
	public static void main(String args[]) {
		PApplet.main(new String[] { "edu.tum.cs.ias.knowrob.vis.gui.applets.EditActionPropertiesApplet" });
	}



}


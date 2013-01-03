package edu.tum.cs.ias.knowrob.vis.applets;

import java.awt.Color;
import java.awt.Frame;
import java.awt.event.KeyEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Vector;

import controlP5.Button;
import controlP5.ControlEvent;
import controlP5.ControlGroup;
import controlP5.ControlP5;
import controlP5.ListBox;
import controlP5.ListBoxItem;
import controlP5.Textfield;
import edu.tum.cs.ias.knowrob.owl.OWLClass;
import edu.tum.cs.ias.knowrob.owl.OWLThing;
import edu.tum.cs.ias.knowrob.vis.actions.Action;
import edu.tum.cs.ias.knowrob.vis.themes.GreyTheme;


import processing.core.PApplet;
import processing.core.PFont;


public class ActionPropertiesEditor  extends PApplet implements MouseListener, MouseMotionListener, IClassSelectionCallback {

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
	private IAddActionCallback callback;
	

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

	private Textfield identifier;

	private Textfield cls;

	private Textfield value;

	private boolean propsIsActive;

	private String base_iri;
	
	
	

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
		
		frameRate(15);	
		background(50);
				
		initControlP5();
	}


	/**
	 * Draw the content of the applet
	 */
	@Override
	public void draw() {

		background(40);
		
		if(initialized) {
			controlP5.draw();
		}
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
		((Textfield) this.controlP5.getController("identifier")).setValue(id);
	}
	
	/**
	 * Get the action identifier
	 * 
	 * @return id  IRI of the current action 
	 */
	public String getIdentifier() {
		return ((Textfield) this.controlP5.getController("identifier")).getStringValue();
	}
	
	/**
	 * Set the super-class of the currently edited action
	 *  
	 * @param cl IRI of the super-class
	 */
	// TODO: extend to the case of multiple super-classes
	public void setActionClass(String cl) {
		((Textfield) this.controlP5.getController("class")).setValue(cl);
	}
	
	/**
	 * Get the super-class of the currently edited action
	 *  
	 * @return cl IRI of the super-class
	 */
	public String getActionClass() {
		return ((Textfield) this.controlP5.getController("class")).getStringValue();
	}
	

	/**
	 * Read all values from the props Map to the action_props list, which
	 * is cleared beforehand.
	 * 
	 * @param props Mapping from the property identifier to a list of values
	 */
	public void setActionProperties(Map<String, Vector<String>> props) {

		synchronized (action_props) {
			action_props.clear();
		}
		addActionProperties(props);
	}
	
	
	/**
	 * Read all values from the props Map to the action_props list in addition
	 * to the existing ones
	 * 
	 * @param new_props Mapping from the property identifier to a list of values
	 */
	public void addActionProperties(Map<String, Vector<String>> new_props) {
		
		for(String prop : new_props.keySet()) {
			for(String val : new_props.get(prop)) {

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
		
		String prop_short = OWLThing.getShortNameOfIRI(prop);
		
		// find largest index (not necessarily consecutive)
		int props_idx = props.getListBoxItems().length + 1;
		for(String[] item : props.getListBoxItems()) {
			if(Integer.valueOf(item[2]) > props_idx)
				props_idx = Integer.valueOf(item[3]);
		}
		props_idx++;
		
		// add property to properties list if not there yet so that mapping shortname->IRI works
		if(getIRIForPropShortName(prop_short) == null) {
			synchronized (props) {
				propId2item.put(props_idx, props.addItem(prop, props_idx).setText(OWLThing.getShortNameOfIRI(prop)));
			}
		}

		addActionPropItem(prop_short + " -- " + val);
						
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
		
		synchronized (action_props) {
			ListBoxItem item = actionpropId2item.put(idx, action_props.addItem(name, idx));
			setCurrentActionPropItem(item);
		}
		
		// adjust height of the action properties ListBox and position of the submit button
		if(submit.getPosition().y < this.height-50) {
			
			listBoxHeight += 20;
			synchronized (action_props) {
				action_props.setHeight(listBoxHeight);
			}
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
			
			// remove from action class
			String id = ((Textfield) controlP5.getController("identifier")).getText();
			Action act = Action.getAction(id, null);
			
			String[] prop_val = item.getName().split(" -- ", 2);
			String prop = getIRIForPropShortName(prop_val[0]);
			String val = prop_val[1];

			if(act.getHasValue().containsKey(prop)) {
				act.getHasValue().get(prop).remove(val);  // TODO: we should directly propagate the change to Prolog here...
				
				if(act.getHasValue().get(prop).isEmpty())
					act.getHasValue().remove(prop);
			}
			if(act.getSomeValuesFrom().containsKey(prop)) {
				act.getSomeValuesFrom().get(prop).remove(val);

				if(act.getSomeValuesFrom().get(prop).isEmpty())
					act.getSomeValuesFrom().remove(prop);
			}
			act.getDrawInfo().notifyModified();
			
			synchronized (action_props) {
				
				// update gui
				action_props.removeItem(item.getName());
				currentActionProp = null;
				actionpropId2item.remove(item.getValue());

				// adapt layout
				listBoxHeight += 20;
				action_props.setHeight(listBoxHeight);
				submit.setPosition(submit.getPosition().x, submit.getPosition().y - 20);
			}
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
	
	private String getIRIForPropShortName(String shortname) {
		
		// getListBoxItems returns String[]{name, value}
		synchronized (props) {
			for(String[] i : props.getListBoxItems()) {
				if(i[0].endsWith(shortname)) {
					return i[0];
				}
			}
		}
		return null;
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
			if(ev.getController().getName().equals("class") || ev.getController().getName().equals("select class")) {
				OWLClassSelect f = new OWLClassSelect("http://ias.cs.tum.edu/kb/knowrob.owl#PurposefulAction");
				f.setClassSelectedCallback(this); 
			}


			// add action property
			if(ev.getController().getName().equals("add property") || ev.getController().getName().equals("value")) {
				
				
				synchronized (props) {
					
					addActionPropItem(propId2item.get((int)props.getValue()).getName(), ((Textfield) controlP5.getController("value")).getText());
					
					// add to action class
					String id = ((Textfield) controlP5.getController("identifier")).getText();
					
					// add default namespace if none is given
					if(!id.contains("#")) {
						id = base_iri + id;
					}
					
					Action act = Action.getAction(id, null);
					act.addHasValue(propId2item.get((int)props.getValue()).getName(), ((Textfield) controlP5.getController("value")).getText());
					act.setSaveToProlog(true);
					act.getDrawInfo().notifyModified();

					// TODO: somehow distinguish between someValuesfrom and hasValue -> create OWLProperty type and use hasValue for DataProperties?

				}
				
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
				
				String id = ((Textfield) controlP5.getController("identifier")).getText(); // TODO: add label field to the action editor forms

				// add default namespace if none is given
				if(!id.contains("#")) {
					id = base_iri + id;
				}
				Action act = Action.getAction(id, null);
				act.setSaveToProlog(true);

				OWLClass sup = OWLClass.getOWLClass(((Textfield) controlP5.getController("class")).getText());
				act.getSuperClasses().clear();
				act.addSuperClass(sup);
				
				if(!editing)
					this.callback.addAction(act);
				
				// Note: updates are automatically propagated since actions are implemented as singletons
				
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

	
	public void keyPressed(KeyEvent e) {
		
		// iterate through form fields with TAB
		if(e.getKeyCode() == KeyEvent.VK_TAB) {
			
			if(identifier.isActive()) {
				identifier.setFocus(false);
				if(e.isShiftDown())
					value.setFocus(true);
				else 
					cls.setFocus(true);
				return;
				
			} else if(cls.isActive()) {
				cls.setFocus(false);
				props.setValue(0);
				if(e.isShiftDown())
					identifier.setFocus(true);
				else 
					propsIsActive = true;
				return;
				
				
			} else if(propsIsActive) {
				propsIsActive = false;
				if(e.isShiftDown())
					cls.setFocus(true);
				else 
					value.setFocus(true);
				return;
				
			} else if(value.isActive()) {
				value.setFocus(false);
				if(e.isShiftDown())
					propsIsActive = true;
				else 
					identifier.setFocus(true);
				return;
			}
			
		} else if(e.getKeyCode() == KeyEvent.VK_DOWN && propsIsActive) {
			if(props.getValue() < props.getListBoxItems().length-1)
				props.setValue(props.getValue()+1);

		} else if(e.getKeyCode() == KeyEvent.VK_UP && propsIsActive) {
			if(props.getValue() > 0)
				props.setValue(props.getValue()-1);
			
		} else {
			controlP5.keyHandler.keyEvent(e, controlP5.controlWindow, true);
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
	public void setAddActionCallback(IAddActionCallback cb) {
		this.callback = cb;
	}

	/**
	 * 
	 * @param base_iri
	 */
	public void setBaseIRI(String base_iri) {
		this.base_iri = base_iri;		
	}

	
	/**
	 * Wrapper class around an OWL class selector applet in a new window
	 *  
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 */
	public class OWLClassSelect extends Frame {
		private static final long serialVersionUID = -7174804883131977878L;

		public OWLClassSelect(String baseclass) {
	        setBounds(100,100,800,600);
	        class_selector = new OwlClassSelectorApplet();
	        class_selector.frame = this;
	        add(class_selector);
	        class_selector.init();
	        class_selector.setBaseClass(baseclass);
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
		GreyTheme.applyStyle(controlP5);
		
		
		identifier = controlP5.addTextfield("identifier", 20, 20, 300, 20).setAutoClear(false).setFocus(true);
				
		cls = controlP5.addTextfield("class", 20, 60, 300, 20).setAutoClear(false);
		controlP5.addButton("select class", 1, 330, 60, 35, 20).setCaptionLabel("select");

		submit = controlP5.addButton("submit", 1, 355, 275, 65, 20).setCaptionLabel("add to task");
		
		
		ControlGroup<?> new_prop = GreyTheme.applyStyle(controlP5.addGroup("Add properties", 20, 120, 400)); 
		new_prop.setBackgroundHeight(105);
		
		
		props = controlP5.addListBox("Properties", 10, 30, 200, 75);
		props.setBarHeight(15).setItemHeight(15);
		props.setColorValue(color(120,40,40));
		GreyTheme.applyStyle(props, 15).moveTo(new_prop);

		int props_idx = 0;
		propId2item = new HashMap<Integer, ListBoxItem>();

		// TODO read properties from KnowRob
		String[] properties = new String[]{"http://ias.cs.tum.edu/kb/knowrob.owl#objectActedOn",
				"http://ias.cs.tum.edu/kb/knowrob.owl#reactionType",
				"http://ias.cs.tum.edu/kb/knowrob.owl#spokenText",
				"http://ias.cs.tum.edu/kb/knowrob.owl#queryText",
				"http://ias.cs.tum.edu/kb/knowrob.owl#toLocation",
				"http://ias.cs.tum.edu/kb/knowrob.owl#fromLocation",
				"http://ias.cs.tum.edu/kb/knowrob.owl#operatorType",
				"http://ias.cs.tum.edu/kb/knowrob.owl#bodyPartUsed",
				"http://ias.cs.tum.edu/kb/knowrob.owl#compressionLevel",
				"http://ias.cs.tum.edu/kb/knowrob.owl#downscaleFactor"};

		synchronized (props) {
			for(String property : properties) {
				propId2item.put(props_idx, props.addItem(property, props_idx).setText(OWLThing.getShortNameOfIRI(property)));
				props_idx++;
			}
		}
		
		value = controlP5.addTextfield("value", 230, 15, 160, 20).setAutoClear(true).moveTo(new_prop);

		controlP5.addButton("add property", 2, 230, 55, 33, 20).moveTo(new_prop).setCaptionLabel(" add");
		controlP5.addButton("remove property", 2, 280, 55, 50, 20).moveTo(new_prop).setCaptionLabel(" remove");
		
		action_props = controlP5.addListBox("Action properties", 20, 260, 400, listBoxHeight).setBarHeight(15).setItemHeight(20);
		GreyTheme.applyStyle(action_props, 20);

		actionpropId2item = new HashMap<Integer, ListBoxItem>();
		setCurrentActionPropItem(null);
		
		this.initialized   = true;
		
	}
	
	

	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	//  Main method
	// 
	
	
	public static void main(String args[]) {
		PApplet.main(new String[] { "edu.tum.cs.ias.knowrob.vis.applets.ActionPropertiesEditor" });
	}



}


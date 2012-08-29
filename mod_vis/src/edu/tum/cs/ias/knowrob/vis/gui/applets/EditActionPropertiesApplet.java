package edu.tum.cs.ias.knowrob.vis.gui.applets;

import java.awt.Color;
import java.awt.Frame;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import controlP5.Button;
import controlP5.ControlEvent;
import controlP5.ControlFont;
import controlP5.ControlGroup;
import controlP5.ControlP5;
import controlP5.ListBox;
import controlP5.ListBoxItem;
import controlP5.MultiList;
import controlP5.MultiListButton;
import controlP5.Textfield;
import edu.tum.cs.ias.knowrob.prolog.PrologInterface;
import edu.tum.cs.ias.knowrob.util.datastructures.Hashmap2List;
import edu.tum.cs.ias.knowrob.vis.actions.Action;


import processing.core.PApplet;
import processing.core.PFont;


public class EditActionPropertiesApplet  extends PApplet implements MouseListener, MouseMotionListener, IClassSelectionCallback {

	private static final long serialVersionUID = 7695328948788620463L;

	public ControlP5 controlP5;
	
	OwlClassSelectorApplet class_selector;
	
	/**
	 * Font for drawing text
	 */
	private PFont dejavuFont;

	
	private Button submit;
	private ListBox action_props;
	private ListBox props;

	private ListBoxItem currentActionProp;
	private ListBoxItem currentProp;

	private Map<Integer, ListBoxItem> propId2item;
	private Map<Integer, ListBoxItem> actionpropId2item;
	
	private int listBoxHeight = 20;

	private iAddActionCallback callback;
	
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
		//PrologInterface.initJPLProlog("ias_knowledge_base");
		background(40);
				
		initControlP5();
	}



	@Override
	public void draw() {

		background(40);
		controlP5.draw();
	}



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
		
		
		
		
		
		controlP5.addTextfield("value", 230, 15, 160, 20).setAutoClear(true).moveTo(new_prop);

		controlP5.addButton("add property", 2, 230, 55, 33, 20).moveTo(new_prop).setCaptionLabel(" add");
		controlP5.addButton("remove property", 2, 280, 55, 50, 20).moveTo(new_prop).setCaptionLabel(" remove");
		
		
		action_props = controlP5.addListBox("Action properties", 20, 260, 400, listBoxHeight).setBarHeight(15).setItemHeight(20);
		applyStyle(action_props);
		
		
		actionpropId2item = new HashMap<Integer, ListBoxItem>();
		setCurrentActionPropItem(null);
		
	}
	

	/**
	 * Apply the default styling to a ListBoxItem
	 * 
	 * @param b The item to be styled
	 * @return the same button instance with updated properties
	 */
	private ListBoxItem applyStyle(ListBoxItem b) {
		//b.getCaptionLabel().toUpperCase(false).setFont(dejavuFont);
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
//		lb.getCaptionLabel().toUpperCase(false).setFont(dejavuFont);
		
		return lb;
	}

	
	
	
	
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
	
	
	public void controlEvent(ControlEvent ev) {


		if(ev.isController()) {
			

			// open class selection dialog	
			if(ev.getController().getName().equals("select class")) {
				OWLClassSelect f = new OWLClassSelect();
				f.setClassSelectedCallback(this); 
			}


			// add action property
			if(ev.getController().getName().equals("add property") || ev.getController().getName().equals("value")) {
				addActionPropItem(propId2item.get((int)props.getValue()).getName() + " -- " + ((Textfield) controlP5.getController("value")).getText());
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
	
	
	
	@Override
	public void owlClassSelected(String identifier) {
		((Textfield) this.controlP5.getController("class")).setValue(identifier);		
	}

	
	public void setAddActionCallback(iAddActionCallback cb) {
		this.callback = cb;
	}
	
	
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
	
	
	public static void main(String args[]) {
		PApplet.main(new String[] { "edu.tum.cs.ias.knowrob.vis.gui.applets.EditActionPropertiesApplet" });
	}



}


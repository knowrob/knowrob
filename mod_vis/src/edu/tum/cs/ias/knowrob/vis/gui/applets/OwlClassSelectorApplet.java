package edu.tum.cs.ias.knowrob.vis.gui.applets;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import controlP5.ControlEvent;
import controlP5.ControlFont;
import controlP5.ControlP5;
import controlP5.Controller;
import controlP5.MultiList;
import controlP5.MultiListButton;


import processing.core.PApplet;
import processing.core.PFont;


public class OwlClassSelectorApplet  extends PApplet {

	private static final long serialVersionUID = 7695328948788620463L;

	/**
	 * Font for drawing text
	 */
	private PFont dejavuFont;
	private ControlFont cfont;

	public ControlP5 controlP5;

	MultiList l ;
	private ArrayList<ActionClass> actionclasses;
	private Map<Float, ActionClass> id2class;
	
	private ActionClass searchResult = null;
	

	@Override
	public void setup()
	{
		size(800, 400);

		if (this.frame != null)
		{
			this.frame.setTitle("OWL class selector");
			this.frame.setBackground(new Color(10, 10, 10));
		}

		dejavuFont = createFont("DejaVu Sans",12);
		textFont(dejavuFont);

		frameRate(10);

		id2class = new HashMap<Float, ActionClass>();
		actionclasses = new ArrayList<ActionClass>();
		readClassesFromOWL();
		
		
		initControlP5();
	}



	@Override
	public void draw() {

		background(40);

		controlP5.draw();
	}



	private void initControlP5() {

		controlP5 = new ControlP5(this);
		cfont = new ControlFont(dejavuFont);

		
		controlP5.addTextfield("search", 10, 10, 200, 20).setAutoClear(false).setFocus(true);
		
		l = applyStyle(controlP5.addMultiList("class list", 10, 60, 80, 17));


		float textwidth = textWidth("CheckingWhetherConditionObtains");
		l.setWidth((int)textwidth + 10);
		
		createActionButtons(actionclasses, l, 100);

	}
	

	public MultiListButton applyStyle(MultiListButton b) {
		b.getCaptionLabel().toUpperCase(false).setFont(cfont);
		b.setHeight(20);
		return b;
	}

	public MultiList applyStyle(MultiList lb) {

		controlP5.setColorForeground(color(180))
		.setColorCaptionLabel(color(240))
		.setColorBackground(color(80))
		.setColorActive(color(200));

		lb.setHeight(20);
		lb.getCaptionLabel().setFont(cfont);
		
		return lb;
	}

	
	
	public void controlEvent(ControlEvent ev) {
		
		if(ev.getController().getName().equals("search")) {
			searchActionClass(ev.getStringValue(), actionclasses);
			return;
		}
		
		if(ev.getController().getName().equals("class list")) {
			System.out.println("Selected class " + id2class.get(Float.valueOf(ev.getValue())).getIRI());
		}
	}

	
	
	
	/**
	 * Greedy search in the action class structure: return the first match and highlight 
	 * it in the GUI
	 * 
	 * @param stringValue
	 * @param actions
	 * @return
	 */
	private ActionClass searchActionClass(String stringValue, List<ActionClass> actions) {
		
		for(ActionClass a : actions) {
			
			if(a.iri.toLowerCase().contains(stringValue.toLowerCase())) {
				
				// reset highlight
				if(searchResult!=null)
					searchResult.getListButton().setColorBackground(80);
				
				// try to open the right tab
				ActionClass tmp = a;
				while(tmp.getParent()!=null) {
					tmp.getParent().getListButton().open();
					tmp = tmp.getParent();
				}
				
				
				// highlight the result
				a.getListButton().setColorBackground(color(180));
				searchResult = a;
				
				l.update();
				return a;
				
			} else if(a.label.toLowerCase().contains(stringValue.toLowerCase())) {
				a.getListButton().show();
				l.update();
				return a;
				
			} else if(a.subclasses.size()>0){
				
				ActionClass found = searchActionClass(stringValue, a.subclasses);
				
				if(found!=null) {
					return found;
				}
			}
		}
		return null;
	}



	/**
	 * 
	 * @param actions
	 * @param parent
	 * @param startIndex
	 * @return last index of created action
	 */
	private int createActionButtons(List<ActionClass> actions, Controller<?> parent, int startIndex) {
		
		int idx = startIndex;
		
		for(ActionClass act : actions) {
			
			MultiListButton b = null;
			if (parent instanceof MultiList) {
				b = applyStyle(((MultiList)parent).add(act.getLabel(),idx));
				act.setListButton(b);
				id2class.put((float)idx, act);
				idx++;
				
			} else if (parent instanceof MultiListButton) {
				b = applyStyle(((MultiListButton)parent).add(act.getLabel(),idx));
				act.setListButton(b);
				id2class.put((float)idx, act);
				idx++;
			} 
			
			if(act.getSubclasses().size()>0)
				idx = createActionButtons(act.subclasses, b, idx);
		}
		return idx;
	}

	
	
	
	public void readClassesFromOWL() {
		
		ActionClass cl = new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#ActionOnObject", "Object manipulation actions");
		actionclasses.add(cl);

		ActionClass sub = new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#ClosingSomething", "Closing something");
		cl.addSubclass(sub);
		sub.addSubclass(new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#CLosingADoor", "Closing a door"));
		sub.addSubclass(new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#ClosingADrawer", "Closing a drawer"));

		
		cl.addSubclass(new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#Collecting", "Collecting something"));
		cl.addSubclass(new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#ControllingSomething", "Controlling a device"));
		cl.addSubclass(new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#HoldingAnObject", "Holding an object"));
		cl.addSubclass(new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#Ingesting", "Ingesting"));
		cl.addSubclass(new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#Movement-TranslationEvent", "Movements and translations"));
		cl.addSubclass(new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#OpeningSomething", "Opening something"));
		cl.addSubclass(new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#PreparingFoodOrDrink", "Preparing food or drinks"));
		
		cl = new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#AvoidingSomething", "Actions to avoiding something");
		actionclasses.add(cl);
		cl.addSubclass(new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#AvoidingCollisions", "Avoid base collisions"));
		
		cl = new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#CheckingWhetherConditionObtains", "Check if condition holds");
		actionclasses.add(cl);
		
		cl = new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#LookingForAThingOrTypeOfThing", "Looking for objects");
		actionclasses.add(cl);
		
		cl = new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#MaterialHandling", "Handling materials");
		actionclasses.add(cl);
		
		cl = new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#Perceiving", "Perception actions");
		actionclasses.add(cl);
		cl.addSubclass(new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#Perceiving-Voluntary", "Active perception"));
		cl.addSubclass(new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#RfidPerception", "RFID tag-based perception"));
		cl.addSubclass(new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#TouchPerception", "Tactile perception"));
		cl.addSubclass(new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#VisualPerception", "Visual perception"));
		
		cl = new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#StandingStill", "Standing still (idle task)");
		actionclasses.add(cl);
		
		cl = new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#VoluntaryBodyMovement", "Body motions");
		actionclasses.add(cl);
		cl.addSubclass(new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#Reaching", "Reaching for something"));
		cl.addSubclass(new ActionClass("http://ias.cs.tum.edu/kb/knowrob.owl#ReleasingGraspOfSomething", "Releasing the grasp of something"));

	}
	
	
	
	public class ActionClass {
		
		String iri;
		String label;
		List<ActionClass> subclasses;
		
		MultiListButton button;
		ActionClass parent;
		
		public ActionClass(String iri, String label) {
			
			this.iri = iri;
			this.label = label;
			this.subclasses = new ArrayList<ActionClass>();
		}

		public String getIRI() {
			return iri;
		}

		public void setIri(String iri) {
			this.iri = iri;
		}

		public String getLabel() {
			return label;
		}

		public void setLabel(String label) {
			this.label = label;
		}
		
		public List<ActionClass> getSubclasses() {
			return subclasses;
		}
		
		public void addSubclass(ActionClass sub) {
			subclasses.add(sub);
			sub.setParent(this);
		}
		
		public void removeSubclass(ActionClass sub) {
			subclasses.remove(sub);
		}
		
		public void setListButton(MultiListButton b) {
			button = b;
		}
		
		public MultiListButton getListButton() {
			return button;
		}
		
		public void setParent(ActionClass p) {
			parent = p;
		}
		
		public ActionClass getParent() {
			return parent;
		}
		
		
	}
	
}


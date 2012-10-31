package edu.tum.cs.ias.knowrob.vis.applets;

import java.awt.Color;
import java.awt.Cursor;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Vector2f;

import controlP5.ControlEvent;
import controlP5.ControlP5;
import controlP5.Controller;
import controlP5.MultiList;
import controlP5.MultiListButton;
import controlP5.Textfield;
import edu.tum.cs.ias.knowrob.owl.OWLClass;
import edu.tum.cs.ias.knowrob.vis.themes.GreyTheme;

import processing.core.PApplet;


/**
 * GUI for selecting an OWL class using free-text search or by clicking through the hierarchy
 * 
 * @author Moritz Tenorth, tenorth@cs.tum.edu
 *
 */
public class OwlClassSelectorApplet  extends PApplet implements MouseListener, MouseMotionListener {

	private static final long serialVersionUID = 7695328948788620463L;
	

	/**
	 * ControlP5 GUI library instance
	 */
	public ControlP5 controlP5;

	/**
	 * GUI element holding the classes that can be selected
	 */
	MultiList class_listbox ;
	
	/**
	 * Position where mouse dragging started
	 */
	private Vector2f draggingStart;

	/**
	 * Normal cursor (arrow)
	 */
	private static final Cursor normalCursor = new Cursor(Cursor.DEFAULT_CURSOR);
	
	/**
	 * Move cursor
	 */
	private static final Cursor moveCursor = new Cursor(Cursor.MOVE_CURSOR);

	
	
	/**
	 * Hierarchical list of OWL classes
	 */
	private List<OWLClass> owl_classes;

	/**
	 * Mapping from a numeric ID to an OWLClass (needed to assign click events)
	 */
	private Map<Float, OWLClass> id2class;
	
	/**
	 * Mapping from the class IRI to a MultiListButton
	 */
	private Map<String, MultiListButton> act2button;
	
	/**
	 * Buffer for the result of searching for a class using the text field
	 */
	private OWLClass searchResult = null;

	/**
	 * Callback to be notified once class has been selected
	 */
	private IClassSelectionCallback cb;


	private Textfield search;

	
	/**
	 * Initialize the applet elements
	 */
	@Override
	public void setup()
	{
		size(800, 600);

		if (this.frame != null)
		{
			this.frame.setTitle("OWL class selector");
			this.frame.setBackground(new Color(10, 10, 10));
		}

		frameRate(10);
		
		id2class = new LinkedHashMap<Float, OWLClass>();
		act2button = new LinkedHashMap<String, MultiListButton>();
		
		
	}


	/** 
	 * Draw the applet elements
	 */
	@Override
	public void draw() {

		background(50);

		if(controlP5!=null) {
			synchronized(controlP5) {
				controlP5.draw();
			}
		}
	}



	public void setBaseClass(String base) {
		
		owl_classes = Collections.synchronizedList(new ArrayList<OWLClass>());

		OWLClass basecl = OWLClass.getOWLClass(base);
		basecl.readFromProlog();
		owl_classes.addAll(basecl.getSubclasses());

		initControlP5();
		
	}

	
	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	//  Searching for OWL classes
	// 
	
	
	/**
	 * Greedy search in the OWL class structure: return the first match and highlight 
	 * it in the GUI
	 * 
	 * @param stringValue
	 * @param owl_classes
	 * @return
	 */
	private OWLClass searchOWLClass(String stringValue, List<OWLClass> owl_classes) {
		
		synchronized (owl_classes) {
			for(OWLClass a : owl_classes) {
				
				if(a.getIRI().toLowerCase().contains(stringValue.toLowerCase())) {
					
					showSearchResult(a);
					return a;
					
				} else if(a.getLabel().toLowerCase().contains(stringValue.toLowerCase())) {

					showSearchResult(a);
					class_listbox.update();
					
					return a;
					
				} else if(a.getSubclasses().size()>0){
					
					OWLClass found = searchOWLClass(stringValue, a.getSubclasses());
					
					if(found!=null) {
						return found;
					}
				}
			}
		}

		return null;
	}



	private void showSearchResult(OWLClass a) {
		
		// reset highlight
		if(searchResult!=null && act2button.get(searchResult.getIRI())!=null)
			act2button.get(searchResult.getIRI()).setColorBackground(80);
		
		// try to open the right tab
		OWLClass tmp = a;
		while((tmp.getSuperClasses() != null) && 
			 (!tmp.getSuperClasses().isEmpty()) &&
			 (act2button.get(tmp.getSuperClasses().firstElement().getIRI()) !=null)) {
			
			act2button.get(tmp.getSuperClasses().firstElement().getIRI()).open();
			tmp = tmp.getSuperClasses().firstElement();
		}
		
		
		// highlight the result
		if(act2button!=null && act2button.get(a.getIRI())!=null)
			act2button.get(a.getIRI()).setColorBackground(color(180));
		searchResult = a;
		class_listbox.update();
	}

	

	
	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	//  Event handling
	// 
	

	public void controlEvent(ControlEvent ev) {
		
		// submit selection with ENTER if selection has been made, search otherwise
		if(ev.getController().getName().equals("search")) {

			if(searchResult!=null) {
				cb.owlClassSelected(searchResult.getIRI());
				this.frame.setVisible(false);
			} else {
				searchOWLClass(ev.getStringValue(), owl_classes);
			}
			return;
		}
		
		if(ev.getController().getName().equals("class list")) {
			
			if(this.cb!=null && id2class!=null) {
				cb.owlClassSelected(id2class.get(Float.valueOf(ev.getValue())).getIRI());
				this.frame.setVisible(false);
				
			} else {
				System.out.println("Selected class " + id2class.get(Float.valueOf(ev.getValue())).getIRI());
			}
		}
	}
	
	public void keyPressed(KeyEvent e) {
		
		// search in class structure with TAB
		if(e.getKeyCode() == KeyEvent.VK_TAB) {
			searchOWLClass(((Textfield) search).getText(), owl_classes);
			return;
		} else {
			controlP5.keyHandler.keyEvent(e, controlP5.controlWindow, true);
		}
	}
	
	@Override
	public void mouseDragged(MouseEvent e) {

		if (draggingStart != null) {

			this.class_listbox.updateLocation(e.getX() - draggingStart.x,
								  			  e.getY() - draggingStart.y);

			draggingStart.x = e.getX();
			draggingStart.y = e.getY();
		}
		
		// only forward left-clicks to ControlP5 
		// (no class selection by right click/dragging)
		if (e.getButton() == MouseEvent.BUTTON1) {
			controlP5.controlWindow.mouseEvent(e);
		}
	}

	@Override
	public void mousePressed(MouseEvent e) {

		if (e.getButton() == MouseEvent.BUTTON3) {

			draggingStart = new Vector2f(e.getX(),e.getY());
			setCursor(moveCursor);

		} else {
			setCursor(normalCursor);
		}
		
		// only forward left-clicks to ControlP5 
		// (no class selection by right click/dragging)
		if (e.getButton() == MouseEvent.BUTTON1) {
			controlP5.controlWindow.mouseEvent(e);
		}
	}

	@Override
	public void mouseReleased(MouseEvent e) {
		setCursor(normalCursor);
		draggingStart = null;
		
		// only forward left-clicks to ControlP5 
		// (no class selection by right click/dragging)
		if (e.getButton() == MouseEvent.BUTTON1) {
			controlP5.controlWindow.mouseEvent(e);
		}
	}

	public void mouseClicked(MouseEvent e) {
		
		// only forward left-clicks to ControlP5 
		// (no class selection by right click/dragging)
		if (e.getButton() == MouseEvent.BUTTON1) {
			controlP5.controlWindow.mouseEvent(e);
		}
	}

	public void setClassSelectedCallback(IClassSelectionCallback cb) {
		this.cb = cb;		
	}


	
	
	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	//  Initialize GUI elements
	// 

	/** 
	 * Initialize the GUI elements, including reading the available classes
	 */
	private void initControlP5() {

		controlP5 = new ControlP5(this);
		GreyTheme.applyStyle(controlP5);
		

		synchronized(controlP5) {
			
			search = controlP5.addTextfield("search", 20, 20, 200, 20).setAutoClear(false).setFocus(true);
			class_listbox = GreyTheme.applyStyle(controlP5.addMultiList("class list", 20, 60, 80, 17));
	
			float textwidth = textWidth("CheckingWhetherConditionObtains");
			class_listbox.setWidth((int)textwidth + 10);
			
			createListButtons(owl_classes, class_listbox, 0, 0);
		}
	}
	
	
	/**
	 * Recursively create the {@link MultiListButton}s for an {@link OWLClass} hierarchy
	 * 
	 * @param owl_classes {@link OWLClass} datastructure
	 * @param parent {@link Controller}, either a {@link MultiList} or a {@link MultiListButton}
	 * @param start_idx index for creating the continuously numbered buttons
	 * @return index of the last button created
	 */
	private int createListButtons(List<OWLClass> owl_classes, Controller<?> parent, int start_idx, int level) {

		int idx = start_idx;
		synchronized(owl_classes) {
			for(OWLClass act : owl_classes) {

				MultiListButton b = null;
				if (parent instanceof MultiList) {

					b = ((MultiList)parent).add(idx + "_" + act.getIRI(), idx++);
					configureButton(b, level, act);

				} else if (parent instanceof MultiListButton) {

					b = ((MultiListButton)parent).add(idx + "_" + act.getIRI(), idx++);
					configureButton(b, level, act);
				} 

				if(act.getSubclasses().size()>0)
					idx = createListButtons(act.getSubclasses(), b, idx, level+1);
			}
		}
		return idx;
	}


	
	/**
	 * Internal helper: set some properties of the create MultiListButton
	 * 
	 * @param b MultiListButton to be set up
	 * @param level Level in the MultiList hierarchy
	 * @param act OWL class associated with this button
	 */
	private void configureButton(MultiListButton b, int level, OWLClass act) {
		
		GreyTheme.applyStyle(b);
		b.setCaptionLabel(act.getLabel());
		
		if(level>=3)
			b.getCaptionLabel().setColor(color(10));
		
		if(act2button==null)
			act2button = new LinkedHashMap<String, MultiListButton>();
		
		act2button.put(act.getIRI(), b);
		
		if(id2class==null)
			id2class = new LinkedHashMap<Float, OWLClass>();
		
		id2class.put(b.getValue(), act);
	}

	
	
	
	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	//  OWL class datastructure
	// 



	

	public static void main(String args[]) {
		PApplet.main(new String[] { "edu.tum.cs.ias.knowrob.vis.applets.OwlClassSelectorApplet" });
	}




}


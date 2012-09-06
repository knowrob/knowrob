package edu.tum.cs.ias.knowrob.vis.gui.applets;

import java.awt.Color;
import java.awt.Cursor;
import java.awt.Frame;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Vector;

import javax.vecmath.Vector2f;

import controlP5.ControlEvent;
import controlP5.ControlP5;

import edu.tum.cs.ias.knowrob.owl.OWLThing;
import edu.tum.cs.ias.knowrob.prolog.PrologInterface;
import edu.tum.cs.ias.knowrob.vis.actions.Action;
import edu.tum.cs.ias.knowrob.vis.actions.ActionDrawInformation;
import edu.tum.cs.ias.knowrob.vis.actions.ActionSelectHistoryInfo;
import edu.tum.cs.ias.knowrob.vis.actions.ActionTransition;
import edu.tum.cs.ias.knowrob.vis.actions.ActionTransitions;

import processing.core.PApplet;
import processing.core.PFont;

/**
 * Visualization applet for action plans as sequences or state machines. 
 * @author Stefan Profanter, Moritz Tenorth
 * @see edu.tum.cs.ias.knowrob.vis.actions.Action
 */
public class PlanVisAppletFsm  extends PApplet implements MouseListener, MouseMotionListener, MouseWheelListener, KeyListener, iAddActionCallback {

	private static final long serialVersionUID = 7695328948788620463L;

	/**
	 * Font for drawing text
	 */
	private PFont dejavuFont;

	/**
	 * Currently selected action
	 */
	private Action currAction;
	
	/**
	 * Currently dragged action
	 */
	private Action draggedAction;

	/**
	 * Buffer storing the action from which a new transition is being 
	 * created (user clicked on its outbound connector)
	 */
	private Action newTransitionFromAction;
	
	private Vector2f newTransitionToLocation = new Vector2f();
	
	
	/**
	 * Actions in this task
	 */
	private List<Action> actions;

	/**
	 * Action transitions in this task
	 */
	private ActionTransitions transitions;
	
	/**
	 * Normal cursor (arrow)
	 */
	private static final Cursor normalCursor = new Cursor(Cursor.DEFAULT_CURSOR);
	
	/**
	 * Hand cursor
	 */
	private static final Cursor handCursor = new Cursor(Cursor.HAND_CURSOR);
	
	/**
	 * Move cursor
	 */
	private static final Cursor moveCursor = new Cursor(Cursor.MOVE_CURSOR);
	
	/**
	 * mouseClicked has a bug so that for each click it is called two times.
	 * Prevent it by time measure
	 */
	private long lastClickTime = 0;
	
	/**
	 * history of selected actions.
	 * It's used like a breadcrumbs menu. If you select an action it will be added to this arrray.
	 * If you go back to a certain action in the history all remaining actions after the selected will be removed from history.
	 */
	private ArrayList<ActionSelectHistoryInfo> clickHistory = new ArrayList<ActionSelectHistoryInfo>();
	
	/**
	 * Draw offset. Used when image moved/dragged.
	 */
	private Vector2f globalPosOffset = new Vector2f(0,0);
	/**
	 * Used in MouseDragging for calculating the dragging distance 
	 */
	private Vector2f draggingStart = new Vector2f(0,0);

	private ActionTransition activeTransition;

	
	public ControlP5 controlP5;

	
	/**
	 * Constructor
	 */
	public PlanVisAppletFsm()
	{
		this.redraw();
		this.actions = Collections.synchronizedList(new ArrayList<Action>());
		this.transitions = new ActionTransitions();
	}
	
	@Override
	public void setup()
	{
		size(1600, 600, P2D);
		if (this.frame != null)
		{
		    this.frame.setTitle("Action plans visualisation");
		    this.frame.setBackground(new Color(10, 10, 10));
		}
		
		addMouseMotionListener(this);
		addMouseListener(this);
		addMouseWheelListener(this);

	    dejavuFont = createFont("DejaVu Sans",13);
	    textFont(dejavuFont);
	    hint(ENABLE_ACCURATE_TEXTURES);
	    ellipseMode(RADIUS);
	    frameRate(25);
	    
	    initControlP5();
	    
	    drawActionsTreeLayout();
	    
	}
	
	@Override
	public void draw() {

		background(40);
		
		textFont(dejavuFont);
	    textMode(SCREEN);
	    
	    
    	drawActions();
	    
	    drawTransitions(this);
	    
    	drawActions();

	    drawHistory();
	    
	    if(this.newTransitionFromAction!=null) {
	    	arrowFromTo(this, newTransitionFromAction.getDrawInfo().getOutboundConnectorPos(), newTransitionToLocation, 5, -1);
	    }


	    controlP5.draw();
	}
	
	/**
	 * Load a plan with the given prolog identifier and all it's subactions.
	 * @param identifier Something like 'http://www.roboearth.org/kb/serve_drink.owl#ServeADrink'
	 * @return The main action initialized by the identifier
	 */
	private Action loadPrologPlanRecursive(String identifier)
	{	
		
		//Get the action name
		String name = "";
		try
		{
			HashMap<String, Vector<String>> qLabel = PrologInterface.executeQuery("rdf_has('"+identifier+"',rdfs:label,L),util:strip_literal_type(L,Label)");
			
			name = qLabel.get("Label").get(0);
			if (name.startsWith("'") && name.endsWith("'"))
			{
				name = name.substring(1,name.length()-1);
			}
		} catch (Exception e)
		{
			if (identifier.indexOf('#') >=0)
			{
				name = identifier.substring(identifier.lastIndexOf('#')+1);
			} else {
				name = identifier;
			}
		}

		Action ret = new Action(name, identifier);
		
		//get properties
		try
		{
			HashMap<String, Vector<String>> qProp = PrologInterface.executeQuery("class_properties('"+identifier+"', Key, V), util:strip_literal_type(V,Val)");

			if(qProp != null) {
				Vector<String> key = qProp.get("Key");
				Vector<String> val = qProp.get("Val");

				//Make sure each property is added only once, it may be that some properties are present two or more times
				HashSet<String> alreadyAdded = new HashSet<String>();
				if(key != null && val != null)
					for(int i=0;i<key.size() && i<val.size();i++) {
						if (alreadyAdded.contains(key.get(i)+val.get(i)))
							continue;
						alreadyAdded.add(key.get(i)+val.get(i));
						String k = OWLThing.getShortNameOfIRI(key.get(i));
						String v = OWLThing.getShortNameOfIRI(OWLThing.removeSingleQuotes(val.get(i)));

						if (k.compareToIgnoreCase("subAction") != 0)
							ret.setProperty(k, v);
					}
			}
		} catch (Exception e)
		{
			e.printStackTrace();
		}		

		//get subactions
		try {
			HashMap<String, Vector<String>> qSub = PrologInterface.executeQuery("plan_subevents('"+identifier+"',List)");

			if(qSub!=null) {

				Vector<String> list = qSub.get("List");

				//Add each action only once
				HashSet<String> alreadyAdded = new HashSet<String>();

				if(list != null) {
					for (Iterator<String> i = list.iterator(); i.hasNext();)
					{
						String o = i.next(); //Is array of strings

						for (String sArr[] : PrologInterface.dottedPairsToArrayList(o))
						{
							for (String s : sArr)
							{
								// TODO: create state machine from sub-actions if transition specs between them can be found
								
								if (alreadyAdded.contains(s))
									continue;
								alreadyAdded.add(s);
								ret.addSequence(loadPrologPlanRecursive(PrologInterface.removeSingleQuotes(s)));
							}
						}
					}
				}
			}
		} catch (Exception e)
		{
			e.printStackTrace();
		}		
		
		return ret;
	}
	
	/**
	 * Load a plan by starting with the given identifier and load all it's referenced actions recursively.
	 * @param identifier Something like 'http://www.roboearth.org/kb/serve_drink.owl#ServeADrink'
	 */
	public void loadPrologPlan(String identifier)
	{
		Action a = loadPrologPlanRecursive(identifier);
		if (a!= null)
			setMainAction(a);
	}	

	
	/**
	 * Load a plan by starting with the given identifier and load all it's referenced actions recursively.
	 * Display the results as a state machine structure
	 * 
	 * TODO: integrate with the other method for reading plans
	 * 
	 * @param identifier Something like 'http://www.roboearth.org/kb/serve_drink.owl#ServeADrink'
	 */
	public void loadPrologPlanFsm(String identifier)
	{

//		// read start action
//		HashMap<String, Vector<String>> qStart = PrologInterface.executeQuery("class_properties('"+identifier+"', knowrob:taskStartState, Start)");
//		
//		if(qStart!=null && qStart.get("Start").size()>0) {
//
//			Action a = loadPrologPlanRecursiveFsm(qStart.get("Start").firstElement());
//			
//			if (a!= null)
//				setMainAction(a);
//		}
		
	}
	
	/**
	 * Sets the action which should be selected at the beginning.
	 * If none set, nothing will be drawn.
	 * @param action action to set as start action
	 */
	public void setMainAction(Action action)
	{
		currAction = action;
		clickHistory.add(new ActionSelectHistoryInfo(currAction));
		updateHistoryPosition();
		
		transitions.setStartAction(action);
		
		this.redraw();
	}
	
	/**
	 * Draw an arrow between the given two points with current stroke and fill settings.
	 * @param applet Applet to draw on
	 * @param from Start position of arrow
	 * @param to End position of arrow
	 * @param lineWidth Width of arrow line (not the stroke).
	 */
	public static void arrowFromTo(PApplet applet, Vector2f from, Vector2f to, float lineWidth, float blockLength)
	{
		
		Vector<Vector2f> vertices = getArrowVertices(applet, from, to, lineWidth, blockLength);
		
		applet.beginShape();

		for(Vector2f p : vertices)
			applet.vertex(p.x,p.y);
				
		applet.endShape(CLOSE);
		
	}
	
	public static Vector<Vector2f> getArrowVertices(PApplet applet, Vector2f from, Vector2f to, float lineWidth, float blockLength) {
		
		Vector<Vector2f> vertices = new Vector<Vector2f>();
		
		Vector2f norm = new Vector2f(to.x - from.x, to.y-from.y);
		float len = norm.length();
		norm.normalize();
		Vector2f rot90 = new Vector2f(norm.x*(float)Math.cos(Math.PI/2f)-norm.y*(float)Math.sin(Math.PI/2f),norm.x*(float)Math.sin(Math.PI/2f)+norm.y*(float)Math.cos(Math.PI/2f));
		Vector2f distLeft = new Vector2f(rot90);
		distLeft.scale(-lineWidth);
		Vector2f distRight = new Vector2f(rot90);
		distRight.scale(lineWidth);
		
		//following points are described as if the arrow is pointing from left to right horizontally
		
		//upper left
		Vector2f p1 = new Vector2f(distLeft);
		p1.add(from);
		//bottom left
		Vector2f p7 = new Vector2f(distRight);
		p7.add(from);
		
		if (blockLength < 0)
		{
			blockLength = Math.max(len*0.5f, len - 3*lineWidth);
		}
		Vector2f transl = new Vector2f(norm);
		transl.scale(blockLength);
		
		//middle up on line
		Vector2f p2 = new Vector2f(distLeft);
		//upper tip
		Vector2f p3 = new Vector2f(distLeft);
		p3.scale(2);
		p2.add(transl);
		p2.add(from);
		p3.add(transl);
		p3.add(from);
		
		//middle bottom on line
		Vector2f p6 = new Vector2f(distRight);
		//bottom tip
		Vector2f p5 = new Vector2f(distRight);
		p5.scale(2);
		p6.add(transl);
		p6.add(from);
		p5.add(transl);
		p5.add(from);
		
		vertices.add(p1);	vertices.add(p2);	vertices.add(p3);	vertices.add(to);
		vertices.add(p5);	vertices.add(p6);	vertices.add(p7);
		
		return vertices;
	}
	
	
	/**
	 * Draw an arrow pointing right at given position. 
	 * @param applet Applet to draw on
	 * @param x x position of the arrow bounding box.
	 * @param y y position of the arrow bounding box
	 * @param width width of the arrow
	 * @param height height of the arrow
	 */
	public static void arrow(PApplet applet, float x, float y, float width, float height)
	{
		applet.beginShape();
		
		float indentY = 2f/7f*height;
		float indentX = 4f/9f*width;
		
		applet.vertex(x,y+indentY);
		applet.vertex(x+indentX,y+indentY);
		applet.vertex(x+indentX,y);
		applet.vertex(x+width,y+height/2f);
		applet.vertex(x+indentX,y+height);
		applet.vertex(x+indentX,y+height-indentY);
		applet.vertex(x,y+height-indentY);
				
		applet.endShape(CLOSE);
	}
	
	/**
	 * Update position for each ActionSelectHistoryInfo object in the clickHistory array.
	 * Needed for example if a new history object is added/deleted.
	 */
	private void updateHistoryPosition()
	{
		float fullWidth = 0;
		for (int i= clickHistory.size()-1; i>=0; i--)
		{
			fullWidth += clickHistory.get(i).getDimension().x;
		}
		if (fullWidth+50 > this.getSize().width)
		{
			//There is not enough space for all history, so begin from right to left
			float prevX = this.getSize().width-50;
			for (int i= clickHistory.size()-1; i>=0; i--)
			{
				float newX = prevX - clickHistory.get(i).getDimension().x;
				prevX = newX;
				clickHistory.get(i).setPosition(newX, 0, i==clickHistory.size()-1);
			}
		} else {
			//Enough space, begin from left
			float currX = 0;
			for (int i= 0; i<clickHistory.size(); i++)
			{
				clickHistory.get(i).setPosition(currX, 0, i==clickHistory.size()-1);
				currX += clickHistory.get(i).getDimension().x;	
			}
		}
	}
	
	/**
	 * Draw the current history in the upper area of the window
	 */
	private void drawHistory()
	{
		if(clickHistory.size()>=1) {
			for (int i= clickHistory.size()-1; i>=0; i--)
			{
				clickHistory.get(i).Draw(this);
			}
		}
	}
	
	/**
	 * Draw the current selected action
	 */
	private void drawActionsTreeLayout() {

		Vector2f currentPosition = new Vector2f(50+globalPosOffset.x,80+globalPosOffset.y);
		Map<Integer, Vector<Action>> levels = transitions.getTreeLevels();
		Map<Integer, Float> levelWidths = transitions.getLevelWidths();
		
		if(levels.size()>0) {
			
			// compute horizontal center as half of the maximum level width
			float center_x = currentPosition.x + Collections.max(levelWidths.values())/2;

			for(int level : levels.keySet()) {

				// left corner of the first action is maxWidth - curWidth
				currentPosition.x = center_x - levelWidths.get(level)/2;


				float maxHeight = 0.0f;
				for(Action a : levels.get(level)) {

					if(a.isExpanded()) {
						a.getDrawInfo().drawSimpleBox(this, currentPosition, globalPosOffset, 10, true);
					} else {
						a.getDrawInfo().drawSimpleBox(this, currentPosition, globalPosOffset, 10, false);
					}

					// increase x-position after each action on this level
					currentPosition.x += a.getDrawInfo().getSimpleBoxDimension().x + ActionDrawInformation.SEQUENCE_BOX_PADDING;

					// remember max. action height per level to know where to start the next one
					maxHeight = Math.max(maxHeight, a.getDrawInfo().getSimpleBoxDimension().y);
				}

				// increase y-coordinate for the next level
				currentPosition.y += maxHeight + 2*ActionDrawInformation.MAIN_BOX_PADDING;
			}
		}
		
		
		// draw all actions that are currently not connected in the upper left corner
		currentPosition = new Vector2f(50+globalPosOffset.x,80+globalPosOffset.y);
		synchronized(actions) {
			for(Action a : actions) {

				if( transitions.getTransitionsFrom(a).isEmpty() &&
						transitions.getTransitionsTo(a).isEmpty()) {

					if(a.isExpanded()) {
						a.getDrawInfo().drawSimpleBox(this, currentPosition, globalPosOffset, 10, true);
					} else {
						a.getDrawInfo().drawSimpleBox(this, currentPosition, globalPosOffset, 10, false);
					}

				}
			}
		}
	}
	
	
	private void drawActions() {

		// draw  actions wherever they have been drawn before (don't re-arrange layout)
		synchronized(actions) {
			for(Action a : actions) {

				if(a.isExpanded()) {
					a.getDrawInfo().drawSimpleBox(this, a.getDrawInfo().position, globalPosOffset, 10, true);
				} else {
					a.getDrawInfo().drawSimpleBox(this, a.getDrawInfo().position, globalPosOffset, 10, false);
				}
			}
		}
	}

	
	public synchronized void drawTransitions(PApplet app) {

		synchronized(transitions) {
			// draw transition arrows
			for(ActionTransition t : transitions) {
				t.drawConnection(app);
			}
		}
	}
	

	private void initControlP5() {
		
	    controlP5 = new ControlP5(this);
		controlP5.setColorForeground(color(180))
		.setColorCaptionLabel(color(240))
		.setColorBackground(color(80))
		.setColorActive(color(200));
		
	    controlP5.addButton("add action", 1, 700, 20, 80, 20);
	}
	
	
	
	@Override
	public void mouseMoved(MouseEvent e) {
		
		if(newTransitionFromAction!=null) {
			newTransitionToLocation.x = e.getX();
			newTransitionToLocation.y = e.getY();
			redraw();
		}
		
		if (getHistoryHover(e.getX(), e.getY())>=0)
		{
			setCursor(handCursor);
			return;
		}
		synchronized(actions) {
			for(Action a : actions) {

				if (a.getDrawInfo().updateHover(e.getX(), e.getY()))
					setCursor(handCursor);
				else
					setCursor(normalCursor);
			}
		}
		
		if (currAction ==null)
		{
			setCursor(normalCursor);
			return;
		}

		// only send left-button events to contolP5
		if(controlP5!=null)
			controlP5.controlWindow.mouseEvent(e);
    }

	@Override
    public void mouseDragged(MouseEvent e) {

		
		if (draggingStart != null) {
			
			// check if we clicked on an action -> move only that action box
			if(draggedAction!=null) {

				draggedAction.getDrawInfo().localPosOffset.x += e.getX() - draggingStart.x;
				draggedAction.getDrawInfo().localPosOffset.y += e.getY() - draggingStart.y;

				draggingStart.x = e.getX();
				draggingStart.y = e.getY();
				return;
			}

			// no action wanted the drag event, so let's move the whole canvas:
			globalPosOffset.x += e.getX() - draggingStart.x;
			globalPosOffset.y += e.getY() - draggingStart.y;

			draggingStart.x = e.getX();
			draggingStart.y = e.getY();
		}
		

		// only send left-button events to contolP5
		if(controlP5!=null && e.getButton() == MouseEvent.BUTTON1)
			controlP5.controlWindow.mouseEvent(e);
    }
	
	@Override
	public void mousePressed(MouseEvent e) {
		
		if (e.getButton() == MouseEvent.BUTTON1) {

			// check if we clicked on an action -> move only that action box
			synchronized(actions) {
				for(Action a : actions) {
					if(a.getDrawInfo().checkClick(e.getX(), e.getY())!=null) {
						draggedAction = a;

						draggingStart = new Vector2f(e.getX(),e.getY());
						setCursor(moveCursor);
					}
				}
			}
			
			// only send left-button events to contolP5
			if(controlP5!=null && e.getButton() == MouseEvent.BUTTON1)
				controlP5.controlWindow.mouseEvent(e);
			
		} else if (e.getButton() == MouseEvent.BUTTON3) {
			
			draggingStart = new Vector2f(e.getX(),e.getY());
			setCursor(moveCursor);
			
		} else {
			setCursor(normalCursor);
		}

		// only send left-button events to contolP5
		if(controlP5!=null && e.getButton() == MouseEvent.BUTTON1)
			controlP5.controlWindow.mouseEvent(e);
    }

	@Override
    public void mouseReleased(MouseEvent e) {
		setCursor(normalCursor);
		draggingStart = null;
		draggedAction = null;


		// only send left-button events to contolP5
		if(controlP5!=null && e.getButton() == MouseEvent.BUTTON1)
			controlP5.controlWindow.mouseEvent(e);
    }
	
	@Override
    public void mouseClicked(MouseEvent e) {
		
		
		long diff = System.currentTimeMillis()-lastClickTime;
		lastClickTime = System.currentTimeMillis();
		
		if (diff < 10) //double fired event
			return;

		
		// only send left-button events to contolP5
		if(controlP5!=null && e.getButton() == MouseEvent.BUTTON1)
			controlP5.controlWindow.mouseEvent(e);
		
		
		if (e.getButton() == MouseEvent.BUTTON1) {

			
			// Check if clicked on outbound connector
			synchronized(actions) {
				for(Action a : actions) {

					Action clicked_on = a.getDrawInfo().checkPosOverOutboundConnector(e.getX(), e.getY());
					if (clicked_on!= null) {

						// clicked nowhere special, reset highlight
						clearHighlight();
						currAction=null;
						if(activeTransition!=null)
							activeTransition.setActive(false);
						activeTransition = null;

						newTransitionFromAction = clicked_on;
						newTransitionToLocation.x = e.getX();
						newTransitionToLocation.y = e.getY();
						return;
					}
				}
			}
			
			// Check if clicked on inbound connector in connection mode
			if(newTransitionFromAction!=null) {

				synchronized(actions) {
					for(Action a : actions) {

						//Action clicked_on = a.getDrawInfo().checkPosOverInboundConnector(e.getX(), e.getY());

						if (a.getDrawInfo().checkHover(e.getX(), e.getY(), null)!= null) {

							// clicked nowhere special, reset highlight
							clearHighlight();
							currAction=null;
							if(activeTransition!=null)
								activeTransition.setActive(false);
							activeTransition = null;

							transitions.add(new ActionTransition(newTransitionFromAction, a, "OK"));
							redraw();

							newTransitionFromAction = null;
							newTransitionToLocation = new Vector2f();
							return;
						}
					}
				}
			}
			
			// reset connection mode if clicked elsewhere
			newTransitionFromAction = null;
			newTransitionToLocation = new Vector2f();
			
// TODO: adapt to multi-action case and reactivate			
//			//Check if clicked on history element
//			int idx = getHistoryHover(e.getX(), e.getY());
//			if (idx >= 0) {
//				currAction = clickHistory.get(idx).getAction();
//				for (int i=clickHistory.size()-1; i>idx; i--)
//				{
//					clickHistory.remove(i);
//				}
//				updateHistoryPosition();
//				return;
//			}
//			
			
// TODO: adapt to multi-action case and reactivate			
//			//Check if clicked on an expand button
//			for(Action a : actions) {
//				
//				Action clicked_on = a.getDrawInfo().checkClickExpand(e.getX(), e.getY());
//				if (clicked_on!= null) {
//					
//					clicked_on.toggleExpand();
//					currAction.getDrawInfo().notifyModified();
//					return;
//				}
//			}
			
			
			// activate transition arrow after click

			synchronized(transitions) {
				for(ActionTransition t : transitions) {

					boolean inside = t.checkPosInArrow(new Vector2f(e.getX(), e.getY()), this);

					if (inside) {

						// deactivate transition with second click
						if(t.isActive()) {
							t.setActive(false);
							activeTransition = null;
						}

						// reset action selection
						clearHighlight();
						currAction=null;

						// reset previously active transition, set to t 
						t.setActive(true);

						if(activeTransition!=null)
							activeTransition.setActive(false);

						activeTransition = t;

						return;
					}
				}
			}
			
			synchronized(actions) {
				for(Action a : actions) {

					Action clicked_on  = a.getDrawInfo().checkClick(e.getX(), e.getY());

					if (clicked_on!= null && clicked_on != currAction) {

						clearHighlight();
						if(activeTransition!=null)
							activeTransition.setActive(false);
						activeTransition = null;

						// edit action if double-clicked
						if(e.getClickCount()==2) {

							// open action editor
							ActionEditorWindow f = new ActionEditorWindow();

							f.setAddActionCallback(this);

							while(!f.applet.isInitialized()) {
								try {Thread.sleep(100);
								} catch (InterruptedException e1) {e1.printStackTrace(); }
							}
							f.applet.setIdentifier(a.getIdentifier());
							f.applet.setActionClass(a.getProperties().get("type").get(0));
							f.applet.setActionProperties(a.getProperties());
							f.applet.setEditing(true);
							// TODO: store superclass in action data structure -> make Action extend OWLClass?
							return;

						} else {

							// activate action after single click
							currAction = clicked_on;
							highlightAction(currAction, false);

							return;
						}
					}
				}
			}
				
			// clicked nowhere special, reset highlight
			clearHighlight();
			currAction=null;
			if(activeTransition!=null)
				activeTransition.setActive(false);
			activeTransition = null;
		}
    }
	

	@Override
    public void mouseEntered(MouseEvent e) {
		
		if(controlP5!=null)
			controlP5.controlWindow.mouseEvent(e);
    }

	@Override
    public void mouseExited(MouseEvent e) {

		if(controlP5!=null)
			controlP5.controlWindow.mouseEvent(e);
    }


	@Override
	public void mouseWheelMoved(MouseWheelEvent e) {
//			int totalScrollAmount = 0;
//			if (e.getScrollType() == MouseWheelEvent.WHEEL_UNIT_SCROLL)
//			{
//				totalScrollAmount = e.getUnitsToScroll() * e.getScrollAmount();
//			} else {
//				totalScrollAmount = e.getWheelRotation() * 30;
//			}
//			draggingStart.y += totalScrollAmount;
		controlP5.controlWindow.mouseEvent(e);
	}
	
	
	
	public void keyPressed(KeyEvent e) {
		
		if(e.getKeyCode() == KeyEvent.VK_DELETE) {
			
			synchronized(transitions) {
				
				if(activeTransition!=null) {
					transitions.remove(activeTransition);
					this.activeTransition = null;
				}

				if(currAction!=null) {

					List<ActionTransition> to_be_removed = new ArrayList<ActionTransition>();

					for(ActionTransition t : transitions) {
						if(t.getFrom().equals(currAction) || 
								t.getTo().equals(currAction)) {

							to_be_removed.add(t);
						} 
					}

					transitions.removeAll(to_be_removed);

					synchronized(actions) {
						actions.remove(currAction);
					}
					this.currAction = null;
				}
			}
		}
		
	}
	
	public void controlEvent(ControlEvent ev) {

		if(ev.isController()) {

			// open action creation dialog	
			if(ev.getController().getName().equals("add action")) {
				
				System.out.println("add action event");
				
				ActionEditorWindow f = new ActionEditorWindow();
				f.setAddActionCallback(this); 
			}
		}
	}
	
	
	
	/**
	 * Get index of history object under current mouse position.
	 * If mouse isn't on history, -1 will be returned.
	 * @param x mouse x position
	 * @param y mouse y position
	 * @return index of history object in clickHistory array or -1 if none
	 */
	private int getHistoryHover(float x, float y)
	{
		int idx = -1;
		for (int i= 0; i<clickHistory.size()-1; i++)
		{
			if (idx >= 0)
			{
				clickHistory.get(i).setHover(false);
			} else {
				if (clickHistory.get(i).checkHover(x, y))
				{
					idx = i;
				}
			}
		}
		return idx;
	}
	
	/**
	 * Highlight the action referenced by <code>a</code>
	 * @param a action to highlight
	 * @param expand Expand all the parents of <code>a</code> so that <code>a</code> is visible
	 * @return true if action found and highlighted
	 */
	public boolean highlightAction(Action a, boolean expand)
	{
		if (currAction == null)
			return false;
		currAction.getDrawInfo().clearHightlight();
		return currAction.getDrawInfo().highlightSubsequence(a,expand);
	}
	
	/**
	 * Highlight the action referenced by <code>identifier</code>
	 * @param identifier action identified by <code>identifier</code> to highlight
	 * @param expand Expand all the parents of <code>identifier</code> so that <code>identifier</code> is visible
	 * @return true if action found and highlighted
	 */
	public boolean highlightAction(String identifier, boolean expand)
	{
		if (currAction == null)
			return false;
		currAction.getDrawInfo().clearHightlight();
		return currAction.getDrawInfo().highlightSubsequence(identifier,expand);
	}
	
	/**
	 * Clear all highlighted actions so that none is highlighted.
	 */
	public void clearHighlight()
	{
		if (currAction == null)
			return;
		currAction.getDrawInfo().clearHightlight();
	}

	
	public void addAction(Action a) {
		
		synchronized(actions) {
			this.actions.add(a);
		}
		a.getDrawInfo().position = new Vector2f(50+globalPosOffset.x,80+globalPosOffset.y);
	}
	
	
	public void updateAction(Action a) {
		
		synchronized(actions) {
			for(Action old : actions) {
				if(old.getIdentifier().equals(a.getIdentifier())) {
					old.setName(a.getName());
					old.setProperties(a.getProperties());
				}
			}
		}
	}
	
	public void addTransition(ActionTransition t) {
		this.transitions.add(t);
	}
	
	
	
	/**
	 * Wrapper class around an action editor applet in a new window
	 *  
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 */
	public class ActionEditorWindow extends Frame {

		private static final long serialVersionUID = 543157068719461737L;
		public EditActionPropertiesApplet applet;
		
		public ActionEditorWindow() {
				PrologInterface.initJPLProlog("ias_knowledge_base");
			
	        setBounds(100,100,500,500);
	        applet = new EditActionPropertiesApplet();
	        applet.frame = this;
	        add(applet);
	        applet.init();
			this.setVisible(true);
	    }

		public void setAddActionCallback(iAddActionCallback cb) {
			applet.setAddActionCallback(cb); 
		}
	}
	
}

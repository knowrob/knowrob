package edu.tum.cs.ias.knowrob.vis.gui.applets;

import java.awt.Color;
import java.awt.Cursor;
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
public class PlanVisAppletFsm  extends PApplet implements MouseListener, MouseMotionListener, MouseWheelListener, KeyListener {

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
	private Vector2f drawOffset = new Vector2f(0,0);
	/**
	 * Used in MouseDragging for calculating the dragging distance 
	 */
	private Vector2f draggingStart = new Vector2f(0,0);

	private ActionTransition activeTransition;

	
	/**
	 * Constructor
	 */
	public PlanVisAppletFsm()
	{
		this.redraw();
		this.actions = new ArrayList<Action>();
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
	}
	
	@Override
	public void draw() {

		background(40);
		
		textFont(dejavuFont);
	    textMode(SCREEN);
	    	    

	    // hacky solution to draw the action boxes on top of the arrows...
	    drawActions();
	    drawTransitions();
	    drawActions();
	    
	    
	    drawHistory();
	    
	    if(this.newTransitionFromAction!=null) {
	    	arrowFromTo(this, newTransitionFromAction.getDrawInfo().getOutboundConnectorPos(), newTransitionToLocation, 5, -1);
	    }
	    
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
						String k = PrologInterface.valueFromIRI(key.get(i));
						String v = PrologInterface.valueFromIRI(PrologInterface.removeSingleQuotes(val.get(i)));

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
	private void drawActions() {

		Vector2f currentPosition = new Vector2f(50+drawOffset.x,80+drawOffset.y);
		Map<Integer, Vector<Action>> levels = transitions.getTreeLevels();
		Map<Integer, Float> levelWidths = transitions.getLevelWidths();
		
		// compute horizontal center as half of the maximum level width
		float center_x = currentPosition.x + Collections.max(levelWidths.values())/2;
		
		for(int level : levels.keySet()) {
			
			// left corner of the first action is maxWidth - curWidth
			currentPosition.x = center_x - levelWidths.get(level)/2;
			
			
			float maxHeight = 0.0f;
			for(Action a : levels.get(level)) {
				
				if(a.isExpanded()) {
					a.getDrawInfo().drawSimpleBox(this, currentPosition, 10, true);
				} else {
					a.getDrawInfo().drawSimpleBox(this, currentPosition, 10, false);
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
	
	
	private void drawTransitions() {

		// draw transition arrows
		for(ActionTransition t : transitions) {
			t.drawConnection(this);
		}
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
		
		if (currAction ==null)
		{
			setCursor(normalCursor);
			return;
		}
		for(Action a : actions) {
			
			if (a.getDrawInfo().updateHover(e.getX(), e.getY()))
				setCursor(handCursor);
			else
				setCursor(normalCursor);
		}
    }

	@Override
    public void mouseDragged(MouseEvent e) {

		
		if (draggingStart != null) {
			
			// check if we clicked on an action -> move only that action box
			if(draggedAction!=null) {

				draggedAction.getDrawInfo().positionOffset.x += e.getX() - draggingStart.x;
				draggedAction.getDrawInfo().positionOffset.y += e.getY() - draggingStart.y;

				draggingStart.x = e.getX();
				draggingStart.y = e.getY();
				return;
			}

			// no action wanted the drag event, so let's move the whole canvas:
			drawOffset.x = drawOffset.x+ e.getX() - draggingStart.x;
			drawOffset.y = drawOffset.y + e.getY() - draggingStart.y;

			draggingStart.x = e.getX();
			draggingStart.y = e.getY();
		}
    }
	
	@Override
	public void mousePressed(MouseEvent e) {
		
		if (e.getButton() == MouseEvent.BUTTON1) {

			// check if we clicked on an action -> move only that action box
			for(Action a : actions) {
				if(a.getDrawInfo().checkClick(e.getX(), e.getY())!=null) {
					draggedAction = a;

					draggingStart = new Vector2f(e.getX(),e.getY());
					setCursor(moveCursor);
				}
			}
			
		} else if (e.getButton() == MouseEvent.BUTTON3) {
			
			draggingStart = new Vector2f(e.getX(),e.getY());
			setCursor(moveCursor);
			
		} else {
			setCursor(normalCursor);
		}
    }

	@Override
    public void mouseReleased(MouseEvent e) {
		setCursor(normalCursor);
		draggingStart = null;
		draggedAction = null;
    }

	@Override
    public void mouseClicked(MouseEvent e) {
		
		
		long diff = System.currentTimeMillis()-lastClickTime;
		lastClickTime = System.currentTimeMillis();
		
		if (diff < 10) //double fired event
			return;
		
		
		
		if (e.getButton() == MouseEvent.BUTTON1 && currAction != null) {

			// Check if clicked on outbound connector
			for(Action a : actions) {
				
				Action clicked_on = a.getDrawInfo().checkPosOverOutboundConnector(e.getX(), e.getY());
				if (clicked_on!= null) {
					
					newTransitionFromAction = clicked_on;
					newTransitionToLocation.x = e.getX();
					newTransitionToLocation.y = e.getY();
					return;
				}
			}
			
			// Check if clicked on inbound connector in connection mode
			if(newTransitionFromAction!=null) {
				for(Action a : actions) {
						
					//Action clicked_on = a.getDrawInfo().checkPosOverInboundConnector(e.getX(), e.getY());

					if (a.getDrawInfo().checkHover(e.getX(), e.getY(), null)!= null) {
						
						transitions.add(new ActionTransition(newTransitionFromAction, a, "OK"));
						redraw();
						
						newTransitionFromAction = null;
						newTransitionToLocation = new Vector2f();
						return;
					}
				}
			}
			
			// reset connection mode if clicked elsewhere
			newTransitionFromAction = null;
			newTransitionToLocation = new Vector2f();
			
			
			//Check if clicked on history element
			int idx = getHistoryHover(e.getX(), e.getY());
			if (idx >= 0) {
				currAction = clickHistory.get(idx).getAction();
				for (int i=clickHistory.size()-1; i>idx; i--)
				{
					clickHistory.remove(i);
				}
				updateHistoryPosition();
				return;
			}
			
			
			//Check if clicked on an expand button
			for(Action a : actions) {
				
				Action clicked_on = a.getDrawInfo().checkClickExpand(e.getX(), e.getY());
				if (clicked_on!= null) {
					
					clicked_on.toggleExpand();
					currAction.getDrawInfo().notifyModified();
					return;
				}
			}
			
			
			//Check if clicked on a transition arrow
			for(ActionTransition t : transitions) {
				
				boolean inside = t.checkPosInArrow(new Vector2f(e.getX(), e.getY()), this);
				
				if (inside) {

					// deactivate transition with second click
					if(t.isActive()) {
						t.setActive(false);
						activeTransition = null;
					}

					// reset previously active transition, set to t 
					t.setActive(true);
					
					if(activeTransition!=null)
						activeTransition.setActive(false);
					
					activeTransition = t;
					
					return;
				}
			}
			
//		
// TODO: expand the action?
//
//			//Check if clicked on an action
//			for(Action a : actions) {
//
//				Action clicked_on  = a.getDrawInfo().checkClick(e.getX(), e.getY());
//				if (clicked_on!= null && clicked_on != currAction)
//				{
//					currAction = clicked_on;
//					if (clickHistory.size()>1 && clickHistory.get(clickHistory.size()-2).getAction()==a)
//						clickHistory.remove(clickHistory.size()-1);
//					else
//						clickHistory.add(new ActionSelectHistoryInfo(currAction));
//					updateHistoryPosition();
//				}
//			}
			
		}
    }
	

	@Override
    public void mouseEntered(MouseEvent e) {
    }

	@Override
    public void mouseExited(MouseEvent e) {
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
	}
	
	
	
	public void keyPressed(KeyEvent e) {
		
		if(e.getKeyCode() == KeyEvent.VK_DELETE) {
			if(activeTransition!=null) {
				transitions.remove(activeTransition);
				this.activeTransition = null;
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
		this.actions.add(a);
	}
	
	public void addTransition(ActionTransition t) {
		this.transitions.add(t);
	}
}

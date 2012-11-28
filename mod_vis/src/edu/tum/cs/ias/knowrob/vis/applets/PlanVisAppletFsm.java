package edu.tum.cs.ias.knowrob.vis.applets;

import java.awt.Color;
import java.awt.Container;
import java.awt.Cursor;
import java.awt.Frame;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Vector;

import javax.vecmath.Vector2f;

import controlP5.ControlEvent;
import controlP5.ControlP5;
import controlP5.Textfield;

import edu.tum.cs.ias.knowrob.owl.OWLIndividual;
import edu.tum.cs.ias.knowrob.owl.OWLThing;
import edu.tum.cs.ias.knowrob.prolog.PrologInterface;
import edu.tum.cs.ias.knowrob.vis.actions.Action;
import edu.tum.cs.ias.knowrob.vis.actions.ActionDrawInformation;
import edu.tum.cs.ias.knowrob.vis.actions.ActionDrawInformation.HighlightType;
import edu.tum.cs.ias.knowrob.vis.actions.ActionSelectHistoryInfo;
import edu.tum.cs.ias.knowrob.vis.actions.ActionTransition;
import edu.tum.cs.ias.knowrob.vis.actions.ActionTransitions;
import edu.tum.cs.ias.knowrob.vis.themes.GreyTheme;

import processing.core.PApplet;
import processing.core.PFont;

/**
 * Visualization applet for action plans as sequences or state machines. 
 * @author Stefan Profanter, Moritz Tenorth
 * @see edu.tum.cs.ias.knowrob.vis.actions.Action
 */
public class PlanVisAppletFsm  extends PApplet implements MouseListener, MouseMotionListener, KeyListener, IAddActionCallback {

	private static final long serialVersionUID = 7695328948788620463L;

	/**
	 * Font for drawing text
	 */
	private PFont dejavuFont;

	/**
	 * Currently selected action
	 */
	private Action selectedAction;
	
	/**
	 * Currently loaded task structure
	 */
	private Action currTask = null;
	

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
	
	private Textfield base_iri = null;
	private Textfield new_recipe_shortname = null;
	private Textfield new_recipe_label = null;
	private Textfield start_action;
	private Textfield end_action;

	private boolean select_start = false;
	private boolean select_end = false;
	
	
	@Override
	public void setup()
	{
		size(1200, 600, P2D);
		
		addMouseMotionListener(this);
		addMouseListener(this);

	    dejavuFont = createFont("DejaVu Sans",13);
	    textFont(dejavuFont);
	    hint(ENABLE_ACCURATE_TEXTURES);
	    ellipseMode(RADIUS);
	    frameRate(15);
	    
	    initControlP5();
		
		if (this.frame != null) {
		    this.frame.setTitle("Action plans visualisation");
		    this.frame.setBackground(new Color(10, 10, 10));
		}
	}
	
	@Override
	public void draw() {

		if(this.frame==null)
			return;
		
		background(40);
		
		textFont(dejavuFont);
	    textMode(SCREEN);
	    
	    if(currTask != null) {
	    	drawActions();

	    	drawTransitions(this);

	    	drawActions();

	    	drawHistory();

	    	if(this.newTransitionFromAction!=null) {
	    		arrowFromTo(this, newTransitionFromAction.getDrawInfo().getOutboundConnectorPos(), newTransitionToLocation, 5, -1);
	    	}
	    }
	    controlP5.draw();
	}
	
	
	/**
	 * Load a plan by starting with the given identifier and load all it's referenced actions recursively.
	 * 
	 * @param identifier Something like 'http://www.roboearth.org/kb/serve_drink.owl#ServeADrink'
	 */
	public void loadPrologPlan(String identifier) {
				
		Action a = Action.getAction(identifier);
		a.readFromProlog();
		a.getDrawInfo().recalculateDimensions(this);
		
		if (a!= null)
			setTask(a);
		
		// set base URL text field
		base_iri.setText(a.getIRIPrefix()+"#");
		
		Vector<String> start = a.getHasValue().get("http://ias.cs.tum.edu/kb/knowrob.owl#taskStartState");
		if(start!=null && !start.isEmpty()) {
			start_action.setText(OWLThing.getShortNameOfIRI(start.firstElement()));
		}
		
		Vector<String> end = a.getHasValue().get("http://ias.cs.tum.edu/kb/knowrob.owl#taskEndState");
		if(end!=null && !end.isEmpty()) {
			end_action.setText(OWLThing.getShortNameOfIRI(end.firstElement()));
		}
	}
	
	
	/**
	 * Sets the action which should be selected at the beginning.
	 * If none set, nothing will be drawn.
	 * @param action action to set as start action
	 */
	public void setTask(Action action)
	{
		currTask = action;
		selectedAction = null;
		clickHistory.clear();
		clickHistory.add(new ActionSelectHistoryInfo(currTask));
		updateHistoryPosition();

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
	public void drawActionsTreeLayout() {

		Vector2f currentPosition = new Vector2f(50+globalPosOffset.x,80+globalPosOffset.y);
		Map<Integer, Vector<Action>> levels = currTask.getTransitionsRecursive().getTreeLevels();
		Map<Integer, Float> levelWidths = currTask.getTransitionsRecursive().getLevelWidths();
		
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
			for(Action a : currTask.getSubActions()) {

				if( currTask.getTransitionsRecursive().getTransitionsFrom(a).isEmpty() &&
						currTask.getTransitionsRecursive().getTransitionsTo(a).isEmpty()) {

					if(a.isExpanded()) {
						a.getDrawInfo().drawSimpleBox(this, currentPosition, globalPosOffset, 10, true);
					} else {
						a.getDrawInfo().drawSimpleBox(this, currentPosition, globalPosOffset, 10, false);
					}

				}
			}
	}
	
	
	// draw  actions wherever they have been drawn before (don't re-arrange layout)
	private void drawActions() {

		List<Action> subactions = currTask.getSubActions();
		if(subactions!=null) {
			synchronized(subactions) {
				for(Action a : subactions) {

					if(a.isExpanded()) {
						a.getDrawInfo().drawSimpleBox(this, a.getDrawInfo().position, globalPosOffset, 10, true);
					} else {
						a.getDrawInfo().drawSimpleBox(this, a.getDrawInfo().position, globalPosOffset, 10, false);
					}
				}
			}
		}
	}

	
	public synchronized void drawTransitions(PApplet app) {

		ActionTransitions trans = currTask.getTransitionsRecursive();
		if(trans!=null) {
			//synchronized(trans) {
				// draw transition arrows
				for(ActionTransition t : trans) {
					t.drawConnection(app);
				}
			//}
		}
	}
	

	private void initControlP5() {
		
		// wait till window has been created
		while(findFrame()==null) {
			try { Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace(); } 
		}
		this.frame = findFrame();
		
	    controlP5 = new ControlP5(this);
	    GreyTheme.applyStyle(controlP5);


		GreyTheme.applyStyle(controlP5.addGroup("recipe properties", 900, 20, 230).setBackgroundHeight(200));

		base_iri = GreyTheme.applyStyle(controlP5.addTextfield("base iri", 10, 10, 210, 20).setGroup("recipe properties"));
		base_iri.setText("http://www.roboearth.org/kb/roboearth.owl#");

		start_action = GreyTheme.applyStyle(controlP5.addTextfield("start action", 10, 55, 160, 20).setGroup("recipe properties"));
		GreyTheme.applyStyle(controlP5.addButton("select start", 1f, 180, 55, 40, 20).setGroup("recipe properties")).getCaptionLabel().set("select");
		
		end_action = GreyTheme.applyStyle(controlP5.addTextfield("end action", 10, 100, 160, 20).setGroup("recipe properties"));
		GreyTheme.applyStyle(controlP5.addButton("select end", 2f, 180, 100, 40, 20).setGroup("recipe properties")).getCaptionLabel().set("select");
		
		GreyTheme.applyStyle(controlP5.addButton("add new action to recipe", 3f, 10, 160, 125, 20).setGroup("recipe properties"));


		
		
		GreyTheme.applyStyle(controlP5.addGroup("create new recipe", 900, 250, 230).setBackgroundHeight(120));
		
		new_recipe_shortname = GreyTheme.applyStyle(controlP5.addTextfield("shortname", 10, 10, 210, 20).setGroup("create new recipe"));
		new_recipe_shortname.setText("UniqueIdentifierForRecipe");
		
		new_recipe_label = GreyTheme.applyStyle(controlP5.addTextfield("label", 10, 55, 210, 20).setGroup("create new recipe"));
		new_recipe_label.setText("natural-language label");
		
		GreyTheme.applyStyle(controlP5.addButton("create", 46f, 185, 90, 35, 20).setGroup("create new recipe"));
		
		
	}
	
	
	
	@Override
	public void mouseMoved(MouseEvent e) {

		if(newTransitionFromAction!=null) {
			newTransitionToLocation.x = e.getX();
			newTransitionToLocation.y = e.getY();
			redraw();
		}

		if (getHistoryHover(e.getX(), e.getY())>=0) {
			setCursor(handCursor);
			return;
		}
		
		if(currTask!=null) {
			for(Action a : currTask.getSubActions()) {

				if (a.getDrawInfo().updateHover(e.getX(), e.getY()))
					setCursor(handCursor);
				else
					setCursor(normalCursor);
			}
		}

		if(controlP5!=null)
			controlP5.controlWindow.mouseEvent(e);

		if (selectedAction==null) {
			setCursor(normalCursor);
			return;
		}
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
		
		if(currTask!=null) {

			if (e.getButton() == MouseEvent.BUTTON1) {

				// check if we clicked on an action -> move only that action box
					for(Action a : currTask.getSubActions()) {
						if(a.getDrawInfo().checkClick(e.getX(), e.getY())!=null) {
							draggedAction = a;

							draggingStart = new Vector2f(e.getX(),e.getY());
							setCursor(moveCursor);
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
		
		if (diff < 150) { // double-fired event 
			return;
		}

		
		// only send left-button events to contolP5
		if(controlP5!=null && e.getButton() == MouseEvent.BUTTON1)
			controlP5.controlWindow.mouseEvent(e);
		
		if(currTask==null) {return;}
		
		if (e.getButton() == MouseEvent.BUTTON1) {

			
			// Check if clicked on outbound connector
			for(Action a : currTask.getSubActions()) {

				Action clicked_on = a.getDrawInfo().checkPosOverOutboundConnector(e.getX(), e.getY());
				if (clicked_on!= null) {

					// clicked nowhere special, reset highlight
					clearHighlight();
					selectedAction=null;
					if(activeTransition!=null)
						activeTransition.setActive(false);
					activeTransition = null;

					newTransitionFromAction = clicked_on;
					newTransitionToLocation.x = e.getX();
					newTransitionToLocation.y = e.getY();
					return;
				}
			}



			// Check if clicked on action in different modes
			for(Action a : currTask.getSubActions()) {
				if (a.getDrawInfo().checkHover(e.getX(), e.getY(), null)!= null) {
					
					// Check if in connection mode
					if(newTransitionFromAction!=null) {

						OWLIndividual t_ind = OWLIndividual.getOWLIndividualOfClass("http://ias.cs.tum.edu/kb/knowrob.owl#Transition");
						newTransitionFromAction.addTransition(ActionTransition.getActionTransition(t_ind.getIRI(), newTransitionFromAction, a, "OK"));
						redraw();

						// clicked nowhere special, reset highlight
						clearHighlight();
						selectedAction=null;
						if(activeTransition!=null)
							activeTransition.setActive(false);
						activeTransition = null;

						newTransitionFromAction = null;
						newTransitionToLocation = new Vector2f();
						return;
						
					} else if(select_start) {
						
						start_action.setText(a.getShortName());
						
						if(currTask.getHasValue().containsKey("http://ias.cs.tum.edu/kb/knowrob.owl#taskStartState"))
							currTask.getHasValue().get("http://ias.cs.tum.edu/kb/knowrob.owl#taskStartState").clear();
						
						currTask.addHasValue("http://ias.cs.tum.edu/kb/knowrob.owl#taskStartState", a.getIRI());
						select_start = false;
						
					} else if(select_end) {
						
						end_action.setText(a.getShortName());
						
						if(currTask.getHasValue().containsKey("http://ias.cs.tum.edu/kb/knowrob.owl#taskEndState"))
							currTask.getHasValue().get("http://ias.cs.tum.edu/kb/knowrob.owl#taskEndState").clear();
						
						currTask.addHasValue("http://ias.cs.tum.edu/kb/knowrob.owl#taskEndState", a.getIRI());
						select_end = false;						
					}
				}
			}
			

			// reset connection mode if clicked elsewhere
			newTransitionFromAction = null;
			newTransitionToLocation = new Vector2f();


			// check if (double)clicked on action
			for(Action a : currTask.getSubActions()) {

				Action clicked_on  = a.getDrawInfo().checkClick(e.getX(), e.getY());

				if (clicked_on!= null) {

					clearHighlight();
					if(activeTransition!=null)
						activeTransition.setActive(false);
					activeTransition = null;

					// edit action if double-clicked
					if(e.getClickCount()==2) {

						// open action editor
						ActionEditorWindow f = new ActionEditorWindow();
						f.setAddActionCallback(this);
						f.setBaseIRI(base_iri.getText());

						while(!f.applet.isInitialized()) {
							try {Thread.sleep(10);
							} catch (InterruptedException e1) {e1.printStackTrace(); }
						}

						// init action editor
						f.applet.setIdentifier(a.getIRI());
						f.applet.setEditing(true);
						f.applet.setActionClass(a.getSuperClasses().firstElement().getIRI());
						f.applet.setActionProperties(a.getProperties());
						
						// update click time if init took longer
						lastClickTime = System.currentTimeMillis();
					
						return;

					} else {

						// activate action after single click
						selectedAction = clicked_on;
						highlightAction(selectedAction, false);

						return;
					}
				}
			}
			
			// activate transition arrow after click
			ActionTransitions trans = currTask.getTransitionsRecursive();
			synchronized(trans) {
				for(ActionTransition t : trans) {

					boolean inside = t.checkPosInArrow(new Vector2f(e.getX(), e.getY()), this);

					if (inside) {

						if(e.getClickCount()==2) {
							

							//debug
							System.out.println("double click on transition");
							new TransitionPropertiesEditor(this, t);
							
						} else {

							// deactivate transition with second click
							if(t.isActive()) {
								t.setActive(false);
								activeTransition = null;
							}

							// reset action selection
							clearHighlight();
							selectedAction=null;

							// reset previously active transition, set to t 
							t.setActive(true);

							if(activeTransition!=null)
								activeTransition.setActive(false);

							activeTransition = t;
							return;
						}
					}
				}
			}
			
			// clicked nowhere special, reset highlight
			clearHighlight();
			selectedAction=null;
			
			if(activeTransition!=null)
				activeTransition.setActive(false);
			
			activeTransition = null;
		}
		
		
		
		// TODO: adapt to multi-action case and reactivate			
//					//Check if clicked on history element
//					int idx = getHistoryHover(e.getX(), e.getY());
//					if (idx >= 0) {
//						selectedAction = clickHistory.get(idx).getAction();
//						for (int i=clickHistory.size()-1; i>idx; i--)
//						{
//							clickHistory.remove(i);
//						}
//						updateHistoryPosition();
//						return;
//					}
//					
					
		// TODO: adapt to multi-action case and reactivate			
//					//Check if clicked on an expand button
//					for(Action a : actions) {
//						
//						Action clicked_on = a.getDrawInfo().checkClickExpand(e.getX(), e.getY());
//						if (clicked_on!= null) {
//							
//							clicked_on.toggleExpand();
//							selectedAction.getDrawInfo().notifyModified();
//							return;
//						}
//					}
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

	
	public void keyPressed(KeyEvent e) {
		
		
		if(e.getKeyCode() == KeyEvent.VK_DELETE) {

			if(currTask!=null) {
				ActionTransitions trans = currTask.getTransitionsRecursive();
				synchronized(trans) {

					if(activeTransition!=null) {
						activeTransition.getFrom().removeTransition(activeTransition);
						this.activeTransition = null;
						currTask.setSaveToProlog(true);
					}

					if(selectedAction!=null) {

						currTask.removeSubAction(selectedAction);

						for(ActionTransition t : trans) {
							if(t.getFrom().equals(selectedAction) || 
									t.getTo().equals(selectedAction)) {

								t.getFrom().removeTransition(t);
							} 
						}

						this.selectedAction = null;
						currTask.setSaveToProlog(true);
					}
				}
			}
		}

		if(controlP5!=null)
			controlP5.keyHandler.keyEvent(e, controlP5.controlWindow, true);
		
	}
	
	
	/**
	 * Handle control events from controlP5
	 * 
	 * @param ev The ControlEvent generated by controlP5
	 */
	public void controlEvent(ControlEvent ev) {

		if(ev.isController()) {

			// open action creation dialog	
			if(ev.getController().getName().equals("add new action to recipe")) {
								
				ActionEditorWindow f = new ActionEditorWindow();
				f.setAddActionCallback(this); 
				f.setBaseIRI(base_iri.getText());
				
				
			} else if(ev.getController().getName().equals("create")) {

				// create new recipe
				Action new_action = Action.getAction(base_iri.getText() + new_recipe_shortname.getText(), 
													 new_recipe_label.getText());
				new_action.addSuperClass(Action.getOWLClass("http://ias.cs.tum.edu/kb/knowrob.owl#PurposefulAction"));
				new_action.getDrawInfo().recalculateDimensions(this);
				this.setTask(new_action);
				
				
			} else if(ev.getController().getName().equals("select start")) {
				select_start = true;
			} else if(ev.getController().getName().equals("select end")) {
				select_end  = true;
			}
		}
	}
	
	
	/**
	 * Synchronize all changes that have been made with the underlying Prolog engine
	 */
	public void syncWithProlog() {
		currTask.setSaveToProlog(true);
		this.currTask.writeToProlog();
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
	public boolean highlightAction(Action a, boolean expand) {
		
		if (currTask == null)
			return false;
		
		return highlightAction(a.getIRI(), expand);
	}
	
	/**
	 * Highlight the action referenced by <code>identifier</code>
	 * @param identifier action identified by <code>identifier</code> to highlight
	 * @param expand Expand all the parents of <code>identifier</code> so that <code>identifier</code> is visible
	 * @return true if action found and highlighted
	 */
	public boolean highlightAction(String identifier, boolean expand) {
		
		if (currTask == null)
			return false;
		
		// find action with this identifier
		for(Action a : currTask.getSubActionsRecursive()) {
			a.getDrawInfo().clearHightlight();
			if(a.getIRI().equals(identifier)) {
				clearHighlight();
				a.getDrawInfo().setHightlight(HighlightType.THIS_HIGHLIGHTED);
				selectedAction=a;
				return true;
			}
		}
		return false;
//		selectedAction.getDrawInfo().clearHightlight();
//		return selectedAction.getDrawInfo().highlightSubsequence(identifier,expand);
	}
	
	/**
	 * Clear all highlighted actions so that none is highlighted.
	 */
	public void clearHighlight()
	{
		if (currTask == null)
			return;

		for(Action a : currTask.getSubActionsRecursive()) {
			a.getDrawInfo().clearHightlight();
		}
	}

	
	public void addAction(Action a) {

		a.getDrawInfo().position = new Vector2f(50+globalPosOffset.x,80+globalPosOffset.y);
		currTask.addSubAction(a);

	}
	

	public Frame findFrame() {	
		
		Container f = this.getParent();
		while (!(f instanceof Frame) && f!=null)
			f = f.getParent();
		return (Frame) f;
	}
	
	public Action getCurrTask() {
		return currTask;
	}



	/**
	 * Wrapper class around an action editor applet in a new window
	 *  
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 */
	public class ActionEditorWindow extends Frame {

		private static final long serialVersionUID = 543157068719461737L;
		public ActionPropertiesEditor applet;
		
		public ActionEditorWindow() {
				PrologInterface.initJPLProlog("ias_knowledge_base");
			
	        setBounds(100,100,500,500);
	        applet = new ActionPropertiesEditor();
	        applet.frame = this;
	        add(applet);
	        applet.init();
			this.setVisible(true);
	    }

		public void setAddActionCallback(IAddActionCallback cb) {
			applet.setAddActionCallback(cb); 
		}
		
		public void setBaseIRI(String base_iri) {
			applet.setBaseIRI(base_iri); 
		}
	}
	
}

package edu.tum.cs.ias.knowrob.vis.applets;

import java.awt.Color;
import java.awt.Cursor;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Vector;

import javax.vecmath.Vector2f;

import edu.tum.cs.ias.knowrob.owl.OWLThing;
import edu.tum.cs.ias.knowrob.prolog.PrologInterface;
import edu.tum.cs.ias.knowrob.vis.actions.Action;
import edu.tum.cs.ias.knowrob.vis.actions.ActionSelectHistoryInfo;

import processing.core.PApplet;
import processing.core.PFont;

/**
 * Visualization applet for action plans/sequences.
 * @author Stefan Profanter
 * @see edu.tum.cs.ias.knowrob.vis.actions.Action
 */
public class PlanVisApplet  extends PApplet implements MouseListener, MouseMotionListener,MouseWheelListener {

	private static final long serialVersionUID = 7695328948788620463L;

	/**
	 * Font for drawing text
	 */
	private PFont dejavuFont;

	/**
	 * Current selected action
	 */
	private Action currAction;

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

	/**
	 * Constructor
	 */
	public PlanVisApplet()
	{
		this.redraw();
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

	    dejavuFont = createFont("DejaVu Sans",13);
	    textFont(dejavuFont);
	    hint(ENABLE_ACCURATE_TEXTURES);
	    ellipseMode(RADIUS);
	    frameRate(20);
	}

	@Override
	public void draw() {

		background(40);

		textFont(dejavuFont);
	    textMode(SCREEN);


	    drawCurrAction();
	    drawHistory();


	}

	/**
	 * Load a plan with the given prolog identifier and all it's subactions.
	 * @param iri Something like 'http://www.roboearth.org/kb/serve_drink.owl#ServeADrink'
	 * @return The main action initialized by the identifier
	 */
	private Action loadPrologPlanRecursive(String iri)
	{

		//Get the action name
		String label = "";
		try
		{
			HashMap<String, Vector<String>> qLabel = PrologInterface.executeQuery("rdf_has('"+iri+"',rdfs:label,L),util:strip_literal_type(L,Label)");

			label = qLabel.get("Label").get(0);
			if (label.startsWith("'") && label.endsWith("'"))
			{
				label = label.substring(1,label.length()-1);
			}
		} catch (Exception e)
		{
			if (iri.indexOf('#') >=0)
			{
				label = iri.substring(iri.lastIndexOf('#')+1);
			} else {
				label = iri;
			}
		}

		Action ret = Action.getAction(iri, label);

		//get properties
		try
		{
			HashMap<String, Vector<String>> qProp = PrologInterface.executeQuery("class_properties('"+iri+"', Key, V), util:strip_literal_type(V,Val)");

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
							ret.addHasValue(k, v);
					}
			}
		} catch (Exception e)
		{
			e.printStackTrace();
		}

		//get subactions
		try {
			HashMap<String, Vector<String>> qSub = PrologInterface.executeQuery("plan_subevents('"+iri+"',List)");

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
								if (alreadyAdded.contains(s))
									continue;
								alreadyAdded.add(s);
								ret.addSubAction(loadPrologPlanRecursive(PrologInterface.removeSingleQuotes(s)));
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
	 * Sets the action which should be selected at the beginning.
	 * If none set, nothing will be drawn.
	 * @param action action to set as start action
	 */
	public void setMainAction(Action action)
	{
		currAction = action;
		clickHistory.add(new ActionSelectHistoryInfo(currAction));
		updateHistoryPosition();
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


		applet.beginShape();

		applet.vertex(p1.x,p1.y);
		applet.vertex(p2.x,p2.y);
		applet.vertex(p3.x,p3.y);
		applet.vertex(to.x,to.y); //front tip
		applet.vertex(p5.x,p5.y);
		applet.vertex(p6.x,p6.y);
		applet.vertex(p7.x,p7.y);

		applet.endShape(CLOSE);


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
	private void drawCurrAction()
	{
		if (currAction == null)
			return;

		currAction.getDrawInfo().drawExtendedBox(this, new Vector2f(50+drawOffset.x,80+drawOffset.y), drawOffset);
	}

	@Override
	public void mouseMoved(MouseEvent e) {
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
       if (currAction.getDrawInfo().updateHover(e.getX(), e.getY()))
    	   setCursor(handCursor);
       else
    	   setCursor(normalCursor);
    }

	@Override
    public void mouseDragged(MouseEvent e) {
		if (draggingStart != null && currAction != null)
		{
			drawOffset.x = Math.min(0, drawOffset.x+ e.getX() - draggingStart.x);
			drawOffset.y = Math.min(0,drawOffset.y + e.getY() - draggingStart.y);

			float viewOffsetX = Math.min(0, this.getWidth()-(currAction.getDrawInfo().getBoundingBox().width+100));
			float viewOffsetY = Math.min(0, this.getHeight()-(currAction.getDrawInfo().getBoundingBox().height+130));

			drawOffset.x = Math.max(viewOffsetX,drawOffset.x);
			drawOffset.y = Math.max(viewOffsetY,drawOffset.y);

			draggingStart.x = e.getX();
			draggingStart.y = e.getY();
		}
    }

	@Override
	public void mousePressed(MouseEvent e) {
		if (e.getButton() == 3)
		{
			draggingStart = new Vector2f(e.getX(),e.getY());
			setCursor(moveCursor);
		} else {
			setCursor(normalCursor);
		}
    }

	@Override
    public void mouseReleased(MouseEvent e) {
		setCursor(normalCursor);
    }

	@Override
    public void mouseEntered(MouseEvent e) {
    }

	@Override
    public void mouseExited(MouseEvent e) {
    }

	@Override
    public void mouseClicked(MouseEvent e) {
		long diff = System.currentTimeMillis()-lastClickTime;
		lastClickTime = System.currentTimeMillis();
		if (diff < 10) //double fired event
			return;
		if (e.getButton() == MouseEvent.BUTTON1 && currAction != null)
		{
			//Check if clicked on history element
			int idx = getHistoryHover(e.getX(), e.getY());
			if (idx >= 0)
			{
				currAction = clickHistory.get(idx).getAction();
				for (int i=clickHistory.size()-1; i>idx; i--)
				{
					clickHistory.remove(i);
				}
				updateHistoryPosition();
				return;
			}

			//Check if clicked on an expand button
			Action a = currAction.getDrawInfo().checkClickExpand(e.getX(), e.getY());
			if (a!= null)
			{
				a.toggleExpand();
				currAction.getDrawInfo().notifyModified();
				return;
			}

			//Check if clicked on an action
			a = currAction.getDrawInfo().checkClick(e.getX(), e.getY());
			if (a!= null && a != currAction)
			{
				currAction = a;
				if (clickHistory.size()>1 && clickHistory.get(clickHistory.size()-2).getAction()==a)
					clickHistory.remove(clickHistory.size()-1);
				else
					clickHistory.add(new ActionSelectHistoryInfo(currAction));
				updateHistoryPosition();
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

	@Override
	public void mouseWheelMoved(MouseWheelEvent e) {
			int totalScrollAmount = 0;
			if (e.getScrollType() == MouseWheelEvent.WHEEL_UNIT_SCROLL)
			{
				totalScrollAmount = e.getUnitsToScroll() * e.getScrollAmount();
			} else {
				totalScrollAmount = e.getWheelRotation() * 30;
			}
			draggingStart.y += totalScrollAmount;
	}

}

package edu.tum.cs.vis.gui.applet;

import java.awt.Color;
import java.awt.Cursor;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedList;

import javax.vecmath.Vector2f;

import edu.tum.cs.vis.action.Action;
import edu.tum.cs.vis.action.ActionSelectHistoryInfo;

import processing.core.PApplet;
import processing.core.PConstants;
import processing.core.PFont;

/**
 * Visualization applet for action plans/sequences. 
 * @author Stefan Profanter
 * @see edu.tum.cs.vis.action.Action
 */
public class PlanVisApplet  extends PApplet implements MouseListener, MouseMotionListener {

	private static final long serialVersionUID = 7695328948788620463L;

	/**
	 * Font for drawing text
	 */
	private PFont dejavuFont;
	
	private Action mainAction;
	
	private Action currAction;
	
	
	private Color borderColor;
	private Color textColor;
	private Color backgroundColor;
	private Color backgroundBrightColor;
	
	private static final Cursor normalCursor = new Cursor(Cursor.DEFAULT_CURSOR);
	private static final Cursor handCursor = new Cursor(Cursor.HAND_CURSOR);
	

	private ArrayList<ActionSelectHistoryInfo> clickHistory = new ArrayList<ActionSelectHistoryInfo>();
	
	@Override
	public void setup()
	{
		size(800, 600, P2D);
		if (this.frame != null)
		{
		    this.frame.setTitle("Action plans visualisation");
		    this.frame.setBackground(new Color(10, 10, 10));
		}
		
		addMouseMotionListener(this);
		addMouseListener(this);
		
		borderColor = new Color(100, 100, 100);
		textColor = new Color(240,240,240);
		backgroundColor = new Color(30,30,30);
		backgroundBrightColor = new Color(50,50,50);
	    
	    dejavuFont = createFont("DejaVu Sans",13);
	    textFont(dejavuFont);
	    hint(ENABLE_ACCURATE_TEXTURES);
	    ellipseMode(RADIUS);
	    frameRate(25);
	}
	

	public void draw() {
		background(10, 10, 10);
		
		textFont(dejavuFont);
	    textMode(SCREEN);
	    
	    drawHistory();
	    drawCurrAction();
	    
	}
	
	public void setMainAction(Action action)
	{
		mainAction = action;
		currAction = action;
		clickHistory.add(new ActionSelectHistoryInfo(currAction));
		updateHistoryPosition();
		this.redraw();
	}
	
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
	
	private void drawHistory()
	{
		for (int i= clickHistory.size()-1; i>=0; i--)
		{
			clickHistory.get(i).Draw(this);
		}
	}
	
	private void drawCurrAction()
	{
		if (currAction == null)
			return;
		//currAction.getDrawInfo().drawSimpleBox(this, new Vector2f(50,50),0);
		currAction.getDrawInfo().drawExtendedBox(this, new Vector2f(50,80));
		stroke(borderColor.getRed(), borderColor.getBlue(), borderColor.getGreen(), borderColor.getAlpha());
	    fill(backgroundColor.getRed(), backgroundColor.getBlue(), backgroundColor.getGreen(), backgroundColor.getAlpha());
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
    }
	
	@Override
	public void mousePressed(MouseEvent e) {
    }

	@Override
    public void mouseReleased(MouseEvent e) {
    }

	@Override
    public void mouseEntered(MouseEvent e) {
    }

	@Override
    public void mouseExited(MouseEvent e) {
    }

	@Override
    public void mouseClicked(MouseEvent e) {
		if (e.getButton() == MouseEvent.BUTTON1)
		{
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
			
			
			
			Action a = currAction.getDrawInfo().checkClick(e.getX(), e.getY());
			if (a!= null && a != currAction)
			{
				currAction = a;
				clickHistory.add(new ActionSelectHistoryInfo(currAction));
				updateHistoryPosition();
			}
		}
    }
	
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

}

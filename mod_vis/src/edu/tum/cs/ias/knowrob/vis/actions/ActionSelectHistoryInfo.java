package edu.tum.cs.ias.knowrob.vis.actions;

import java.awt.Color;

import javax.vecmath.Vector2f;

import processing.core.PApplet;
import processing.core.PConstants;

/**
 * Action Wrapper for placing an action into the click history.
 * Also used for drawing the action in the history.
 * 
 * @author Stefan Profanter
 *
 */
public class ActionSelectHistoryInfo {

	/**
	 * Action for which this wrapper is
	 */
	private Action action;
	/**
	 * True if mouse is hovering over this history entry
	 */
	private boolean isHover = false;
	
	/**
	 * History may be longer than actual window width. If this action is out of view, draw is set to false.
	 * So it doesn't need to be drawn.
	 */
	private boolean draw = true;
	
	/**
	 * Position of the history entry
	 */
	private Vector2f pos;
	/**
	 * Dimension of the history entry
	 */
	private Vector2f dim;
	
	/**
	 * Default colors for drawing the history entry
	 */
	private final static Color borderColor = new Color(80, 80, 80);
	private final static Color backgroundColor = new Color(30,30,30);
	private final static Color textColor = new Color(80,80,80);
	
	/**
	 * Colors used for drawing when mouse is hovering over this entry or this entry is current action
	 */
	private final static Color hoverBorderColor = new Color(100, 100, 100);
	private final static Color hoverBackgroundColor = new Color(50,50,50);
	private final static Color hoverTextColor = new Color(150,150,150);
	
	/**
	 * Colors set according to current state (hovering/current or not)
	 */
	private Color currentBorderColor = borderColor;
	private Color currentBackgroundColor = backgroundColor;
	private Color currentTextColor = textColor;
	
	/**
	 * Text padding left and right
	 */
	private final static float PADDING_LEFT_RIGHT = 10;
	
	/**
	 * Constructor
	 * @param a Action for which this wrapper is
	 */
	public ActionSelectHistoryInfo(Action a)
	{
		this.action = a;
		dim = new Vector2f(a.getDrawInfo().getNameWidth()+2f * PADDING_LEFT_RIGHT,a.getDrawInfo().getNameBoxHeight());
	}
	
	/**
	 * Set position of the history entry.
	 * @param x x position
	 * @param y y position
	 * @param isCurrent Set to true if this is last entry in history list (this represents the current drawn action)
	 */
	public void setPosition(float x, float y, boolean isCurrent)
	{
		pos = new Vector2f(x,y);
		draw = (x+dim.x > 0);
		if (isCurrent)
		{
			currentBorderColor = hoverBorderColor;
			currentBackgroundColor = hoverBackgroundColor;
			currentTextColor = hoverTextColor;
		} else 
		{
			currentBorderColor = borderColor;
			currentBackgroundColor = backgroundColor;
			currentTextColor = textColor;
		}
	}
	
	/**
	 * Get wrapping action
	 * @return action
	 */
	public Action getAction()
	{
		return action;
	}
	
	/**
	 * Get dimension of history entry
	 * @return dimenstion of history entry (including padding)
	 */
	public Vector2f getDimension()
	{
		return new Vector2f(dim);
	}
	
	/**
	 * Set correct drawing colors if mouse is hovering over entry
	 * @param hover true if mouse is hovering over this entry
	 */
	public void setHover(boolean hover)
	{
		if (hover)
		{
			currentBorderColor = hoverBorderColor;
			currentBackgroundColor = hoverBackgroundColor;
			currentTextColor = hoverTextColor;
		} else 
		{
			currentBorderColor = borderColor;
			currentBackgroundColor = backgroundColor;
			currentTextColor = textColor;
		}
		isHover = hover;
	}
	
	/**
	 * Check if mouse coordinates x|y are over this history entry.
	 * @param x x position of mouse
	 * @param y y position of mouse
	 * @return true if position is over this entry
	 */
	public boolean checkHover(float x, float y)
	{
		if (x>pos.x && x<pos.x+dim.x+PADDING_LEFT_RIGHT*0.5f && y>pos.y && y<pos.y+dim.y)
		{
			setHover(true);
			return true;
		} else {
			setHover(false);
			return false;
		}
	}
	
	/**
	 * Draw the history item on the given applet.
	 * @param applet Applet to draw on.
	 */
	public void Draw(PApplet applet)
	{
		if (!draw)
			return;
		if (dim.y == 0) //Not yet correctly initialized
			dim = new Vector2f(action.getDrawInfo().getNameWidth()+2f * PADDING_LEFT_RIGHT,action.getDrawInfo().getNameBoxHeight());
		
		applet.stroke(currentBorderColor.getRed(), currentBorderColor.getGreen(), currentBorderColor.getBlue(), currentBorderColor.getAlpha());
	    applet.fill(currentBackgroundColor.getRed(), currentBackgroundColor.getGreen(), currentBackgroundColor.getBlue(), currentBackgroundColor.getAlpha());
		applet.beginShape();
		
		applet.vertex(pos.x,pos.y);
		applet.vertex(pos.x+dim.x,pos.y);
		applet.vertex(pos.x+dim.x+10,pos.y+dim.y/2f);
		applet.vertex(pos.x+dim.x,pos.y+dim.y);
		applet.vertex(pos.x,pos.y+dim.y);
				
		applet.endShape(PConstants.CLOSE);
		
		applet.fill(currentTextColor.getRed(), currentTextColor.getGreen(), currentTextColor.getBlue(), currentTextColor.getAlpha());
		applet.text(action.getLabel(),pos.x + PADDING_LEFT_RIGHT*1.5f,pos.y + action.getDrawInfo().getTextHeight()*2f);

	}
	
	/**
	 * Return if mouse is hovering over this entry.
	 * @return true if hovering.
	 */
	public boolean IsHover()
	{
		return isHover;
	}
	
}

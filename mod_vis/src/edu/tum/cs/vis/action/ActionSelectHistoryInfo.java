package edu.tum.cs.vis.action;

import java.awt.Color;

import javax.vecmath.Vector2f;

import processing.core.PApplet;
import processing.core.PConstants;

public class ActionSelectHistoryInfo {

	private Action action;
	private boolean isHover = false;
	private boolean draw = true;
	
	private Vector2f pos;
	private Vector2f dim;
	
	private final static Color borderColor = new Color(80, 80, 80);
	private final static Color backgroundColor = new Color(30,30,30);
	private final static Color textColor = new Color(80,80,80);
	
	private final static Color hoverBorderColor = new Color(100, 100, 100);
	private final static Color hoverBackgroundColor = new Color(50,50,50);
	private final static Color hoverTextColor = new Color(150,150,150);
	

	private Color currentBorderColor = borderColor;
	private Color currentBackgroundColor = backgroundColor;
	private Color currentTextColor = textColor;
	
	private final static float PADDING_LEFT_RIGHT = 10;
	
	public ActionSelectHistoryInfo(Action a)
	{
		this.action = a;
		dim = new Vector2f(a.getDrawInfo().getNameWidth()+2f * PADDING_LEFT_RIGHT,a.getDrawInfo().getNameBoxHeight());
	}
	
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
	
	public Action getAction()
	{
		return action;
	}
	
	public Vector2f getDimension()
	{
		return new Vector2f(dim);
	}
	
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
		applet.text(action.getName(),pos.x + PADDING_LEFT_RIGHT*1.5f,pos.y + action.getDrawInfo().getTextHeight()*2f);

	}
	
}

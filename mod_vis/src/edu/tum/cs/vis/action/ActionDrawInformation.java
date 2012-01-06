package edu.tum.cs.vis.action;

import java.awt.Color;
import java.util.Collection;
import java.util.Iterator;

import javax.vecmath.Vector2f;

import processing.core.PApplet;

public class ActionDrawInformation {
	
	public final static float LINE_HEIGHT = 1.5f;
	public final static float INNER_CONTENT_PADDING = 10f;
	public final static float MAIN_BOX_PADDING = 15f;
	public final static float SEQUENCE_BOX_PADDING = 10f;
		
	private final static Color borderColor = new Color(100, 100, 100);
	private final static Color backgroundColor = new Color(30,30,30);
	private final static Color backgroundBrightColor = new Color(50,50,50);
	private final static Color textColor = new Color(240,240,240);
	
	private final static Color hoverBorderColor = new Color(200, 200, 200);
	private final static Color hoverBackgroundColor = new Color(60,60,60);
	private final static Color hoverBackgroundBrightColor = new Color(100,100,100);
	private final static Color hoverTextColor = new Color(255,255,255);
	
	private Color currentBorderColor = borderColor;
	private Color currentBackgroundColor = backgroundColor;
	private Color currentBackgroundBrightColor = backgroundBrightColor;
	private Color currentTextColor = textColor;
	
	private Action action;
	private boolean needsRecalculation = true;
	
	private boolean isHover = false;
	
	private float nameWidth;
	
	private float maxKeyWidth;
	private float maxValueWidth;
	private float propertiesHeight;
	
	private float textHeight;
	
	private float parentsMaxHeight;
	private float childrenMaxHeight;
	
	private Vector2f sequenceBoxDimension = new Vector2f();
	
	public ActionDrawInformation(Action parent)
	{
		action = parent;
	}
	
	public void notifyModified()
	{
		needsRecalculation = true;
	}
	
	public float getNameBoxHeight()
	{
		return textHeight*3;
	}
	
	public Vector2f getSimpleBoxDimension()
	{
		//Max with of name or properties
		float width = Math.max(nameWidth,maxKeyWidth+maxValueWidth)+ MAIN_BOX_PADDING*2;
		
		//Height of name box
		float height = getNameBoxHeight();
		
		height += propertiesHeight+INNER_CONTENT_PADDING + MAIN_BOX_PADDING;
		
		return new Vector2f(width, height);
	}
	
	public Vector2f getExtendedBoxDimension()
	{
		Vector2f dim = getSimpleBoxDimension();
		dim.x = Math.max(dim.x,sequenceBoxDimension.x + 2*MAIN_BOX_PADDING);
		dim.y += sequenceBoxDimension.y + INNER_CONTENT_PADDING;
		
		return dim;
	}
	
	private void recalculateDimensions(PApplet applet)
	{
		if (!needsRecalculation)
			return;
		
		textHeight = applet.textAscent();
		nameWidth = applet.textWidth(action.getName());
		
		maxKeyWidth = 0;
		maxValueWidth = 0;
		propertiesHeight = 0;
		Collection<String> keys = action.getProperties().keySet();
		for (Iterator<String> it = keys.iterator(); it.hasNext(); )
		{
			String key = it.next();
			String value = action.getProperty(key);
			maxKeyWidth = Math.max(maxKeyWidth, applet.textWidth(key+":  "));
			maxValueWidth = Math.max(maxValueWidth, applet.textWidth(value));
			propertiesHeight += textHeight* LINE_HEIGHT;
		}
		
		needsRecalculation = false;
		
		//Calculation for sequence boxes
		sequenceBoxDimension.x = SEQUENCE_BOX_PADDING;
		sequenceBoxDimension.y = 0;
		for (Iterator<Action> i = action.getSequenceIterator(); i.hasNext();)
		{
			ActionDrawInformation inf = i.next().getDrawInfo();
			
			inf.recalculateDimensions(applet);
			
			sequenceBoxDimension.x += inf.getSimpleBoxDimension().x + SEQUENCE_BOX_PADDING;
			sequenceBoxDimension.y = Math.max(sequenceBoxDimension.y, inf.getSimpleBoxDimension().y + 2*SEQUENCE_BOX_PADDING);
		}
		
		//Calculation for parent boxes
		parentsMaxHeight = 0;
		for (Iterator<Action> i = action.getParentActionsIterator(); i.hasNext(); )
		{
			ActionDrawInformation inf = i.next().getDrawInfo();
			
			inf.recalculateDimensions(applet);
			
			parentsMaxHeight = Math.max(parentsMaxHeight, inf.getSimpleBoxDimension().y);
		}
		
		//Calculation for child boxes
		childrenMaxHeight = 0;
		for (Iterator<Action> i = action.getChildActionsIterator(); i.hasNext(); )
		{
			ActionDrawInformation inf = i.next().getDrawInfo();
			
			inf.recalculateDimensions(applet);
			
			childrenMaxHeight = Math.max(childrenMaxHeight, inf.getSimpleBoxDimension().y);
		}
	}
	
	private void drawBorderAndTitle(PApplet applet, Vector2f position, Vector2f dimension)
	{
		//Draw outer border and background
		applet.stroke(currentBorderColor.getRed(), currentBorderColor.getBlue(), currentBorderColor.getGreen(), currentBorderColor.getAlpha());
	    applet.fill(currentBackgroundColor.getRed(), currentBackgroundColor.getBlue(), currentBackgroundColor.getGreen(), currentBackgroundColor.getAlpha());
		applet.rect(position.x, position.y, dimension.x, dimension.y);
		
		//Draw box title
		applet.fill(currentBackgroundBrightColor.getRed(), currentBackgroundBrightColor.getBlue(), currentBackgroundBrightColor.getGreen(), currentBackgroundBrightColor.getAlpha());
		applet.rect(position.x,position.y,dimension.x, getNameBoxHeight());
		applet.fill(currentTextColor.getRed(), currentTextColor.getBlue(), currentTextColor.getGreen(), currentTextColor.getAlpha());
		applet.text(action.getName(),position.x + MAIN_BOX_PADDING,position.y + textHeight*2f);
	}
	
	private void drawProperties(PApplet applet, Vector2f position)
	{
		applet.fill(currentTextColor.getRed(), currentTextColor.getBlue(), currentTextColor.getGreen(), currentTextColor.getAlpha());
		Collection<String> keys = action.getProperties().keySet();
		for (Iterator<String> it = keys.iterator(); it.hasNext(); )
		{
			String key = it.next();
			String value = action.getProperty(key);
			
			applet.text(key + ":",position.x,position.y+textHeight);
			applet.text(value,position.x + maxKeyWidth, position.y+textHeight);

			position.y += textHeight * LINE_HEIGHT;
		}
	}
	
	public void drawSimpleBox(PApplet applet,Vector2f position, float minHeight)
	{
		Vector2f tmpPos = new Vector2f(position);
		recalculateDimensions(applet);
		
		drawBorderAndTitle(applet, tmpPos,new Vector2f(getSimpleBoxDimension().x,Math.max(minHeight, getSimpleBoxDimension().y)));		
		
		tmpPos.x += MAIN_BOX_PADDING;
		tmpPos.y += getNameBoxHeight()+INNER_CONTENT_PADDING;
		
		drawProperties(applet, tmpPos);
	}
	
	private void drawParentBoxes(PApplet applet, Vector2f position, Vector2f connectionPoint)
	{
		Vector2f tmpPos = new Vector2f(position);
		
		for (Iterator<Action> i = action.getParentActionsIterator(); i.hasNext(); )
		{
			ActionDrawInformation inf = i.next().getDrawInfo();
			
			inf.drawSimpleBox(applet, tmpPos, parentsMaxHeight);
			
			applet.stroke(inf.currentBorderColor.getRed(), inf.currentBorderColor.getBlue(), inf.currentBorderColor.getGreen(), inf.currentBorderColor.getAlpha());
			applet.line(tmpPos.x + inf.getSimpleBoxDimension().x/2f, tmpPos.y+parentsMaxHeight, connectionPoint.x,connectionPoint.y);
			
			tmpPos.x += inf.getSimpleBoxDimension().x + MAIN_BOX_PADDING;
		}
	}
	
	private void drawChildrenBoxes(PApplet applet, Vector2f position, Vector2f connectionPoint)
	{
		Vector2f tmpPos = new Vector2f(position);
		
		for (Iterator<Action> i = action.getChildActionsIterator(); i.hasNext(); )
		{
			ActionDrawInformation inf = i.next().getDrawInfo();
			
			inf.drawSimpleBox(applet, tmpPos, childrenMaxHeight);
			
			applet.stroke(inf.currentBorderColor.getRed(), inf.currentBorderColor.getBlue(), inf.currentBorderColor.getGreen(), inf.currentBorderColor.getAlpha());
			applet.line(tmpPos.x + inf.getSimpleBoxDimension().x/2f, tmpPos.y, connectionPoint.x,connectionPoint.y);
			
			tmpPos.x += inf.getSimpleBoxDimension().x + MAIN_BOX_PADDING;
		}
	}
	
	private void drawSequence(PApplet applet, Vector2f position)
	{
		//Draw outer box of sequence list
		applet.stroke(currentBorderColor.getRed(), currentBorderColor.getBlue(), currentBorderColor.getGreen(), currentBorderColor.getAlpha());
	    applet.fill(currentBackgroundBrightColor.getRed(), currentBackgroundBrightColor.getBlue(), currentBackgroundBrightColor.getGreen(), currentBackgroundBrightColor.getAlpha());
		applet.rect(position.x, position.y, sequenceBoxDimension.x, sequenceBoxDimension.y);
		
		float boxMinHeight = sequenceBoxDimension.y - 2*SEQUENCE_BOX_PADDING;
		Vector2f tmpPos = new Vector2f(position);
		tmpPos.y += SEQUENCE_BOX_PADDING;
		tmpPos.x += SEQUENCE_BOX_PADDING;
		
		for (Iterator<Action> i = action.getSequenceIterator(); i.hasNext();)
		{
			ActionDrawInformation inf = i.next().getDrawInfo();
			
			inf.drawSimpleBox(applet, tmpPos, boxMinHeight);
			tmpPos.x += inf.getSimpleBoxDimension().x + SEQUENCE_BOX_PADDING;
		}
	}
	
	public void drawExtendedBox(PApplet applet, Vector2f position)
	{
		recalculateDimensions(applet);
		
		Vector2f extendedDim = getExtendedBoxDimension();
		
		drawParentBoxes(applet, position, new Vector2f(position.x+extendedDim.x/2f,position.y+parentsMaxHeight+MAIN_BOX_PADDING));
		
		position.y += parentsMaxHeight+MAIN_BOX_PADDING;
		
		drawBorderAndTitle(applet, position,extendedDim);		
		
		position.x += MAIN_BOX_PADDING;
		position.y += getNameBoxHeight()+INNER_CONTENT_PADDING;
		
		drawProperties(applet, position);

		position.y += INNER_CONTENT_PADDING;
		
		drawSequence(applet, position);
		
		position.y += sequenceBoxDimension.y;
		position.x -= MAIN_BOX_PADDING;
		
		position.y += MAIN_BOX_PADDING*2;
		drawChildrenBoxes(applet, position, new Vector2f(position.x+extendedDim.x/2f,position.y-MAIN_BOX_PADDING));
	}
}

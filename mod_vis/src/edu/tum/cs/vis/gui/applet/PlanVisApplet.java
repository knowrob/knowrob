package edu.tum.cs.vis.gui.applet;

import java.awt.Color;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedList;

import javax.vecmath.Vector2f;

import edu.tum.cs.vis.action.Action;

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
	
	@Override
	public void setup()
	{
		size(800, 600, P2D);
		if (this.frame != null)
		{
		    this.frame.setTitle("Action plans visualisation");
		    this.frame.setBackground(new Color(10, 10, 10));
		}
		
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
	    
	    
	    drawCurrAction();
	    
	}
	
	public void setMainAction(Action action)
	{
		mainAction = action;
		currAction = action;
		this.redraw();
	}
	
	private void roundArrow(float x, float y, float radius, float width)
	{
		beginShape();
		
		curveVertex(x-radius, y+1);
		curveVertex(x-radius, y);
		curveVertex(x-(float)Math.cos(Math.PI/4)*radius, y-(float)Math.sin(Math.PI/4)*radius);
		curveVertex(x, y-radius);
		curveVertex(x+(float)Math.cos(Math.PI/4)*radius, y-(float)Math.sin(Math.PI/4)*radius);
		curveVertex(x+radius, y);
		curveVertex(x+radius, y+1);
		
		curveVertex(x+radius+width,y);
		curveVertex(x+radius+width+1,y);
		//vertex(x+radius, y);
		//vertex(x+radius+width,y);

		/*curveVertex(x+radius+width, y);
		curveVertex(x, y-radius-width);
		curveVertex(x-radius-width, y);
		curveVertex(x-radius-width, y);*/

		//vertex(x-radius-width, y);
		//vertex(x-radius,y);
				
		endShape(CLOSE);
	}
	
	private void drawCurrAction()
	{
		if (currAction == null)
			return;
		//currAction.getDrawInfo().drawSimpleBox(this, new Vector2f(50,50),0);
		currAction.getDrawInfo().drawExtendedBox(this, new Vector2f(50,50));
		/*stroke(borderColor.getRed(), borderColor.getBlue(), borderColor.getGreen(), borderColor.getAlpha());
	    fill(backgroundColor.getRed(), backgroundColor.getBlue(), backgroundColor.getGreen(), backgroundColor.getAlpha());
		roundArrow(500,100,50,20);
		*/
	}
}

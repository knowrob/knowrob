package edu.tum.cs.ias.knowrob.vis.items;

import processing.core.PFont;
import edu.tum.cs.ias.knowrob.vis.AnimatedCanvas;
import edu.tum.cs.ias.knowrob.vis.Canvas;
import edu.tum.cs.ias.knowrob.vis.Drawable;

public class FrameDisplay implements Drawable {

	protected AnimatedCanvas canvas;
	protected int x, y, textAlign;
	protected PFont font;
	
	/**
	 * 
	 * @param c
	 * @param x
	 * @param y
	 * @param textAlign text alignment (use one of the members of Canvas, e.g. LEFT or RIGHT)
	 * @param font  may be null if it is not to be explicitly set
	 */
	public FrameDisplay(AnimatedCanvas c, int x, int y, int textAlign, PFont font) {
		this.canvas = c;
		this.x = x;
		this.y = y;
		this.textAlign = textAlign;
		this.font = font;
	}
	
	public void draw(Canvas c) {
		int numSteps = canvas.getMaxAnimationStep()+1;
		if(numSteps == 1) // don't draw frame display if there is only one frame in the sequence
			return;
		if(font != null)
			c.textFont(font);
		c.fill(0xffffffff);
		c.textMode(Canvas.SCREEN);
		c.textAlign(textAlign);
		c.text(String.format("%d/%d", canvas.getAnimationStep()+1, numSteps), x, y);
	}
}

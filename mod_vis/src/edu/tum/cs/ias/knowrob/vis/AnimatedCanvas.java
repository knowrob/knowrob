package edu.tum.cs.ias.knowrob.vis;

import java.util.Vector;

public class AnimatedCanvas extends Canvas {

	private static final long serialVersionUID = 1L;
	
	protected int animationStep = 0;
	//maxAnimationStep = 0;
	protected Vector<DrawableAnimated> animatedItems = new Vector<DrawableAnimated>();

	@Override
	public void keyPressed() {
		super.keyPressed();
		switch(keyCode) {
		case RIGHT:
			animationStepForward();
			break;
		case LEFT:
			animationStepBackward();
			break;			
		case java.awt.event.KeyEvent.VK_HOME:
			animationReset();
			break;
		case java.awt.event.KeyEvent.VK_END:
			animationEnd();
			break;
		case java.awt.event.KeyEvent.VK_PAGE_UP:
			animationSkip(10);
			break;
		case java.awt.event.KeyEvent.VK_PAGE_DOWN:
			animationSkip(-10);
			break;
		}
	}
	
	public synchronized int getMaxAnimationStep() {
		int max = 0;
		for(DrawableAnimated item : animatedItems) {
			max = Math.max(max, item.getMaxStep());
		}
		return max;
	}
	
	public void animationStepForward() {
		if(animationStep < getMaxAnimationStep())
			animationStep++;
		redraw();
	}
	
	public void animationStepBackward() {
		if(animationStep > 0)
			animationStep--;
		redraw();
	}
	
	public void animationReset() {
		animationStep = 0;
		redraw();
	}
	
	public void animationEnd() {
		animationStep = getMaxAnimationStep();
		redraw();
	}

	/**
	 * skips a number of steps forward or backward
	 * @param steps
	 */
	public void animationSkip(int steps) {
		animationStep += steps;
		int maxStep = getMaxAnimationStep();
		if(animationStep > maxStep)
			animationStep = maxStep;
		if(animationStep < 0)
			animationStep = 0;
		redraw();
	}
	
	@Override
	public synchronized void drawItems() {
		super.drawItems();
		
		for(DrawableAnimated d : animatedItems) {
			d.draw(this, animationStep);
		}
	}
	
	public synchronized void addAnimated(DrawableAnimated item) {
		//maxAnimationStep = Math.max(maxAnimationStep, item.getMaxStep());
		animatedItems.add(item);
	}
	
	public int getAnimationStep() {
		return this.animationStep;
	}
	
	public Vector<DrawableAnimated> getAnimatedItems() {
		return animatedItems;
	}
}

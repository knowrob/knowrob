/*
 * Created on Oct 19, 2009
 *
 * TODO To change the template for this generated file go to
 * Window - Preferences - Java - Code Style - Code Templates
 */
package edu.tum.cs.ias.knowrob.vis.items;

import java.util.Vector;

import edu.tum.cs.ias.knowrob.vis.Canvas;
import edu.tum.cs.ias.knowrob.vis.Drawable;
import edu.tum.cs.ias.knowrob.vis.DrawableAnimated;
import edu.tum.cs.ias.knowrob.vis.items.transform.DrawingTransformation;

/**
 * a generic drawable item, which may be animated by changing the transformation that is applied to it over time.
 * The transformation is assumed to be applicable prior to the rendering of the object itself, which may otherwise be drawn in the same fashion at all times 
 *
 * @author jain
 */
public abstract class GenericItem<T extends DrawingTransformation> implements Drawable, DrawableAnimated {
	
	protected Vector<T> animation = new Vector<T>();
	
	public GenericItem(T transform) {
		animation.add(transform);
	}
	
	public boolean isAnimated() {
		return animation.size() > 0;
	}
	
	public void draw(Canvas c) {
		draw(c, 0);
	}
	
	public void draw(Canvas c, int step) {
		T currentTransform; 
		try {
			currentTransform = animation.get(step);
		}
		catch(ArrayIndexOutOfBoundsException e) {
			currentTransform = animation.lastElement();
		}
		c.pushMatrix();
		currentTransform.applyTransformation(c);
		drawActualItem(c);
		c.popMatrix();
	}
	
	public int getMaxStep() {
		return animation.size()-1;
	}

	protected abstract void drawActualItem(Canvas c);	
	
	public void setFrame(int frame, T transformation) {
		setFrames(frame, frame, transformation);
	}
	
	/**
	 * sets the transformation for several frames. The frames being addressed need not exist already.
	 * If the first frame is beyond the current animation, the current last frame is repeated until firstFrame-1 
	 * @param from 0-based index of the first frame to be changed
	 * @param to last frame to be changed; -1 for "until the end".
	 */
	public void setFrames(int from, int to, T transformation) {
		int lastFrame = animation.size()-1;
		if(to == -1)
			to = Math.max(to, lastFrame);
		makeAnimated(from-1);		
		if(lastFrame < from) 
			animation.add(transformation);
		else {
			int resetTo = Math.min(lastFrame, to);
			for(int i = from; i < resetTo; i++)
				animation.set(i, transformation);
		}
		makeAnimated(to);
	}
	
	/**
	 * sets the transformation for several frames by replacing the current transformation based on the replacer that is given.
	 * The interval specified need not exist already (if it doesn't the last frame is replicated by reference accordingly).
	 * @param from 0-based index of the first frame to be changed
	 * @param to last frame to be changed; -1 for "until the end".
	 */
	public void setFrames(int from, int to, PartialTransformationReplacer<T> ptr) {
		if(to == -1) 
			to = Math.max(from, animation.size()-1);
		makeAnimated(to);
		
		T prev = null;
		for(int i = from; i <= to; i++) {			
			T newT = ptr.replace(animation.get(i));			
			if(prev != null && newT.equals(prev))
				animation.set(i, prev);
			else {
				animation.set(i, newT);
				prev = newT;
			}
		}
	}
	
	/**
	 * abstract base class for transformation replacement (for use with setFrames)
	 * @author jain
	 */
	public static abstract class PartialTransformationReplacer<T extends DrawingTransformation> {
		T replacement;

		public PartialTransformationReplacer(T replacement) {
			this.replacement = replacement;
		}

		/**
		 * constructs a new transformation by partially replacing the content of 'before' with data
		 * from the 'replacement' member given at construction 
		 * @param before
		 * @return
		 */
		public abstract T replace(T before);
	}
	
	/**
	 * ensures that the animation for this item is defined up to the given frame
	 * by repeating the current last frame's transformation (by reference) as many times as necessary 
	 * @param untilFrame 0-based index up to which the animation must be defined
	 */
	protected void makeAnimated(int untilFrame) {
		if(untilFrame < animation.size())
			return;
		T transform = this.animation.lastElement();
		for(int i = animation.size()-1; i <= untilFrame; i++)
			animation.add(transform);
	}
}

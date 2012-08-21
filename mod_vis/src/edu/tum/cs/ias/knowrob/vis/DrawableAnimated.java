package edu.tum.cs.ias.knowrob.vis;

public interface DrawableAnimated {
	/** 
	 * @param c  the canvas to draw to
	 * @param step  the animation step/frame to draw (0-based index)
	 */
	public void draw(Canvas c, int step);
	/**
	 * the maximum frame (0-based index) that can be drawn
	 * @return
	 */
	public int getMaxStep();
}

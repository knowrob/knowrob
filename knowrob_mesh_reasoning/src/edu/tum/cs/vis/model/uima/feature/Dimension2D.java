package edu.tum.cs.vis.model.uima.feature;

import edu.tum.cs.uima.Feature;

/**
 * Dimension of a surface/object in 2D. Eg used for flat surfaces.
 * 
 * @author Stefan Profanter
 * 
 */
public class Dimension2D extends Feature {
	private int	width;
	private int	height;

	/**
	 * @return the height
	 */
	public int getHeight() {
		return height;
	}

	/**
	 * @return the width
	 */
	public int getWidth() {
		return width;
	}

	/**
	 * @param height
	 *            the height to set
	 */
	public void setHeight(int height) {
		this.height = height;
	}

	/**
	 * @param width
	 *            the width to set
	 */
	public void setWidth(int width) {
		this.width = width;
	}

}

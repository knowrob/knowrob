package edu.tum.cs.vis.model.uima.feature;

import edu.tum.cs.uima.Feature;

/**
 * Area of a surface projected into 2D. All coordinates are in Millimeters.
 * 
 * @author Stefan Profanter
 * 
 */
public class Area extends Feature {
	private float	squareMM;

	/**
	 * @return the squareMM
	 */
	public float getSquareMM() {
		return squareMM;
	}

	/**
	 * @param squareMM
	 *            the squareMM to set
	 */
	public void setSquareMM(float squareMM) {
		this.squareMM = squareMM;
	}

}

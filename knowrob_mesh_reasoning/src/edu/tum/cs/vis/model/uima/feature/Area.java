/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors:
 *     Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.feature;

import edu.tum.cs.uima.Feature;

/**
 * Area of a surface projected into 2D. All coordinates are in Millimeters.
 * 
 * @author Stefan Profanter
 * 
 */
public class Area extends Feature {
	/**
	 * area of this feature in square millimeters
	 */
	private double	squareMM;

	/**
	 * @return the squareMM
	 */
	public double getSquareMM() {
		return squareMM;
	}

	/**
	 * @param squareMM
	 *            the squareMM to set
	 */
	public void setSquareMM(float squareMM) {
		this.squareMM = squareMM;
	}

	@Override
	public String toString() {
		return squareMM + "m²";
	}

}

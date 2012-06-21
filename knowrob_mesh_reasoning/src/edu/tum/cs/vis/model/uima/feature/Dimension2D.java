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
 * Dimension of a surface/object in 2D. Eg used for flat surfaces.
 * 
 * @author Stefan Profanter
 * 
 */
public class Dimension2D extends Feature {
	/**
	 * Width of this feature in millimeters
	 */
	private double	width;

	/**
	 * height of this feature in millimeters
	 */
	private double	height;

	/**
	 * @return the height
	 */
	public double getHeight() {
		return height;
	}

	/**
	 * @return the width
	 */
	public double getWidth() {
		return width;
	}

	/**
	 * @param height
	 *            the height to set
	 */
	public void setHeight(double height) {
		this.height = height;
	}

	/**
	 * @param width
	 *            the width to set
	 */
	public void setWidth(double width) {
		this.width = width;
	}

	@Override
	public String toString() {
		return String.format("%.2fx%.2fmm", width, height);
	}

}

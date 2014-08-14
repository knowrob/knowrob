/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.util;

import java.awt.Color;
import java.io.Serializable;

import processing.core.PImage;

/**
 * Appearance of triangle or line. May contain texture or simply a color.
 * 
 * @author Stefan Profanter
 * 
 */
public class Appearance implements Serializable {
	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= 3878380145216484383L;

	/**
	 * the Filename of the texture-Image
	 */
	private String				imageFileName;

	/**
	 * an Object reference of a loaded image. Default is null.
	 */
	private PImage				imageReference;

	/**
	 * the color of the Triangle if the Triangle has no texture Set to null if no fill needed
	 */
	private Color				colorFill;

	/**
	 * the color stroke for Triangle or Line Set to null if no stroke needed
	 */
	private Color				colorLine;

	/**
	 * Thickness of the stroke
	 */
	private int					strokeWeight;

	/**
	 * Empty constructor for appearance objects
	 */
	public Appearance() {
		imageFileName = null;
		imageReference = null;
		colorFill = null;
		colorLine = null;
		strokeWeight = 2;
	};

	/**
	 * Copy constructor for the Appearance objects
	 * 
	 * @param toCopy
	 *            object to copy to newly created instance
	 */
	public Appearance(final Appearance toCopy) {
		if (toCopy.colorFill != null) {
			this.colorFill = toCopy.colorFill;
		}
		if (toCopy.colorLine != null) {
			this.colorLine = toCopy.colorLine;
		}
		if (toCopy.imageFileName != null) {
			this.imageFileName = toCopy.imageFileName;
		}
		if (toCopy.imageReference != null) {
			this.imageReference = toCopy.imageReference;
		}
	}

	/**
	 * @return the colorFill
	 */
	public Color getColorFill() {
		return colorFill;
	}

	/**
	 * @return the colorLine
	 */
	public Color getColorLine() {
		return colorLine;
	}

	/**
	 * @return the imageFileName
	 */
	public String getImageFileName() {
		return imageFileName;
	}

	/**
	 * @return the imageReference
	 */
	public PImage getImageReference() {
		return imageReference;
	}

	/**
	 * @return the strokeWeight
	 */
	public int getStrokeWeight() {
		return strokeWeight;
	}

	/**
	 * @param colorFill
	 *            the colorFill to set
	 */
	public void setColorFill(Color colorFill) {
		this.colorFill = colorFill;
	}

	/**
	 * @param colorLine
	 *            the colorLine to set
	 */
	public void setColorLine(Color colorLine) {
		this.colorLine = colorLine;
	}

	/**
	 * @param imageFileName
	 *            the imageFileName to set
	 */
	public void setImageFileName(String imageFileName) {
		this.imageFileName = imageFileName;
	}

	/**
	 * @param imageReference
	 *            the imageReference to set
	 */
	public void setImageReference(PImage imageReference) {
		this.imageReference = imageReference;
	}

	/**
	 * @param strokeWeight
	 *            the strokeWeight to set
	 */
	public void setStrokeWeight(int strokeWeight) {
		this.strokeWeight = strokeWeight;
	}
}

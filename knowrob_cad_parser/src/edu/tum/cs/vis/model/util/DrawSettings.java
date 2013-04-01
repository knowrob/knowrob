/*******************************************************************************
 * Copyright (c) 2013 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2013
 ******************************************************************************/
package edu.tum.cs.vis.model.util;

import java.awt.Color;

/**
 * General settings used for drawing triangles, lines and vertices. This class is used to override
 * the default appearance of a triangle.
 * 
 * @author Stefan Profanter
 * 
 */
public class DrawSettings {
	/**
	 * Default constructor. Initializes DrawSettings with default values which don't override the
	 * appearance.
	 */
	public DrawSettings() {
		this.drawType = DrawType.FILL;
		overrideColor = null;
	}

	/**
	 * Override color. If set, it overrides the appearance color.
	 */
	private Color	overrideColor;
	/**
	 * Type of drawing (only for triangles): filled, lines, points
	 */
	public DrawType	drawType;
	/**
	 * Line width for draw type lines
	 */
	private int		lineWidth	= 1;
	/**
	 * Forces the annotation or triangle to draw, even if it is unselected from the annotation list.
	 */
	public boolean	forceDraw	= false;

	/**
	 * Increase the line width for draw type lines.
	 */
	public void incLineWidth() {
		lineWidth++;
	}

	/**
	 * Decrease the line width for draw type lines (minimum is 1).
	 */
	public void decLineWidth() {
		if (lineWidth == 1)
			return;
		lineWidth--;
	}

	/**
	 * Returns the current line width.
	 * 
	 * @return current line width.
	 */
	public int getLineWidth() {
		return lineWidth;
	}

	/**
	 * @param overrideColor
	 *            the overrideColor to set
	 */
	public void setOverrideColor(Color overrideColor) {
		this.overrideColor = overrideColor;
	}

	/**
	 * @param lineWidth
	 *            the lineWidth to set
	 */
	public void setLineWidth(int lineWidth) {
		this.lineWidth = lineWidth;
	}

	@Override
	public Object clone() {
		DrawSettings ds = new DrawSettings();
		ds.overrideColor = overrideColor == null ? null : new Color(overrideColor.getRed(),
				overrideColor.getGreen(), overrideColor.getBlue(), overrideColor.getAlpha());
		ds.drawType = drawType;
		ds.lineWidth = lineWidth;
		ds.forceDraw = forceDraw;
		return ds;

	}

	/**
	 * Clones the calling draw settings object and sets the override color of the cloned object to
	 * the given one. The cloned object is then returned.
	 * 
	 * @param overrideColor
	 *            The new override color for the cloned object.
	 * @return A cloned copy of this, where override color is set to the provided one.
	 */
	public DrawSettings getTemporaryOverride(Color overrideColor) {
		DrawSettings ds = (DrawSettings) this.clone();
		ds.overrideColor = overrideColor;
		return ds;
	}

	/**
	 * @return the overrideColor
	 */
	public Color getOverrideColor() {
		return overrideColor;
	}

}

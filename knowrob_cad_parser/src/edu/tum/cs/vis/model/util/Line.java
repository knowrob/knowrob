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
package edu.tum.cs.vis.model.util;

import java.awt.Color;

import processing.core.PGraphics;

/**
 * A simple line of a model
 * 
 * @author Stefan Profanter
 * 
 */
public class Line extends DrawObject {

	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= 6189622478733284349L;

	/**
	 * Default constructor
	 */
	public Line() {
		super(2);
	}

	/**
	 * Draw the line onto the applet.
	 * 
	 * @param g
	 *            Applet to draw on
	 * @param overrideColor
	 *            override the draw color an texture. Draw whole object in the given color if !=
	 *            null
	 */
	public void draw(PGraphics g, Color overrideColor) {
		applyColor(g, overrideColor);
		g.line(position[0].x, position[0].y, position[0].z, position[1].x, position[1].y,
				position[1].z);
	}
}

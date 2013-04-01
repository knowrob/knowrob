/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.util;

import processing.core.PGraphics;

/**
 * A simple line of a CAD model which connects two vertices and has a specific color/appearance.
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
	 * @param drawSettings
	 *            override the draw color, texture (and other settings). Draw whole object in the
	 *            given color if != null
	 */
	public void draw(PGraphics g, DrawSettings drawSettings) {
		applyColor(g, drawSettings);
		if (drawSettings != null && drawSettings.drawType == DrawType.POINTS) {
			for (int i = 0; i < position.length; i++) {
				if (position[i].overrideColor != null) {
					g.stroke(position[i].overrideColor.getRed(),
							position[i].overrideColor.getGreen(),
							position[i].overrideColor.getBlue());
					g.noFill();
				} else if (drawSettings.getOverrideColor() == null && position[i].color != null) {
					g.stroke(position[i].color.getRed(), position[i].color.getGreen(),
							position[i].color.getBlue());
					g.noFill();
				}
				g.point(position[i].x, position[i].y, position[i].z);
			}
		} else
			g.line(position[0].x, position[0].y, position[0].z, position[1].x, position[1].y,
					position[1].z);
	}
}

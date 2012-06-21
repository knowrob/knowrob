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

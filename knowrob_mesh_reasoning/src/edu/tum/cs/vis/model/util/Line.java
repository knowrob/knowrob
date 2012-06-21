package edu.tum.cs.vis.model.util;

import java.awt.Color;

import javax.vecmath.Point3f;

import processing.core.PApplet;

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
		position = new Point3f[2];
	}

	/**
	 * Draw the line onto the applet.
	 * 
	 * @param applet
	 *            Applet to draw on
	 * @param overrideColor
	 *            override the draw color an texture. Draw whole object in the given color if !=
	 *            null
	 */
	public void draw(PApplet applet, Color overrideColor) {
		applyColor(applet, overrideColor);
		applet.line(position[0].x, position[0].y, position[0].z, position[1].x, position[1].y,
				position[1].z);
	}
}

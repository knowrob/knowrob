package edu.tum.cs.vis.model.util;

import javax.vecmath.Point3f;

import processing.core.PApplet;

public class Line {
	/**
	 * the 2 Vertices of the Line
	 */
	public Point3f position[];
	public Appearance appearance;

	public Line() {
		position = new Point3f[2];
	}

	/**
	 * Draw the triangle onto the applet.
	 * 
	 * @param applet
	 *            Applet to draw on
	 */
	public void draw(PApplet applet) {
		applet.stroke(appearance.colour.red * 255,
				appearance.colour.green * 255, appearance.colour.blue * 255);
		applet.strokeWeight(2);
		applet.line(position[0].x, position[0].y, position[0].z, position[1].x,
				position[1].y, position[1].z);
	}
}

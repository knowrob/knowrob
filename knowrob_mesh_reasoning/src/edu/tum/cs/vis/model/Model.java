/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model;

import java.awt.Color;
import java.util.ArrayList;

import javax.vecmath.Point3f;

import processing.core.PGraphics;
import edu.tum.cs.vis.model.util.Group;
import edu.tum.cs.vis.model.util.Line;
import edu.tum.cs.vis.model.util.Triangle;

/**
 * @author Stefan Profanter
 * 
 */
public class Model {

	private Group						group;

	private final ArrayList<Point3f>	vertices	= new ArrayList<Point3f>();
	private final ArrayList<Triangle>	triangles	= new ArrayList<Triangle>();
	private final ArrayList<Line>		lines		= new ArrayList<Line>();

	/**
	 * @param line
	 */
	public void addLine(Line line) {
		for (int i = 0; i < line.getPosition().length; i++) {
			line.getPosition()[i] = checkOrAddVertice(line.getPosition()[i]);
		}
		lines.add(line);
	}

	/**
	 * @param tri
	 */
	public void addTriangle(Triangle tri) {
		for (int i = 0; i < tri.getPosition().length; i++) {
			tri.getPosition()[i] = checkOrAddVertice(tri.getPosition()[i]);
		}
		triangles.add(tri);

	}

	private Point3f checkOrAddVertice(Point3f v) {

		for (Point3f p : vertices) {
			if (p.equals(v))
				return p;
		}
		vertices.add(v);
		return v;
	}

	/**
	 * @param g
	 * @param overrideColor
	 */
	public void draw(PGraphics g, Color overrideColor) {
		if (group != null)
			group.draw(g, overrideColor);
	}

	/**
	 * @return the group
	 */
	public Group getGroup() {
		return group;
	}

	/**
	 * @return the lines
	 */
	public ArrayList<Line> getLines() {
		return lines;
	}

	/**
	 * @return the triangles
	 */
	public ArrayList<Triangle> getTriangles() {
		return triangles;
	}

	/**
	 * @return the vertices
	 */
	public ArrayList<Point3f> getVertices() {
		return vertices;
	}

	/**
	 * @param group
	 *            the group to set
	 */
	public void setGroup(Group group) {
		this.group = group;
	}

}

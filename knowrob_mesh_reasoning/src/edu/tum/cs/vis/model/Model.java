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
	private void addLine(Line line) {
		for (int i = 0; i < line.getPosition().length; i++) {
			line.getPosition()[i] = checkOrAddVertex(line.getPosition()[i]);
		}
		lines.add(line);
	}

	/**
	 * @param tri
	 */
	private void addTriangle(Triangle tri) {
		for (int i = 0; i < tri.getPosition().length; i++) {
			tri.getPosition()[i] = checkOrAddVertex(tri.getPosition()[i]);
		}
		triangles.add(tri);

	}

	private Point3f checkOrAddVertex(Point3f v) {

		for (Point3f p : vertices) {
			if (p.x == v.x && p.y == v.y && p.z == v.z)
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
	 * 
	 */
	public void modelChanged() {
		vertices.clear();
		triangles.clear();
		lines.clear();
		processGroup(group);
	}

	private void processGroup(Group g) {
		for (Line l : g.getMesh().getLines())
			addLine(l);
		for (Triangle t : g.getMesh().getTriangles())
			addTriangle(t);
		for (Group c : g.getChildren())
			processGroup(c);
	}

	/**
	 * @param group
	 *            the group to set
	 */
	public void setGroup(Group group) {
		this.group = group;
		modelChanged();
	}
}

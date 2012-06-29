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
import java.util.Arrays;

import javax.vecmath.Point3f;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3f;

import processing.core.PGraphics;
import edu.tum.cs.vis.model.util.BSphere;
import edu.tum.cs.vis.model.util.Group;
import edu.tum.cs.vis.model.util.Line;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.Vertex;

/**
 * @author Stefan Profanter
 * 
 */
public class Model {

	private String					textureBasePath	= null;

	private Group					group;

	private final ArrayList<Vertex>	vertices		= new ArrayList<Vertex>();

	final ArrayList<Triangle>		triangles		= new ArrayList<Triangle>();

	private final ArrayList<Line>	lines			= new ArrayList<Line>();

	private BSphere					boundingSphere	= null;

	private float					scale			= 1;

	/**
	 * @param g
	 * @param overrideColor
	 */
	public void draw(PGraphics g, Color overrideColor) {
		if (group != null)
			group.draw(g, overrideColor);
	}

	/**
	 * A characteristic "feature size" for the mesh. Computed as an approximation to the median edge
	 * length
	 * 
	 * @return
	 */
	float feature_size() {

		int nf = triangles.size();
		int nsamp = Math.min(nf / 2, 333);

		float samples[] = new float[nsamp * 3];
		int idx = 0;

		for (int i = 0; i < nsamp; i++) {
			// Quick 'n dirty portable random number generator
			int ind = (int) (Math.random() * nf);

			Point3f p0 = triangles.get(ind).getPosition()[0];
			Point3f p1 = triangles.get(ind).getPosition()[1];
			Point3f p2 = triangles.get(ind).getPosition()[2];

			Vector3f f1 = new Vector3f(p0);
			f1.sub(p1);
			samples[idx++] = f1.lengthSquared();

			Vector3f f2 = new Vector3f(p1);
			f2.sub(p2);
			samples[idx++] = f2.lengthSquared();

			Vector3f f3 = new Vector3f(p2);
			f3.sub(p0);
			samples[idx++] = f3.lengthSquared();
		}
		Arrays.sort(samples);
		return (float) Math.sqrt(samples[samples.length / 2]);
	}

	/**
	 * @return the boundingSphere
	 */
	public BSphere getBoundingSphere() {
		return boundingSphere;
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

	public float getScale() {
		return scale;
	}

	/**
	 * @return the textureBasePath
	 */
	public String getTextureBasePath() {
		return textureBasePath;
	}

	/**
	 * @return the triangles
	 */
	public ArrayList<Triangle> getTriangles() {
		return triangles;
	}

	public float getUnscaled(float scaled) {
		return scaled * 1f / scale;
	}

	/**
	 * @param corner
	 * @return
	 */
	public Tuple3f[] getUnscaled(Point3f[] corner) {
		Tuple3f[] ret = new Point3f[corner.length];
		for (int i = 0; i < corner.length; i++) {
			ret[i] = getUnscaled(corner[i]);
		}
		return ret;
	}

	public Tuple3f getUnscaled(Tuple3f t) {
		Tuple3f tr = new Point3f(t);
		tr.scale(1f / scale);
		return tr;
	}

	public Vector3f getUnscaled(Vector3f t) {
		Vector3f tr = new Vector3f(t);
		tr.scale(1f / scale);
		return tr;
	}

	/**
	 * @return the vertices
	 */
	public ArrayList<Vertex> getVertices() {
		return vertices;
	}

	/**
	 * 
	 */
	public void mirrorX() {
		for (Vertex v : vertices) {
			v.x *= (-1);
		}
		group.resetMinMaxValues();

	}

	public float normalize() {
		float x = group.getMaxX() - group.getMinX();
		float y = group.getMaxY() - group.getMinY();
		float z = group.getMaxZ() - group.getMinZ();

		float max = Math.max(x, Math.max(y, z));

		scale(1f / max);

		scale = 1f / max;
		return max;
	}

	/**
	 * @param meter
	 */
	public void scale(float factor) {

		for (Vertex v : vertices) {
			v.scale(factor);
		}
		for (Triangle t : triangles)
			t.updateCentroid();
		for (Line l : lines)
			l.updateCentroid();
		group.resetMinMaxValues();
	}

	/**
	 * @param boundingSphere
	 *            the boundingSphere to set
	 */
	public void setBoundingSphere(BSphere boundingSphere) {
		this.boundingSphere = boundingSphere;
	}

	/**
	 * @param group
	 *            the group to set
	 */
	public void setGroup(Group group) {
		this.group = group;
	}

	/**
	 * @param textureBasePath
	 *            the textureBasePath to set
	 */
	public void setTextureBasePath(String textureBasePath) {
		this.textureBasePath = textureBasePath;
	}

}

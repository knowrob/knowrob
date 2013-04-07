/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.util;

import java.awt.Color;
import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

import javax.vecmath.Point3f;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3f;

/**
 * A vertex (corner point) of a triangle or line. Vertex may have normal vector and voronoi area
 * assigned.
 * 
 * 
 * @author Stefan Profanter
 * 
 */
public class Vertex extends Point3f {

	/**
	 * 
	 */
	private static final long	serialVersionUID	= 4454667509075960402L;

	/**
	 * normal vector of vertex
	 */
	private Vector3f			normalVector		= new Vector3f();

	/**
	 * voronoi area of vertex
	 */
	private float				pointarea			= 0f;

	/**
	 * Color of vertex. May be used to color vertex instead of triangle.
	 */
	public Color				color;

	/**
	 * Overrides color of triangle with this color.
	 */
	public Color				overrideColor		= null;

	/**
	 * List of direct neighbors of vertex
	 */
	private final Set<Vertex>	neighbors			= new HashSet<Vertex>();

	/**
	 * Constructor for vertex
	 * 
	 * @param x
	 *            x coordinate
	 * @param y
	 *            y coordinate
	 * @param z
	 *            z coordinate
	 */
	public Vertex(float x, float y, float z) {
		super(x, y, z);
	}

	/**
	 * Constructor for vertex
	 * 
	 * @param p
	 *            coordinates for new vertex
	 */
	public Vertex(Tuple3f p) {
		super(p);
	}

	/**
	 * Create a new vertex at given point <code>p</code> with given normal vector <code>norm</code>
	 * 
	 * @param p
	 *            point of the vertex
	 * @param norm
	 *            normal vector of the vertex
	 */
	public Vertex(Tuple3f p, Vector3f norm) {
		super(p);
		this.normalVector = (Vector3f) norm.clone();
	}

	@Override
	public Object clone() {
		Vertex v = new Vertex(this);
		v.normalVector = (Vector3f) normalVector.clone();
		v.color = color == null ? null : new Color(color.getRed(), color.getGreen(),
				color.getBlue(), color.getAlpha());
		v.overrideColor = overrideColor == null ? null : new Color(overrideColor.getRed(),
				overrideColor.getGreen(), overrideColor.getBlue(), overrideColor.getAlpha());
		return v;
	}

	@Override
	public boolean equals(Object o) {
		if (o == this) {
			return true;
		}

		return false;
	}

	/**
	 * Check if p has same coordinates as this vertex.
	 * 
	 * @param p
	 *            point
	 * @return true if x,y,z are equal
	 */
	public boolean sameCoordinates(Point3f p) {
		return (p.x == x && p.y == y && p.z == z);
	}

	/**
	 * Get normal vector of vertex
	 * 
	 * @return the normalVector
	 */
	public Vector3f getNormalVector() {
		return normalVector;
	}

	/**
	 * Get voronoi area of vertex
	 * 
	 * @return the pointarea
	 */
	public float getPointarea() {
		return pointarea;
	}

	@Override
	public int hashCode() {
		return Float.valueOf(x).hashCode() ^ Float.valueOf(y).hashCode()
				^ Float.valueOf(z).hashCode() ^ Double.valueOf(pointarea).hashCode()
				^ normalVector.hashCode();
	}

	/**
	 * set normal vector of vertex
	 * 
	 * @param normalVector
	 *            the normalVector to set
	 */
	public void setNormalVector(Vector3f normalVector) {
		this.normalVector = normalVector;
	}

	/**
	 * set Voronoi area of vertex
	 * 
	 * @param pointarea
	 *            the pointarea to set
	 */
	public void setPointarea(float pointarea) {

		this.pointarea = pointarea;
	}

	/**
	 * Apply 4x4 transformation matrix to the vector
	 * 
	 * @param matrix
	 *            the transformation matrix
	 */
	public void transform(float[][] matrix) {

		float[] newPos = new float[4];
		for (int row = 0; row < 4; row++) {
			newPos[row] = x * matrix[row][0] + y * matrix[row][1] + z * matrix[row][2]
					+ matrix[row][3];
		}
		x = newPos[0] / newPos[3];
		y = newPos[1] / newPos[3];
		z = newPos[2] / newPos[3];
	}

	/**
	 * @return the neighbors
	 */
	public Set<Vertex> getNeighbors() {
		return neighbors;
	}

	/**
	 * Add neighbor vertex to the neighbors list. You have to check, that v is really a direct
	 * neighbor (this one is connected to v through a single edge).
	 * 
	 * @param v
	 *            vertex to add
	 */
	public void addNeighbor(Vertex v) {
		if (v == this)
			return;
		synchronized (neighbors) {
			neighbors.add(v);
		}
	}

	/**
	 * Add list of vertices as neighbors. You have to check, that v is really a direct neighbor
	 * (this one is connected to v through a single edge).
	 * 
	 * @param neig
	 *            list of neighbors to add
	 */
	public void addNeighbor(Collection<Vertex> neig) {
		synchronized (neighbors) {
			synchronized (neig) {
				neighbors.addAll(neig);
			}
			// make sure we didn't add ourself as neighbor
			if (neighbors.contains(this))
				neighbors.remove(this);
		}
	}

	/**
	 * Add list of vertices as neighbors. You have to check, that v is really a direct neighbor
	 * (this one is connected to v through a single edge).
	 * 
	 * @param neig
	 *            list of neighbors to add
	 */
	public void addNeighbor(Vertex[] neig) {
		synchronized (neighbors) {
			synchronized (neig) {
				for (Vertex v : neig) {
					if (v == this)
						continue;
					neighbors.add(v);
				}
			}
		}
	}

}

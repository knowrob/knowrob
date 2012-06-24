/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.util;

import java.awt.Color;

import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;

/**
 * @author Stefan Profanter
 * 
 */
public class Vertex extends Point3f {

	/**
	 * 
	 */
	private static final long	serialVersionUID	= 4454667509075960402L;

	private Vector3f			normalVector		= new Vector3f();

	private float				pointarea			= 0f;

	//private Curvature			curvature;

	public Color				color;

	/**
	 * @param f
	 * @param g
	 * @param h
	 */
	public Vertex(float x, float y, float z) {
		super(x, y, z);
	}

	public Vertex(Point3f p) {
		super(p);
	}

	@Override
	public boolean equals(Object o) {
		if (o == this) {
			return true;
		}
		if (!(o instanceof Point3f)) {
			return false;
		}

		Point3f p = (Point3f) o;
		Vertex v = (Vertex)o;
		return (p.x == x && p.y == y && p.z == z && v.pointarea == pointarea && v.normalVector.equals(normalVector));
	}

	/**
	 * @return the curvature
	 */
	/*public Curvature getCurvature() {
		return curvature;
	}*/

	/**
	 * @return the normalVector
	 */
	public Vector3f getNormalVector() {
		return normalVector;
	}

	/**
	 * @return the pointarea
	 */
	public float getPointarea() {
		return pointarea;
	}

	@Override
	public int hashCode() {
		return Float.valueOf(x).hashCode() ^ Float.valueOf(y).hashCode()
				^ Float.valueOf(z).hashCode() ^ Float.valueOf(pointarea).hashCode()^normalVector.hashCode();
	}

	/**
	 * @param curvature
	 *            the curvature to set
	 */
	/*public void setCurvature(Curvature curvature) {
		this.curvature = curvature;
	}*/

	/**
	 * @param normalVector
	 *            the normalVector to set
	 */
	public void setNormalVector(Vector3f normalVector) {
		this.normalVector = normalVector;
	}

	/**
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

}

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

	private Vector3f	normalVector	= new Vector3f();

	private float		pointarea		= 0f;

	private Curvature	curvature;

	public Color		color;

	/**
	 * @param f
	 * @param g
	 * @param h
	 */
	public Vertex(float x, float y, float z) {
		super(x, y, z);
	}

	/**
	 * @return the curvature
	 */
	public Curvature getCurvature() {
		return curvature;
	}

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

	/**
	 * @param curvature
	 *            the curvature to set
	 */
	public void setCurvature(Curvature curvature) {
		this.curvature = curvature;
	}

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

}

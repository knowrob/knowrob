/*******************************************************************************
 * Copyright (c) 2013 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2013
 ******************************************************************************/
package edu.tum.cs.vis.model.util;

import javax.vecmath.Point3f;

/**
 * Class which represents a triangle with intersection point.
 * 
 * @author Stefan Profanter
 * 
 */
public class IntersectedTriangle {
	/**
	 * Constructor for intersected triangle.
	 * 
	 * @param tri
	 *            triangle which is intersected
	 * @param intersect
	 *            intersection point on triangle
	 */
	public IntersectedTriangle(Triangle tri, Point3f intersect) {
		this.t = tri;
		this.intersection = intersect;
	}

	/**
	 * triangle which is intersected
	 */
	public Triangle	t;

	/**
	 * intersection point of triangle.
	 */
	public Point3f	intersection;

}
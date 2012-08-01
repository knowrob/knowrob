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
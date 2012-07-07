package edu.tum.cs.vis.model.util;

import javax.vecmath.Point3f;

public class IntersectedTriangle {
	public IntersectedTriangle(Triangle tri, Point3f intersect) {
		this.t = tri;
		this.intersection = intersect;
	}
	public Triangle	t;
	public Point3f	intersection;

}
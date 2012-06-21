package edu.tum.cs.vis.model.util;

import javax.vecmath.Point3f;

/**
 * Triangle of a model. May have color or texture.
 * 
 * @author Stefan Profanter
 *
 */
public class Triangle extends Polygon {

	/**
	 * Default constructor
	 */
	public Triangle() {
		position = new Point3f[3];
	}
	
	
}

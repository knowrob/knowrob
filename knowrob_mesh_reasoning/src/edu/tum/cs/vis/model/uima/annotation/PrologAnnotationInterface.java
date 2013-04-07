/*******************************************************************************
 * Copyright (c) 2013 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2013
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation;

import javax.vecmath.Matrix4f;

import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.Vertex;

/**
 * @author Stefan Profanter
 * 
 */
public interface PrologAnnotationInterface {
	/**
	 * Get area of primitive annotation by summing area of all triangles.
	 * 
	 * @return the area
	 * 
	 * @see PrologAnnotationInterface#getPrimitiveArea()
	 */
	public float getArea();

	/**
	 * Get value between > 0 for area coverage which indicates how good primitive annotation is fit
	 * into mesh. 1 indicates perfect fit, because area of triangles is exactly the same as area of
	 * primitive annotation.
	 * 
	 * @return value > 0
	 */
	public float getAreaCoverage();

	/**
	 * Get pose matrix for cone.
	 * 
	 * @return 4x4 pose matrix of the annotation relative to the object centroid
	 */
	public Matrix4f getPoseMatrix();

	/**
	 * Returns the total area of the primitive (total surface of sphere, cylinder, plane). The
	 * covered area (area of all triangles of annotation) is normally smaller than this area.
	 * 
	 * @return the total area of the primitive
	 */
	public abstract float getPrimitiveArea();

	public Triangle[] getTriangles();

	public Vertex[] getVertices();
}

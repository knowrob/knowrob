/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2013
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation.primitive;

import java.awt.Color;
import java.util.List;
import java.util.Map;
import java.util.Set;

import javax.vecmath.Matrix4f;
import javax.vecmath.Vector3f;

import processing.core.PGraphics;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.Vertex;

/**
 * @author Stefan Profanter
 * 
 */
public abstract class PrimitiveShape {

	/**
	 * Fit error for primitive shape
	 */
	protected float	fitError	= -1;

	/**
	 * Calculates fit error of fitted primitive w.r.t. annotation vertices.
	 * 
	 * @param vertices
	 *            Vertices of annotation
	 * @param weights
	 *            Weight for each vertex
	 * @param triangles
	 *            Triangles of annotation
	 */
	protected abstract void calculateFitError(Set<Vertex> vertices, Map<Vertex, Float> weights,
			List<Triangle> triangles);

	/**
	 * Draw primitive shape (cone, plane, sphere) on graphics context.
	 * 
	 * @param g
	 *            Graphics context to draw on.
	 * @param drawColor
	 *            draw color
	 */
	public abstract void draw(PGraphics g, Color drawColor);

	/**
	 * Fit primitive shape to given vertices and triangles.
	 * 
	 * @param centroid
	 *            Centroid of all vertices
	 * @param vertices
	 *            set of vertices to fit shape into
	 * @param weights
	 *            Weight of each vertex
	 * @param triangles
	 *            to fit shape into
	 * @return true if successfully fit.
	 */
	public abstract boolean fit(Vector3f centroid, Set<Vertex> vertices,
			Map<Vertex, Float> weights, List<Triangle> triangles);

	/**
	 * Get area of primitive shape.
	 * 
	 * @return Area of primitive shape.
	 */
	public abstract float getArea();

	/**
	 * Get fit error of primitive shape w.r.t. base vertices and triangles.
	 * 
	 * @return the fitError
	 */
	public float getFitError() {
		return fitError;
	}

	/**
	 * Get pose matrix of primitive shape.
	 * 
	 * @return the pose matrix.
	 */
	public abstract Matrix4f getPoseMatrix();

	/**
	 * Get volume of primitive shape.
	 * 
	 * @return the volume
	 */
	public abstract float getVolume();

}

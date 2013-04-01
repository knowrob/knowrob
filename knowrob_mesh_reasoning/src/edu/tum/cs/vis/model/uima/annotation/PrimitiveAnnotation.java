/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation;

import java.awt.Color;
import java.util.HashMap;
import java.util.HashSet;

import javax.vecmath.Vector3f;

import processing.core.PGraphics;
import edu.tum.cs.vis.model.Model;
import edu.tum.cs.vis.model.util.Curvature;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.Vertex;

/**
 * Mesh annotation for primitive types such as cone, cylinder, plane.
 * 
 * @author Stefan Profanter
 * @param <S>
 *            Type of primitive annotation
 * 
 */
@SuppressWarnings("rawtypes")
public abstract class PrimitiveAnnotation<S extends PrimitiveAnnotation> extends MeshAnnotation<S> {

	/**
	 * auto generated
	 */
	private static final long				serialVersionUID	= -7298994988518239919L;

	/**
	 * total area of annotation
	 */
	private float							area				= 0;

	/**
	 * Holds curvature properties for each vertex
	 */
	protected HashMap<Vertex, Curvature>	curvatures			= new HashMap<Vertex, Curvature>();

	/**
	 * Creates a new primitive annotation.
	 * 
	 * @param clazz
	 *            type of primitive annotation
	 * @param curvatures
	 *            curvature mapping for vertices
	 * @param model
	 *            main model
	 * @param annotationColor
	 *            predefined color of primitive annotation
	 */
	public PrimitiveAnnotation(Class<S> clazz, HashMap<Vertex, Curvature> curvatures, Model model,
			Color annotationColor) {
		super(clazz, model, annotationColor);
		this.curvatures = curvatures;
	}

	/**
	 * Draws the primitive annotation (= cone, plane, sphere) of this annotation.
	 * 
	 * @param g
	 *            graphics context to draw on.
	 */
	public void drawPrimitiveAnnotation(PGraphics g) {
		drawPrimitiveAnnotation(g, null);
	}

	/**
	 * Draws primitive annotation on given graphics context. Drawin primitive annotation means:
	 * drawing a sphere for sphere annotation or a cone for cone annotation, ...
	 * 
	 * @param g
	 *            graphics context
	 * @param color
	 *            draw color for drawing primitive annotation. may also be null to use default
	 *            values.
	 */
	public abstract void drawPrimitiveAnnotation(PGraphics g, Color color);

	/**
	 * Try to best fit parameters for primitive annotation into triangle mesh and update area of
	 * annotation.
	 * 
	 * @return true if annotation successfully fit. False if there was an error and the annotation
	 *         is most likely something else (eg. a plane)
	 */
	public boolean fit() {
		updateAnnotationArea();
		return fitAnnotation();
	}

	/**
	 * Try to best fit parameters for primitive annotation into triangle mesh.
	 * 
	 * @return true if annotation successfully fit. False if there was an error and the annotation
	 *         is most likely something else (eg. a plane)
	 */
	protected abstract boolean fitAnnotation();

	/**
	 * Get area of primitive annotation by summing area of all triangles.
	 * 
	 * @return the area
	 * 
	 * @see PrimitiveAnnotation#getPrimitiveArea()
	 */
	public float getArea() {
		if (area == 0)
			updateAnnotationArea();
		return area;
	}

	/**
	 * Get value between > 0 for area coverage which indicates how good primitive annotation is fit
	 * into mesh. 1 indicates perfect fit, because area of triangles is exactly the same as area of
	 * primitive annotation.
	 * 
	 * @return value > 0
	 */
	public float getAreaCoverage() {
		return getArea() / getPrimitiveArea();
	}

	/**
	 * Get total area of triangles unscaled.
	 * 
	 * @return sum of area of all triangles.
	 */
	public float getAreaUnscaled() {
		return model.getUnscaled(area);
	}

	/**
	 * Returns the total area of the primitive (total surface of sphere, cylinder, plane). The
	 * covered area (area of all triangles of annotation) is normally smaller than this area.
	 * 
	 * @return the total area of the primitive
	 */
	public abstract float getPrimitiveArea();

	/**
	 * Returns the total area (unscaled) of the primitive (total surface of sphere, cylinder,
	 * plane). The covered area (area of all triangles of annotation) is normally smaller than this
	 * area.
	 * 
	 * @return the total area of the primitive
	 */
	public abstract float getPrimitiveAreaUnscaled();

	/**
	 * Add all vertices of mesh to <tt>vertices</tt> and additionally calculate centroid of all
	 * vertices.
	 * 
	 * @param vertices
	 *            Empty set to add vertices of mesh.
	 * @return centroid of all vertices
	 */
	protected Vector3f getVertices(HashSet<Vertex> vertices) {
		vertices.clear();
		Vector3f centroid = new Vector3f();

		for (Triangle t : mesh.getTriangles()) {
			for (Vertex v : t.getPosition()) {
				if (vertices.contains(v))
					continue;

				centroid.add(v);
				vertices.add(v);
			}
		}

		int numberOfPoints = vertices.size();
		centroid.scale(1f / numberOfPoints);

		return centroid;

	}

	/**
	 * Add all vertices of mesh to <tt>vertices</tt> and add weight (voronoi area) for each vertex
	 * to set. Additionally calculate weighted centroid of all vertices.
	 * 
	 * @param vertices
	 *            Empty set to add vertices of mesh.
	 * @return weighted centroid of all vertices
	 */
	protected Vector3f getVerticesWithWeight(HashMap<Vertex, Float> vertices) {
		Vector3f centroid = new Vector3f();

		float totalArea = 0;

		for (Triangle t : mesh.getTriangles()) {
			float a = t.getArea();
			for (Vertex v : t.getPosition()) {
				float newArea = a / 3f;
				if (vertices.containsKey(v)) {
					newArea += vertices.get(v);
				}

				vertices.put(v, newArea);
			}
			// Calculate weighted centroid
			centroid.x += t.getCentroid().x * a;
			centroid.y += t.getCentroid().y * a;
			centroid.z += t.getCentroid().z * a;

			totalArea += a;
		}

		centroid.scale(1f / totalArea);

		return centroid;

	}

	/**
	 * Recalculate area which is the sum of area of all triangles
	 */
	public void updateAnnotationArea() {
		area = 0;
		for (Triangle t : mesh.getTriangles())
			area += t.getArea();
	}

}

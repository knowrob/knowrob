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
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.Vertex;

/**
 * @author Stefan Profanter
 * 
 */
public abstract class PrimitiveAnnotation extends MeshAnnotation {

	private float	area	= 0;

	/**
	 * @param annotationColor
	 */
	public PrimitiveAnnotation(Color annotationColor) {
		super(annotationColor);
	}

	public abstract void drawPrimitiveAnnotation(PGraphics g);

	public void fit() {
		updateAnnotationArea();
		fitAnnotation();
	}

	protected abstract void fitAnnotation();

	/**
	 * @return the area
	 */
	public float getArea() {
		if (area == 0)
			updateAnnotationArea();
		return area;
	}

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

	protected Vector3f getVerticesWithWeight(HashMap<Vertex, Float> vertices) {
		vertices.clear();
		Vector3f centroid = new Vector3f();

		for (Triangle t : mesh.getTriangles()) {
			float area = t.getArea();
			for (Vertex v : t.getPosition()) {
				float newArea = area;
				if (vertices.containsKey(v)) {

					newArea += vertices.get(v);
				} else {
					centroid.add(v);
				}
				vertices.put(v, newArea);
			}
		}

		int numberOfPoints = vertices.size();
		centroid.scale(1f / numberOfPoints);

		return centroid;

	}

	/**
	 * 
	 */
	public void updateAnnotationArea() {
		area = 0;
		for (Triangle t : mesh.getTriangles())
			area += t.getArea();
	}

}

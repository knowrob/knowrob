/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation.primitive;

import java.awt.Color;
import java.util.HashMap;
import java.util.HashSet;

import javax.vecmath.Matrix4f;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3f;

import processing.core.PGraphics;
import edu.tum.cs.vis.model.Model;
import edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation;
import edu.tum.cs.vis.model.util.Curvature;
import edu.tum.cs.vis.model.util.Vertex;

/**
 * Primitive annotation for concave or convex sphere.
 * 
 * @author Stefan Profanter
 * 
 */
public final class SphereAnnotation extends PrimitiveAnnotation<SphereAnnotation> {

	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= 4870579150170536881L;

	private final Sphere		sphere;

	/**
	 * Creates a new sphere annotation
	 * 
	 * @param curvatures
	 *            Map of curvatures for vertices
	 * @param model
	 *            parent model
	 * @param concave
	 *            is sphere concave or convex
	 */
	public SphereAnnotation(HashMap<Vertex, Curvature> curvatures, Model model, boolean concave) {
		super(SphereAnnotation.class, curvatures, model, concave ? new Color(0, 255, 0)
				: new Color(255, 0, 0));
		sphere = new Sphere(concave);
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation#drawAnnotation(processing.core.PGraphics)
	 */
	@Override
	public void drawPrimitiveAnnotation(PGraphics g, Color color) {

		sphere.draw(g, color == null ? getDrawColor() : color);
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation#fitAnnotation()
	 */
	@Override
	public boolean fitAnnotation() {

		HashMap<Vertex, Float> weights = new HashMap<Vertex, Float>();
		getVerticesWithWeight(weights);
		HashSet<Vertex> vert = new HashSet<Vertex>();
		Vector3f centroid = getVertices(vert);
		return sphere.fit(centroid, weights.keySet(), weights, mesh.getTriangles());
	}

	/**
	 * Get sphere center
	 * 
	 * @return the center
	 */
	public Vector3f getCenter() {
		return sphere.getCenter();
	}

	/**
	 * Get sphere center unscaled
	 * 
	 * @return the center
	 */
	public Tuple3f getCenterUnscaled() {
		return model.getUnscaled(getCenter());
	}

	/**
	 * Since a sphere is rotationally symmetric, the orientation does not make much sense (is chosen
	 * to be unit orientation here), but for compatibility reasons, this method is nevertheless
	 * implemented.
	 * 
	 * @return 4x4 pose matrix of the plane relative to the object centroid
	 */
	public Matrix4f getPoseMatrix() {
		return sphere.getPoseMatrix();
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation#getPrimitiveArea()
	 */
	@Override
	public float getPrimitiveArea() {
		return sphere.getArea();
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation#getPrimitiveAreaUnscaled()
	 */
	@Override
	public float getPrimitiveAreaUnscaled() {
		return model.getUnscaled(getPrimitiveArea());
	}

	/**
	 * get radius of sphere
	 * 
	 * @return the radius
	 */
	public float getRadius() {
		return sphere.getRadius();
	}

	/**
	 * get unscaled radius of sphere
	 * 
	 * @return the radius
	 */
	public float getRadiusUnscaled() {
		return model.getUnscaled(getRadius());
	}

	public Sphere getSphere() {
		return sphere;
	}

	/**
	 * Get sphere volume
	 * 
	 * @return the volume of the sphere
	 */
	public float getVolume() {
		return sphere.getVolume();
	}

	/**
	 * Get unscaled sphere volume
	 * 
	 * @return the unscaled volume of the sphere
	 */
	public float getVolumeUnscaled() {

		return model.getUnscaled(sphere.getVolume());
	}

	/**
	 * Is sphere concave or convex?
	 * 
	 * @return true if concave
	 */
	public boolean isConcave() {
		return sphere.isConcave();
	}

}

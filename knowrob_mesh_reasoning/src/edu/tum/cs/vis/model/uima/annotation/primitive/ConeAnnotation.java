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
import java.util.LinkedHashMap;
import java.util.Set;

import javax.vecmath.Matrix4f;
import javax.vecmath.Point3f;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3f;

import processing.core.PGraphics;
import edu.tum.cs.vis.model.Model;
import edu.tum.cs.vis.model.uima.annotation.HandleAnnotation;
import edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.Curvature;
import edu.tum.cs.vis.model.util.Vertex;

/**
 * Annotation for primitive type: Cone (convex and concave)
 * 
 * @author Stefan Profanter
 * 
 */
public final class ConeAnnotation extends PrimitiveAnnotation<ConeAnnotation> implements
		HandleAnnotation {

	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= -7420446109108464883L;

	private final Cone			cone;

	/**
	 * Create new cone annotation.
	 * 
	 * @param curvatures
	 *            Map of curvatures for vertices
	 * @param model
	 *            parent model
	 * @param concave
	 *            is cone concave or convex
	 * 
	 */
	public ConeAnnotation(HashMap<Vertex, Curvature> curvatures, Model model, boolean concave) {
		super(ConeAnnotation.class, curvatures, model, concave ? new Color(0, 125, 125)
				: new Color(255, 255, 0));
		cone = new Cone(concave);
	}

	public ConeAnnotation(HashMap<Vertex, Curvature> curvatures, Model model, boolean concave,
			Color annotationColor) {
		super(ConeAnnotation.class, curvatures, model, annotationColor);
		cone = new Cone(concave);
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation#drawAnnotation(processing.core.PGraphics)
	 */
	@Override
	public void drawPrimitiveAnnotation(PGraphics g, Color color) {

		cone.draw(g, color == null ? getDrawColor() : color);

	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation#fitAnnotation()
	 */
	@Override
	public boolean fitAnnotation() {

		LinkedHashMap<Vertex, Float> vertices = new LinkedHashMap<Vertex, Float>();
		Vector3f centroid = getVerticesWithWeight(vertices);
		return cone.fit(centroid, vertices.keySet(), vertices, mesh.getTriangles());
	}

	/**
	 * Get centroid of cone
	 * 
	 * @return the centroid
	 */
	public Point3f getCentroid() {
		return cone.getCentroid();
	}

	/**
	 * get centroid of cone at unscaled position
	 * 
	 * @return the centroid
	 */
	public Tuple3f getCentroidUnscaled() {
		return model.getUnscaled(getCentroid());
	}

	@Override
	public Cone getCone() {
		return cone;
	}

	/**
	 * get direction of cone. Direction is aligned with generating line and shows from centroid to
	 * small radius cap. Length of direction is half height of the cone (center to one end).
	 * 
	 * @return the direction
	 */
	public Vector3f getDirection() {
		return cone.getDirection();
	}

	/**
	 * 
	 * get direction (unscaled) of cone. Direction is aligned with generating line and shows from
	 * centroid to small radius cap. Length of direction is half height of the cone (center to one
	 * end).
	 * 
	 * @return the direction
	 */
	public Vector3f getDirectionUnscaled() {
		return new Vector3f(model.getUnscaled(getDirection()));
	}

	/**
	 * Get total unscaled height of cone from bottom cap to top cap which is 2*directionUnscaled.
	 * 
	 * @return unscaled height
	 */
	public float getHeightUnscaled() {
		return getDirectionUnscaled().length() * 2;
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.MeshAnnotation#getNeighborAnnotations(edu.tum.cs.vis.model.uima.cas.MeshCas)
	 */
	@Override
	public Set<ConeAnnotation> getNeighborAnnotations(MeshCas cas) {
		return getNeighborAnnotations(cas, ConeAnnotation.class);
	}

	/**
	 * Get pose matrix for cone.
	 * 
	 * @return 4x4 pose matrix of the plane relative to the object centroid
	 */
	public Matrix4f getPoseMatrix() {

		return cone.getPoseMatrix();
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation#getPrimitiveArea()
	 */
	@Override
	public float getPrimitiveArea() {
		return cone.getArea();
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation#getPrimitiveAreaUnscaled()
	 */
	@Override
	public float getPrimitiveAreaUnscaled() {

		return model.getUnscaled(getPrimitiveArea());
	}

	/**
	 * Get average radius of cone which is the average between small and large radius
	 * 
	 * @return the average radius
	 */
	public float getRadiusAvg() {
		return cone.getRadiusAvg();
	}

	/**
	 * Get average radius (unscaled) of cone which is the average between small and large radius
	 * 
	 * @return average radius unscaled
	 */
	public float getRadiusAvgUnscaled() {
		return model.getUnscaled(getRadiusAvg());
	}

	/**
	 * Get large radius, which is at the bottom of cone.
	 * 
	 * @return the radiusLarge
	 */
	public float getRadiusLarge() {
		return cone.getRadiusLarge();
	}

	/**
	 * Get large radius (unscaled), which is at the bottom of cone.
	 * 
	 * @return the radiusLarge
	 */
	public float getRadiusLargeUnscaled() {
		return model.getUnscaled(getRadiusLarge());
	}

	/**
	 * Get small radius, which is at the bottom of cone.
	 * 
	 * @return the radiusSmall
	 */
	public float getRadiusSmall() {
		return cone.getRadiusSmall();
	}

	/**
	 * Get small radius (unscaled), which is at the bottom of cone.
	 * 
	 * @return the radiusSmall
	 */
	public float getRadiusSmallUnscaled() {
		return model.getUnscaled(getRadiusSmall());
	}

	/**
	 * Get volume of cone.
	 * 
	 * @return the volume
	 */
	public float getVolume() {
		return cone.getVolume();
	}

	/**
	 * Get unscaled volume of cone.
	 * 
	 * @return the volume unscaled.
	 */
	public float getVolumeUnscaled() {

		return model.getUnscaled(getVolume());
	}

	/**
	 * Is cone concave or convex?
	 * 
	 * @return the concave
	 */
	public boolean isConcave() {
		return cone.isConcave();
	}

}

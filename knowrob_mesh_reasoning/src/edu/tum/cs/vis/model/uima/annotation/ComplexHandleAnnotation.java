/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Set;

import javax.vecmath.Matrix4f;
import javax.vecmath.Point3f;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3f;

import processing.core.PGraphics;
import edu.tum.cs.vis.model.Model;
import edu.tum.cs.vis.model.uima.annotation.primitive.Cone;
import edu.tum.cs.vis.model.uima.annotation.primitive.PlaneAnnotation;
import edu.tum.cs.vis.model.util.DrawSettings;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.Vertex;

/**
 * Annotation for a complex handle.
 * 
 * A complex handle is a convex part in the model.
 * 
 * 
 * @author Stefan Profanter
 * 
 */
public class ComplexHandleAnnotation extends DrawableAnnotation implements HandleAnnotation {

	/**
	 * generated
	 */
	private static final long				serialVersionUID		= 1983829446921229660L;

	@SuppressWarnings("rawtypes")
	private final Set<PrimitiveAnnotation>	primitiveAnnotations	= new HashSet<PrimitiveAnnotation>();

	private final Cone						cone;
	private final Model						model;
	/**
	 * allowed tolerance between normal vectors. Tolerance is indicated as the result of the dot
	 * product. The resulting angle is PI-acos(DOT_NORMAL_TOLERANCE), which is for 0.1 approx. 6
	 * degree.
	 */
	public final static float				DOT_NORMAL_TOLERANCE	= 0.1f;

	/**
	 * Create new complex handle annotation.
	 * 
	 * @param curvatures
	 *            Map of curvatures for vertices
	 * @param model
	 *            parent model
	 */
	public ComplexHandleAnnotation(Model model) {
		super(new Color(255, 41, 255, 200));
		this.model = model;
		cone = new Cone(false);
	}

	public void addAnnotation(@SuppressWarnings("rawtypes") PrimitiveAnnotation pa) {
		primitiveAnnotations.add(pa);

	}

	/**
	 * Check if the neighborHandle violates constraints for complex handle.
	 * 
	 */
	@SuppressWarnings("rawtypes")
	public boolean allowMerge(ComplexHandleAnnotation neighborHandle) {
		for (PrimitiveAnnotation pa : primitiveAnnotations) {
			if (!(pa instanceof PlaneAnnotation))
				continue;
			PlaneAnnotation plane1 = (PlaneAnnotation) pa;
			for (PrimitiveAnnotation pn : neighborHandle.primitiveAnnotations) {
				if (!(pn instanceof PlaneAnnotation))
					continue;
				PlaneAnnotation plane2 = (PlaneAnnotation) pn;

				// we have two planes. Now check if they violate the angle constraint. (the angle
				// between two planes must be bigger or equal to 180 degree).
				float dot = plane1.getPlaneNormal().dot(plane2.getPlaneNormal());
				// allow a small error
				if (dot > DOT_NORMAL_TOLERANCE)
					return false;
			}
		}
		return true;

	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.DrawableAnnotation#containsTriangle(edu.tum.cs.vis.model.util.Triangle)
	 */
	@Override
	public boolean containsTriangle(Triangle t) {
		for (@SuppressWarnings("rawtypes")
		PrimitiveAnnotation pa : primitiveAnnotations) {
			if (pa.containsTriangle(t))
				return true;
		}
		return false;
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.DrawableAnnotation#drawAnnotation(processing.core.PGraphics, edu.tum.cs.vis.model.util.DrawSettings)
	 */
	@Override
	protected void drawAnnotation(PGraphics g, DrawSettings drawSettings) {
		DrawSettings tmp = (DrawSettings) drawSettings.clone();
		if (tmp.getOverrideColor() == null)
			tmp = drawSettings.getTemporaryOverride(getDrawColor());
		tmp.forceDraw = true;
		for (@SuppressWarnings("rawtypes")
		PrimitiveAnnotation pa : primitiveAnnotations) {
			pa.draw(g, tmp);
		}
	}

	public void drawPrimitiveAnnotation(PGraphics g, Color color) {

		// Creating new color to set alpha to 255
		Color c = color == null ? getDrawColor() : color;
		cone.draw(g, new Color(c.getRed(), c.getGreen(), c.getBlue(), 255));

	}

	/**
	 * @return
	 */
	@SuppressWarnings("unchecked")
	public boolean fit() {

		LinkedHashMap<Vertex, Float> vertices = new LinkedHashMap<Vertex, Float>();
		Vector3f centroid = new Vector3f();
		List<Triangle> triangles = new ArrayList<Triangle>();

		for (@SuppressWarnings("rawtypes")
		PrimitiveAnnotation pa : primitiveAnnotations) {
			centroid.add(pa.getVerticesWithWeight(vertices));
			triangles.addAll(pa.mesh.getTriangles());
		}
		centroid.scale(1f / primitiveAnnotations.size());
		return cone.fit(centroid, vertices.keySet(), vertices, triangles);
	}

	public float getArea() {
		return cone.getArea();
	}

	/**
	 * Get value between > 0 for area coverage which indicates how good primitive annotation is fit
	 * into mesh. 1 indicates perfect fit, because area of triangles is exactly the same as area of
	 * primitive annotation.
	 * 
	 * @return value > 0
	 */
	@Override
	public float getAreaCoverage() {
		float trianglesArea = 0;
		for (@SuppressWarnings("rawtypes")
		PrimitiveAnnotation pa : primitiveAnnotations) {
			trianglesArea += pa.getArea();
		}
		return trianglesArea / cone.getArea();
	}

	public float getAreaUnscaled() {

		return model.getUnscaled(getArea());
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

	public Model getModel() {
		return model;
	}

	/**
	 * Get pose matrix for cone.
	 * 
	 * @return 4x4 pose matrix of the plane relative to the object centroid
	 */
	public Matrix4f getPoseMatrix() {

		return cone.getPoseMatrix();
	}

	@SuppressWarnings("rawtypes")
	public Set<PrimitiveAnnotation> getPrimitiveAnnotations() {
		return primitiveAnnotations;
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

	/**
	 * @param an
	 * @param t
	 */
	public void merge(ComplexHandleAnnotation an) {
		primitiveAnnotations.addAll(an.primitiveAnnotations);
	}
}

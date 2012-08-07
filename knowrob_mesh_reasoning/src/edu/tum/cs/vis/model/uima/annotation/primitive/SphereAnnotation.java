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

import javax.vecmath.Matrix3f;
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
public class SphereAnnotation extends PrimitiveAnnotation<SphereAnnotation> {

	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= 4870579150170536881L;

	/**
	 * For additional explanation see: <a
	 * href="http://www.profanter.me/pub/BachelorThesis/Thesis.pdf"
	 * >http://www.profanter.me/pub/BachelorThesis/Thesis.pdf</a>, chapter "3.3.3.2 Fit sphere"
	 * 
	 * @param v
	 * @param a
	 * @param b
	 * @param c
	 * @return
	 */
	private static float getLi(Vertex v, float a, float b, float c) {
		return (float) Math
				.sqrt(Math.pow(v.x - a, 2) + Math.pow(v.y - b, 2) + Math.pow(v.z - c, 2));
	}

	/**
	 * is sphere concave or convex
	 */
	private final boolean			concave;

	/**
	 * list of all vertices representing the sphere in mesh
	 */
	private final HashSet<Vertex>	vertices	= new HashSet<Vertex>();

	/**
	 * center of sphere
	 */
	private final Vector3f			center		= new Vector3f();

	/**
	 * radius of sphere
	 */
	private float					radius		= 0;

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
		this.concave = concave;
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation#drawAnnotation(processing.core.PGraphics)
	 */
	@Override
	public void drawPrimitiveAnnotation(PGraphics g) {

		g.sphereDetail(40);
		g.fill(getDrawColor().getRed(), getDrawColor().getGreen(), getDrawColor().getBlue(), 120);
		g.pushMatrix();
		g.translate(center.x, center.y, center.z);
		g.sphere(radius);
		g.popMatrix();
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation#fitAnnotation()
	 */
	@Override
	public void fitAnnotation() {
		/*
		 * Fitting sphere iteratively according to http://www.geometrictools.com/Documentation/LeastSquaresFitting.pdf
		 */

		Vector3f centroid = getVertices(vertices);

		float a = centroid.x;
		float b = centroid.y;
		float c = centroid.z;

		float a2 = Float.MAX_VALUE, b2 = Float.MAX_VALUE, c2 = Float.MAX_VALUE;

		int iterations = 0;

		while (Math.abs(a - a2) + Math.abs(b - b2) + Math.abs(c - c2) != 0 && iterations < 500) {

			a2 = a;
			b2 = b;
			c2 = c;

			float L = getL(a2, b2, c2);

			a = centroid.x + L * getLa(a2, b2, c2);
			b = centroid.y + L * getLb(a2, b2, c2);
			c = centroid.z + L * getLc(a2, b2, c2);

			iterations++;

		}

		center.x = a;
		center.y = b;
		center.z = c;

		radius = getL(a, b, c);

		// System.out.println("Center: " + center.toString() + " Radius: " + radius);

	}

	/**
	 * Get sphere center
	 * 
	 * @return the center
	 */
	public Vector3f getCenter() {
		return center;
	}

	/**
	 * Get sphere center unscaled
	 * 
	 * @return the center
	 */
	public Tuple3f getCenterUnscaled() {
		return model.getUnscaled(center);
	}

	/**
	 * For additional explanation see: <a
	 * href="http://www.profanter.me/pub/BachelorThesis/Thesis.pdf"
	 * >http://www.profanter.me/pub/BachelorThesis/Thesis.pdf</a>, chapter "3.3.3.2 Fit sphere"
	 * 
	 * @param a
	 * @param b
	 * @param c
	 * @return
	 */
	private float getL(float a, float b, float c) {
		float sum = 0;
		for (Vertex v : vertices) {
			sum += getLi(v, a, b, c);
		}
		return sum / vertices.size();
	}

	/**
	 * For additional explanation see: <a
	 * href="http://www.profanter.me/pub/BachelorThesis/Thesis.pdf"
	 * >http://www.profanter.me/pub/BachelorThesis/Thesis.pdf</a>, chapter "3.3.3.2 Fit sphere"
	 * 
	 * @param a
	 * @param b
	 * @param c
	 * @return
	 */
	private float getLa(float a, float b, float c) {
		float sum = 0;
		for (Vertex v : vertices) {
			sum += (a - v.x) / getLi(v, a, b, c);
		}
		return sum / vertices.size();
	}

	/**
	 * For additional explanation see: <a
	 * href="http://www.profanter.me/pub/BachelorThesis/Thesis.pdf"
	 * >http://www.profanter.me/pub/BachelorThesis/Thesis.pdf</a>, chapter "3.3.3.2 Fit sphere"
	 * 
	 * @param a
	 * @param b
	 * @param c
	 * @return
	 */
	private float getLb(float a, float b, float c) {
		float sum = 0;
		for (Vertex v : vertices) {
			sum += (b - v.y) / getLi(v, a, b, c);
		}
		return sum / vertices.size();
	}

	/**
	 * For additional explanation see: <a
	 * href="http://www.profanter.me/pub/BachelorThesis/Thesis.pdf"
	 * >http://www.profanter.me/pub/BachelorThesis/Thesis.pdf</a>, chapter "3.3.3.2 Fit sphere"
	 * 
	 * @param a
	 * @param b
	 * @param c
	 * @return
	 */
	private float getLc(float a, float b, float c) {
		float sum = 0;
		for (Vertex v : vertices) {
			sum += (c - v.z) / getLi(v, a, b, c);
		}
		return sum / vertices.size();
	}

	/**
	 * Since a sphere is rotationally symmetric, the orientation does not make much sense (is chosen
	 * to be unit orientation here), but for compatibility reasons, this method is nevertheless
	 * implemented.
	 * 
	 * @return 4x4 pose matrix of the plane relative to the object centroid
	 */
	public Matrix4f getPoseMatrix() {

		Matrix3f or = new Matrix3f();
		or.setIdentity();
		Matrix4f res = new Matrix4f(or, center, 1.0f);

		return res;
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation#getPrimitiveArea()
	 */
	@Override
	public float getPrimitiveArea() {
		return (float) (4f * Math.PI * (radius * radius));
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
		return radius;
	}

	/**
	 * get unscaled radius of sphere
	 * 
	 * @return the radius
	 */
	public float getRadiusUnscaled() {
		return model.getUnscaled(radius);
	}

	/**
	 * Get sphere volume
	 * 
	 * @return the volume of the sphere
	 */
	public float getVolume() {

		return (float) ((4 / 3 * Math.PI) * radius * radius * radius);
	}

	/**
	 * Get unscaled sphere volume
	 * 
	 * @return the unscaled volume of the sphere
	 */
	public float getVolumeUnscaled() {

		float r = getRadiusUnscaled();
		return (float) ((4 / 3 * Math.PI) * r * r * r);
	}

	/**
	 * Is sphere concave or convex?
	 * 
	 * @return true if concave
	 */
	public boolean isConcave() {
		return concave;
	}

}

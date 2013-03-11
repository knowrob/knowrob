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

import javax.vecmath.Matrix3f;
import javax.vecmath.Matrix4f;
import javax.vecmath.Vector3f;

import processing.core.PGraphics;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.Vertex;

/**
 * Represents a Sphere as PrimitiveShape
 * 
 * @author Stefan Profanter
 * 
 */
public class Sphere extends PrimitiveShape {

	/**
	 * For additional explanation see: <a
	 * href="http://www.profanter.me/pub/BachelorThesis/Thesis.pdf"
	 * >http://www.profanter.me/pub/BachelorThesis/Thesis.pdf</a>, chapter "3.3.3.2 Fit sphere"
	 * 
	 * @param a
	 * @param b
	 * @param c
	 * @param vertices
	 * @return
	 */
	private static float getL(float a, float b, float c, Set<Vertex> vertices) {
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
	private static float getLa(float a, float b, float c, Set<Vertex> vertices) {
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
	private static float getLb(float a, float b, float c, Set<Vertex> vertices) {
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
	private static float getLc(float a, float b, float c, Set<Vertex> vertices) {
		float sum = 0;
		for (Vertex v : vertices) {
			sum += (c - v.z) / getLi(v, a, b, c);
		}
		return sum / vertices.size();
	}

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
	private final boolean	concave;

	/**
	 * center of sphere
	 */
	private final Vector3f	center	= new Vector3f();

	/**
	 * radius of sphere
	 */
	private float			radius	= 0;

	public Sphere(boolean concave) {
		this.concave = concave;
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.primitive.PrimitiveShape#draw(processing.core.PGraphics, java.awt.Color)
	 */
	@Override
	public void draw(PGraphics g, Color drawColor) {
		g.sphereDetail(40);
		g.fill(drawColor.getRed(), drawColor.getGreen(), drawColor.getBlue(), 120);
		g.pushMatrix();
		g.translate(center.x, center.y, center.z);
		g.sphere(radius);
		g.popMatrix();
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.primitive.PrimitiveShape#fit(javax.vecmath.Vector3f, java.util.Map, java.util.List)
	 */
	@Override
	public boolean fit(Vector3f centroid, Set<Vertex> vertices, Map<Vertex, Float> weights,
			List<Triangle> triangles) {
		/*
		 * Fitting sphere iteratively according to http://www.geometrictools.com/Documentation/LeastSquaresFitting.pdf
		 */

		centroid.set(centroid);

		float a = centroid.x;
		float b = centroid.y;
		float c = centroid.z;

		float a2 = Float.MAX_VALUE, b2 = Float.MAX_VALUE, c2 = Float.MAX_VALUE;

		int iterations = 0;

		while (Math.abs(a - a2) + Math.abs(b - b2) + Math.abs(c - c2) != 0 && iterations < 500) {

			a2 = a;
			b2 = b;
			c2 = c;

			float L = getL(a2, b2, c2, vertices);

			a = centroid.x + L * getLa(a2, b2, c2, vertices);
			b = centroid.y + L * getLb(a2, b2, c2, vertices);
			c = centroid.z + L * getLc(a2, b2, c2, vertices);

			iterations++;

		}

		center.x = a;
		center.y = b;
		center.z = c;

		radius = getL(a, b, c, vertices);

		// System.out.println("Center: " + center.toString() + " Radius: " + radius);
		return true;

	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.primitive.PrimitiveShape#getArea()
	 */
	@Override
	public float getArea() {

		return (float) (4f * Math.PI * (radius * radius));
	}

	/**
	 * @return the center
	 */
	public Vector3f getCenter() {
		return center;
	}

	/**
	 * Since a sphere is rotationally symmetric, the orientation does not make much sense (is chosen
	 * to be unit orientation here), but for compatibility reasons, this method is nevertheless
	 * implemented.
	 * 
	 * @return 4x4 pose matrix of the plane relative to the object centroid
	 */
	@Override
	public Matrix4f getPoseMatrix() {

		Matrix3f or = new Matrix3f();
		or.setIdentity();
		Matrix4f res = new Matrix4f(or, center, 1.0f);

		return res;
	}

	/**
	 * @return the radius
	 */
	public float getRadius() {
		return radius;
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.primitive.PrimitiveShape#getVolume()
	 */
	@Override
	public float getVolume() {

		return (float) ((4 / 3 * Math.PI) * radius * radius * radius);
	}

	/**
	 * @return the concave
	 */
	public boolean isConcave() {
		return concave;
	}
}

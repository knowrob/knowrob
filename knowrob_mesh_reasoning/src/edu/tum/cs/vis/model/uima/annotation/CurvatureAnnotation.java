/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation;

import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;

import processing.core.PGraphics;
import edu.tum.cs.vis.model.util.Triangle;

/**
 * @author Stefan Profanter
 * 
 *         Mesh contains only one triangle for which this annotation is
 * 
 */
public class CurvatureAnnotation extends DrawObjectAnnotation {

	/**
	 * Direction and magnitude of maximum principal curvature at centroid of Triangle. kMax, kMin
	 * and NormalVector form a othonormal basis: N = kMax x kMin.
	 */
	private final Vector3f	kMax;
	/**
	 * Direction and magnitude of minimum principal curvature at centroid of Triangle. kMax, kMin
	 * and NormalVector form a othonormal basis: N = kMax x kMin.
	 */
	private final Vector3f	kMin;

	public CurvatureAnnotation(Triangle t, Vector3f kMin, Vector3f kMax) {
		object = t;
		this.kMax = kMax;
		this.kMin = kMin;
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.DrawableAnnotation#drawAnnotation(processing.core.PGraphics)
	 */
	@Override
	protected void drawAnnotation(PGraphics g) {
		Point3f cent = ((Triangle) object).getCentroid();

		Vector3f max = (Vector3f) kMax.clone();
		max.normalize();
		max.scale(0.005f);
		Vector3f min = (Vector3f) kMin.clone();
		min.normalize();
		min.scale(0.005f);

		g.stroke(0, 0, 255);
		g.line(cent.x - max.x, cent.y - max.y, cent.z - max.z, max.x + cent.x, max.y + cent.y,
				max.z + cent.z);

		g.stroke(0, 255, 255);
		g.line(cent.x - min.x, cent.y - min.y, cent.z - min.z, min.x + cent.x, min.y + cent.y,
				min.z + cent.z);

	}

	/**
	 * @return the kMax
	 */
	public Vector3f getkMax() {
		return kMax;
	}

	/**
	 * @return the kMin
	 */
	public Vector3f getkMin() {
		return kMin;
	}

}

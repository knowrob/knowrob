/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

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
	private final Vector3d	kMax;
	/**
	 * Direction and magnitude of minimum principal curvature at centroid of Triangle. kMax, kMin
	 * and NormalVector form a othonormal basis: N = kMax x kMin.
	 */
	private final Vector3d	kMin;

	public CurvatureAnnotation(Triangle t, Vector3d kMin, Vector3d kMax) {
		object = t;
		this.kMax = kMax;
		this.kMin = kMin;
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.DrawableAnnotation#drawAnnotation(processing.core.PGraphics)
	 */
	@Override
	protected void drawAnnotation(PGraphics g) {
		Point3d cent = ((Triangle) object).getCentroid();

		g.stroke(0, 0, 255);
		g.line((float) cent.x - (float) kMax.x, (float) cent.y - (float) kMax.y, (float) cent.z
				- (float) kMax.z, (float) kMax.x + (float) cent.x, (float) kMax.y + (float) cent.y,
				(float) kMax.z + (float) cent.z);

		g.stroke(0, 255, 255);
		g.line((float) cent.x - (float) kMin.x, (float) cent.y - (float) kMin.y, (float) cent.z
				- (float) kMin.z, (float) kMin.x + (float) cent.x, (float) kMin.y + (float) cent.y,
				(float) kMin.z + (float) cent.z);

	}

	/**
	 * @return the kMax
	 */
	public Vector3d getkMax() {
		return kMax;
	}

	/**
	 * @return the kMin
	 */
	public Vector3d getkMin() {
		return kMin;
	}

}

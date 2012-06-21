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

		Vector3d max = (Vector3d) kMax.clone();
		max.normalize();
		max.scale(0.005);
		Vector3d min = (Vector3d) kMin.clone();
		min.normalize();
		min.scale(0.005);

		g.stroke(0, 0, 255);
		g.line((float) cent.x - (float) max.x, (float) cent.y - (float) max.y, (float) cent.z
				- (float) max.z, (float) max.x + (float) cent.x, (float) max.y + (float) cent.y,
				(float) max.z + (float) cent.z);

		g.stroke(0, 255, 255);
		g.line((float) cent.x - (float) min.x, (float) cent.y - (float) min.y, (float) cent.z
				- (float) min.z, (float) min.x + (float) cent.x, (float) min.y + (float) cent.y,
				(float) min.z + (float) cent.z);

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

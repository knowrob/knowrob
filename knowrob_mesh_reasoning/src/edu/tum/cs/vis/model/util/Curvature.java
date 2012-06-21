/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.util;

import javax.vecmath.Vector3f;

/**
 * @author Stefan Profanter
 * 
 */
public class Curvature {

	private Vector3f	principleDirectionMax	= new Vector3f();

	private Vector3f	principleDirectionMin	= new Vector3f();

	private float		curvatureMax			= 0;
	private float		curvatureMin			= 0;

	private float		curvatureMinMax			= 0;

	/**
	 * @return the curvatureMax
	 */
	public float getCurvatureMax() {
		return curvatureMax;
	}

	/**
	 * @return the curvatureMin
	 */
	public float getCurvatureMin() {
		return curvatureMin;
	}

	/**
	 * @return the curvatureMinMax
	 */
	public float getCurvatureMinMax() {
		return curvatureMinMax;
	}

	/**
	 * @return the principleDirectionMax
	 */
	public Vector3f getPrincipleDirectionMax() {
		return principleDirectionMax;
	}

	/**
	 * @return the principleDirectionMin
	 */
	public Vector3f getPrincipleDirectionMin() {
		return principleDirectionMin;
	}

	/**
	 * @param curvatureMax
	 *            the curvatureMax to set
	 */
	public void setCurvatureMax(float curvatureMax) {
		this.curvatureMax = curvatureMax;
	}

	/**
	 * @param curvatureMin
	 *            the curvatureMin to set
	 */
	public void setCurvatureMin(float curvatureMin) {
		this.curvatureMin = curvatureMin;
	}

	/**
	 * @param curvatureMinMax
	 *            the curvatureMinMax to set
	 */
	public void setCurvatureMinMax(float curvatureMinMax) {
		this.curvatureMinMax = curvatureMinMax;
	}

	/**
	 * @param principleDirectionMax
	 *            the principleDirectionMax to set
	 */
	public void setPrincipleDirectionMax(Vector3f principleDirectionMax) {
		this.principleDirectionMax = principleDirectionMax;
	}

	/**
	 * @param principleDirectionMin
	 *            the principleDirectionMin to set
	 */
	public void setPrincipleDirectionMin(Vector3f principleDirectionMin) {
		this.principleDirectionMin = principleDirectionMin;
	}

}

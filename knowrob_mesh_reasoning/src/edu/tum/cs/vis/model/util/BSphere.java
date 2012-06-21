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
public class BSphere {
	private final float		r;
	private final Vector3f	center;

	public BSphere(float r, Vector3f center) {
		this.r = r;
		this.center = center;
	}

	/**
	 * @return the center
	 */
	public Vector3f getCenter() {
		return (Vector3f) center.clone();
	}

	/**
	 * @return the r
	 */
	public float getR() {
		return r;
	}
}

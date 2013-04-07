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
 * Class which represents a bounding sphere by center and radius.
 * 
 * @author Stefan Profanter
 * 
 */
public class BSphere {
	/**
	 * radius
	 */
	private final float		r;
	/**
	 * center
	 */
	private final Vector3f	center;

	/**
	 * Create new bounding sphere with radius <tt>r</tt> and center <tt>center</tt>
	 * 
	 * @param r
	 *            radius of bounding sphere
	 * @param center
	 *            center of bounding sphere
	 */
	public BSphere(float r, Vector3f center) {
		this.r = r;
		this.center = center;
	}

	/**
	 * Get center of bounding sphere
	 * 
	 * @return the center
	 */
	public Vector3f getCenter() {
		return (Vector3f) center.clone();
	}

	/**
	 * Get radius of bounding sphere.
	 * 
	 * @return the radius
	 */
	public float getR() {
		return r;
	}

	/**
	 * Get area of bounding sphere.
	 * 
	 * @return area
	 */
	public float getArea() {

		return (float) (4f * Math.PI * (r * r));
	}

	/**
	 * Get volume of bounding sphere
	 * 
	 * @return volume
	 */
	public float getVolume() {

		return (float) ((4 / 3 * Math.PI) * r * r * r);
	}
}

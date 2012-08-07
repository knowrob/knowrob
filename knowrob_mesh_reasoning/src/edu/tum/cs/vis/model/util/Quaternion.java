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
 * Quaternion used to represent rotation.
 * 
 * 
 * @author Stefan Profanter
 * @see <a
 *      href="http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation">http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation</a>
 * 
 */
public class Quaternion {
	/**
	 * Convert from Axis Angle.
	 * 
	 * To rotate through an angle θ (radiant), about the axis (unit vector) \vec{v}, use:
	 * 
	 * q = \cos(\theta/2) + \vec{v}\sin(\theta/2)
	 * 
	 * @param v
	 *            Rotation vector
	 * @param angle
	 *            angle to rotate
	 * @return resulting Quaternion
	 * 
	 **/
	public static Quaternion FromAxis(final Vector3f v, double angle) {
		Vector3f vn = new Vector3f(v);
		vn.normalize();

		float sinAngle = (float) Math.sin(angle * 0.5f);
		return new Quaternion(vn.x * sinAngle, vn.y * sinAngle, vn.z * sinAngle,
				(float) Math.cos(angle * 0.5f));
	}

	/**
	 * Multiplying a quaternion q with a vector v applies the q-rotation to v
	 * 
	 * @param vec
	 *            original vector to rotate
	 * @param rotQuat
	 *            rotation quaternion
	 * @param rotQuatConjugate
	 *            conjugated rotation quaternion
	 * @return new rotated vector
	 */
	public static Vector3f rotate(final Vector3f vec, final Quaternion rotQuat,
			final Quaternion rotQuatConjugate) {
		Vector3f vn = new Vector3f(vec);
		vn.normalize();

		Quaternion vecQuat = new Quaternion(vn.x, vn.y, vn.z, 0f), resQuat;

		resQuat = vecQuat.mult(rotQuatConjugate);
		resQuat = rotQuat.mult(resQuat);

		return (new Vector3f(resQuat.x, resQuat.y, resQuat.z));
	}

	/**
	 * w value for quaternion
	 */
	public float	w;
	/**
	 * x value for quaternion
	 */
	public float	x;

	/**
	 * y value for quaternion
	 */
	public float	y;

	/**
	 * z value for quaternion
	 */
	public float	z;

	/**
	 * Creates a new Quaternion with given values.
	 * 
	 * @param x
	 *            x value
	 * @param y
	 *            y value
	 * @param z
	 *            z value
	 * @param w
	 *            w value
	 */
	public Quaternion(float x, float y, float z, float w) {
		this.w = w;
		this.z = z;
		this.y = y;
		this.x = x;
	}

	/**
	 * We need to get the inverse of a quaternion to properly apply a quaternion-rotation to a
	 * vector. The conjugate of a quaternion is the same as the inverse, as long as the quaternion
	 * is unit-length
	 * 
	 * @return conjugated quaternion
	 **/
	public Quaternion getConjugate() {
		return new Quaternion(-x, -y, -z, w);
	}

	/**
	 * Multiplying q1 with q2 applies the rotation q2 to q1
	 * 
	 * @param rq
	 *            rotate this quaternion by rq and return result
	 * @return get rotated quaterion.
	 **/
	public Quaternion mult(final Quaternion rq) {
		// the constructor takes its arguments as (x, y, z, w)
		return new Quaternion(w * rq.x + x * rq.w + y * rq.z - z * rq.y, w * rq.y + y * rq.w + z
				* rq.x - x * rq.z, w * rq.z + z * rq.w + x * rq.y - y * rq.x, w * rq.w - x * rq.x
				- y * rq.y - z * rq.z);
	}

	/**
	 * Normalizing a quaternion works similar to a vector. This method will not do anything if the
	 * quaternion is close enough to being unit-length. define TOLERANCE as something small like
	 * 0.00001f to get accurate results
	 */
	public void normalize() {
		double TOLERANCE = 0.00001f;
		// Don't normalize if we don't have to
		double mag2 = w * w + x * x + y * y + z * z;
		if (Math.abs(mag2) > TOLERANCE && Math.abs(mag2 - 1.0f) > TOLERANCE) {
			double mag = Math.sqrt(mag2);
			w /= mag;
			x /= mag;
			y /= mag;
			z /= mag;
		}
	}
}

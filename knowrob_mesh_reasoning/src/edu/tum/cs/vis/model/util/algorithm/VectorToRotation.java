/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.util.algorithm;

import javax.vecmath.Vector3f;

import org.ejml.simple.SimpleMatrix;

/**
 * Converts vector and rotation angle to rotation matrix.
 * 
 * @author Stefan Profanter
 * 
 */
public class VectorToRotation {

	/**
	 * Converts vector and rotation angle to rotation matrix.
	 * 
	 * @param rot
	 *            rotation angle in radiant
	 * @param vec
	 *            rotation vector
	 * @return rotation matrix
	 */
	public static SimpleMatrix getRotationMatrix(double rot, Vector3f vec) {
		SimpleMatrix mat = new SimpleMatrix(3, 3);

		double c = Math.cos(rot);
		double s = Math.sin(rot);

		mat.set(0, 0, c + Math.pow(vec.x, 2) * (1 - c));
		mat.set(0, 1, vec.x * vec.y * (1 - c) - vec.z * s);
		mat.set(0, 2, vec.x * vec.z * (1 - c) + vec.y * s);

		mat.set(1, 0, vec.y * vec.x * (1 - c) + vec.z * s);
		mat.set(1, 1, c + Math.pow(vec.y, 2) * (1 - c));
		mat.set(1, 2, vec.y * vec.z * (1 - c) - vec.x * s);

		mat.set(2, 0, vec.z * vec.x * (1 - c) - vec.y * s);
		mat.set(2, 1, vec.z * vec.y * (1 - c) + vec.x * s);
		mat.set(2, 2, c + Math.pow(vec.z, 2) * (1 - c));

		return mat;

	}

}

/*******************************************************************************
 * Copyright (c) 2014 Andrei Stoica. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Andrei Stoica - initial API and implementation during Google Summer of Code 2014
 ******************************************************************************/

package edu.tum.cs.vis.model.util;

/**
 * Class that implements some parameters used in the CAD mesh parsing and initialization of
 * components and their neighboring relations according to the model geometry.
 * 
 * @author Andrei Stoica
 */
public class Thresholds {
	/**
	 * Maximum distance tolerance undere the Euclidean geometry for two 3D points to be considered
	 * one and the same. This parameter is used in analyzing the neighboring relations among the
	 * triangles that form the faces of the mesh.
	 */
	public static final float	DISTANCE_TOLERANCE	= 1e-5f;

	/**
	 * Maximum angle tolerance (in degrees) for deciding whether or not two lines are parallel which
	 * is applied to the acute angle formed by the directions of the two lines.
	 */
	public static final float	ANGLE_TOLERANCE		= 0.25f;
}